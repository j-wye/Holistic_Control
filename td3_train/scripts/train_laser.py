from ament_index_python.packages import get_package_share_directory
import rclpy
import rclpy.logging
from rclpy.node import Node
import numpy as np
import threading, itertools, math, torch, argparse, datetime, os
from TD3 import TD3
from torch.utils.tensorboard import SummaryWriter
from replay_memory import ReplayMemory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, Quaternion
from tf2_msgs.msg import TFMessage
from rclpy.clock import Clock
from std_msgs.msg import Float32MultiArray
import cv2

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "physx_use_gpu": True})

import omni
from omni.isaac.core import World, SimulationContext
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils import stage as stage_utils
from omni.isaac.core.objects import DynamicCylinder
from pxr import UsdGeom, UsdPhysics, Gf, PhysxSchema, Sdf

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

world = World(stage_units_in_meters = 1.0, physics_dt = 1/500, rendering_dt = 1/60)
scripts_path = os.path.abspath(os.path.dirname(__file__))
pkg_path = os.path.dirname(scripts_path)
usd_file_path = os.path.join(pkg_path, "usd/train_model.usd")
omni.usd.get_context().open_stage(usd_file_path)

simulation_app.update()
while stage_utils.is_stage_loading():
    simulation_app.update()

simulation_context = SimulationContext()
simulation_context.initialize_physics()
simulation_context.play()

class IsaacEnv(Node):
    def __init__(self, size):
        super().__init__('env')
        self.size = size
        self.pose = Point(x=0.0, y=0.0, z=0.0)
        self.orient = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.qos = QoSProfile(depth=10, reliability = ReliabilityPolicy.RELIABLE, durability = DurabilityPolicy.VOLATILE)
        self.tf_subscription = self.create_subscription(TFMessage, '/tf', self.tf_cb, self.qos)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', self.qos)
        self.publisher_cylinder_coords = self.create_publisher(Float32MultiArray, '/cylinder_coords', self.qos)
    
        # About Laser Data
        self.laser_dim = 1024
        self.laser_data = np.zeros(self.laser_dim)
        self.laser_subscription = self.create_subscription(LaserScan, '/lidar/scan', self.laser_cb, self.qos)
        
        # About Cmd Vel
        self.cmd_vel_timer = self.create_timer(0.05, self.timer_cb)
        self.current_cmd_vel = Twist()

        # About initial parameters
        self.next_state = np.zeros(self.laser_dim + 2)
        self.vel = 1.0
        self.omega = 0.0
        
        # About Constraints
        self.start = [0.0, 0.0] 
        self.goal = [size, size]
        self.start_to_goal_dist = math.sqrt((self.goal[0] - self.start[0])**2 + (self.goal[1] - self.start[1])**2)
        self.prev_dist = 0
        self.max_vel = 1.0
        self.vel_constraint = 0.5
        self.max_omega = 0.7853981633974483
        self.COLLISION_DIST = 0.4
        self.collision_bool = False
        
        self.cylinder_coords = []
        self.initialized_cv_window = False
    
    def tf_cb(self, msg):
        for transform in msg.transforms:
            transform.header.stamp = Clock().now().to_msg()
            if transform.header.frame_id == 'base_link' and transform.child_frame_id == 'base_body':
                self.pose = transform.transform.translation
                self.orient = transform.transform.rotation
                self.yaw = -math.atan2(2*(self.orient.x*self.orient.y + self.orient.z*self.orient.w), (self.orient.x**2 - self.orient.y**2 - self.orient.z**2 + self.orient.w**2))
            else:
                continue

    def laser_cb(self, msg):
        laser_points = msg.ranges
        filtered_points = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i, point in enumerate(laser_points):
            angle = angle_min + i * angle_increment
            if -math.pi/2 <= angle and len(filtered_points) < self.laser_dim:
                filtered_points.append(point)
        valid_points = [p for p in filtered_points if p > 0.2]
        local_min_dist = min(valid_points) if valid_points else 10
        self.laser_data = np.array(filtered_points, dtype=np.float32)
        self.collision_bool = False if local_min_dist > self.COLLISION_DIST else True
        self.debug_draw_points_cv2_scan(filtered_points, angle_increment)
    
    def debug_draw_points_cv2_scan(self, filtered_points, angle_increment):
        if not self.initialized_cv_window:
            cv2.namedWindow("Filtered Lidar Debug", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Filtered Lidar Debug", 1000, 1000)
            self.initialized_cv_window = True
        
        canvas_size = 1000
        canvas = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
        center_x, center_y = canvas_size // 2, canvas_size // 2
        scale = 200
        
        cv2.circle(canvas, (center_x, center_y), 2, (0, 0, 255), 2)
        
        for i, point in enumerate(filtered_points):
            angle = -math.pi/2 + i * angle_increment
            x = point * math.cos(angle)
            y = point * math.sin(angle)
            px = int(center_x - y * scale)
            py = int(center_y - x * scale)
            if point > 0.3:
                if angle < 0:
                    cv2.circle(canvas, (px, py), 2, (255, 0, 0), -1)
                else: cv2.circle(canvas, (px, py), 2, (0, 0, 255), -1)

        cv2.imshow("Filtered Lidar Debug", canvas)
        cv2.waitKey(1)
    
    def timer_cb(self):
        self.publisher_cmd_vel.publish(self.current_cmd_vel)
    
    def reset(self):
        simulation_context.reset()
        self.prev_dist = 0
        self.cylinder_coords = []
        
        wall_height = 3.0
        wall_thickness = 1.0
        bias = 2
        for wall_name in ["bottom", "top", "left", "right"]:
            prim_path = f"/World/Walls/wall_{wall_name}"
            wall_xform = XFormPrim(prim_path=prim_path)
            cube_geom = UsdGeom.Cube.Define(world.stage, f"{prim_path}/Cube")
            cube_geom.GetSizeAttr().Set(1.0)
            UsdPhysics.RigidBodyAPI.Apply(world.stage.GetPrimAtPath(prim_path))
            UsdPhysics.CollisionAPI.Apply(world.stage.GetPrimAtPath(prim_path))

            if wall_name == "bottom":
                position = [ -bias + wall_thickness / 2, self.size / 2, wall_height / 2]
                scale = [wall_thickness, self.size + 2 * bias, wall_height]
            elif wall_name == "top":
                position = [self.size + bias - wall_thickness / 2, self.size / 2, wall_height / 2]
                scale = [wall_thickness, self.size + 2 * bias, wall_height]
            elif wall_name == "left":
                position = [self.size / 2, self.size + bias - wall_thickness / 2, wall_height / 2]
                scale = [self.size + bias, wall_thickness, wall_height]
            elif wall_name == "right":
                position = [self.size / 2, -bias + wall_thickness / 2, wall_height / 2]
                scale = [self.size + bias, wall_thickness, wall_height]

            wall_xform.set_world_pose(position)
            wall_xform.set_local_scale(scale)
        
        cylinder_xforms = []
        one_side_length = 4
        region_size = self.size / one_side_length
        cylinder_radius = self.size / 50
        cylinder_height = 2.0
        
        for i in range(1, int(one_side_length ** 2) - 1, 3):
            prim_path = f"/World/Static_Obstacles/Cylinder{i}"
            try:
                cylinder_xform = XFormPrim(prim_path=prim_path)
                cylinder_xforms.append(cylinder_xform)
                
                cylinder_geom = UsdGeom.Cylinder.Define(world.stage, f"{prim_path}/Geom{i}")
                cylinder_geom.GetRadiusAttr().Set(cylinder_radius)
                cylinder_geom.GetHeightAttr().Set(cylinder_height)
                
                cylinder_prim = world.stage.GetPrimAtPath(prim_path)
                UsdPhysics.RigidBodyAPI.Apply(cylinder_prim)
                UsdPhysics.CollisionAPI.Apply(cylinder_prim)
                
                region_number_y, region_number_x = divmod(i, one_side_length)
                local_box_coord = np.random.uniform(0, region_size, size=2)
                coords = [local_box_coord[0] + region_number_x * region_size,
                        local_box_coord[1] + region_number_y * region_size,
                        cylinder_height/2]
                cylinder_xform.set_world_pose(coords, [0.0, 0.0, 0.0, 1.0])
                self.cylinder_coords.append(coords)
                
            except Exception as e:
                self.get_logger().error(f'Error for creating/resetting Cylinder{i}: {str(e)}')
        
        msg = Float32MultiArray()
        data = []
        for coords in self.cylinder_coords:
            data.extend(coords[:2])
        msg.data = data
        self.publisher_cylinder_coords.publish(msg)
        
        self.done = False
        self.next_state = np.zeros(self.laser_dim + 2)

        return self.next_state

    def step(self, action, time_steps, max_episode_steps):
        self.done = False
        
        if action[0] > self.max_omega:
            self.omega = self.max_omega
        elif action[0] < -self.max_omega:
            self.omega = -self.max_omega
        else:
            self.omega = action[0]
        
        self.vel = self.max_vel - (abs(self.omega) / self.max_omega) * (self.max_vel - self.vel_constraint)
        self.current_cmd_vel.linear.x = self.vel
        self.current_cmd_vel.angular.z = float(self.omega)

        for _ in range(20):
            simulation_context.step(render=True)

        distance_to_the_goal = math.sqrt((self.goal[0] - self.pose.x)**2 + (self.goal[1] - self.pose.y)**2)
        
        self.next_state = np.zeros(self.laser_dim + 2)
        self.next_state[:self.laser_dim] = self.laser_dim
        self.next_state[-2] = distance_to_the_goal
        self.next_state[-1] = self.yaw
        
        self.prev_dist = distance_to_the_goal
        reward, self.done = self.reward_function(time_steps, max_episode_steps)

        return self.next_state, reward, self.done
    
    def reward_function(self, time_steps, max_episode_steps):
        sigma = self.start_to_goal_dist / self.size
        rel_vec = [self.goal[0] - self.pose.x, self.goal[1] - self.pose.y]
        norm_rel_vec = math.sqrt(rel_vec[0]**2 + rel_vec[1]**2)
        
        coef = 100
        share, remain = divmod(norm_rel_vec, sigma)
        max_share, _ = divmod(self.start_to_goal_dist, sigma)
        
        min_reward_per_region = math.exp(-0.5)
        reward_per_region_amplitude = 1 - min_reward_per_region
        
        region_num = max_share - share - 1 if share < max_share - 1 else -100
        reward = coef * ((math.exp(-0.5 * (remain / sigma) ** 2) - min_reward_per_region) + region_num * reward_per_region_amplitude)
        
        if (self.collision_bool == True):
            self.get_logger().info(f"CLASH. DONE.")
            self.done = True
            reward = -1000
        elif (time_steps >= max_episode_steps):
            self.get_logger().info(f"TIMEOUT. DONE.")
            self.done = True
            reward = -100000
        elif ((self.goal[0] - 1) < self.pose.x < (self.goal[0] + 1) and (self.goal[1] - 1) < self.pose.y < (self.goal[1] + 1)):
            self.get_logger().info(f"GOAL. DONE.")
            self.done = True
            reward = 100000
        
        return reward, self.done
        
if __name__ == '__main__':
    rclpy.init(args=None)

    parser = argparse.ArgumentParser(description='TD3 Args')
    parser.add_argument('--env-name', default="obstacle_avoidance", help='quadruped_isaac')
    parser.add_argument('--policy', default="Gaussian", help='Policy Type: Gaussian | Deterministic (default: Gaussian)')
    parser.add_argument('--eval', type=bool, default=True, help='Evaluates a policy a policy every 10 episode (default: True)')
    parser.add_argument('--gamma', type=float, default=0.99, metavar='G', help='discount factor for reward (default: 0.99)')
    parser.add_argument('--tau', type=float, default=0.005, metavar='G', help='target smoothing coefficient(τ) (default: 0.005)')
    parser.add_argument('--lr', type=float, default=0.0003, metavar='G', help='learning rate (default: 0.0003)')
    parser.add_argument('--alpha', type=float, default=0.2, metavar='G', help='Temperature parameter α determines the relative importance of the entropy term against the reward (default: 0.2)')
    parser.add_argument('--policy_freq', type=bool, default=2, metavar='G', help='policy frequency for TD3 updates')
    parser.add_argument('--seed', type=int, default=123456, metavar='N', help='random seed (default: 123456)')
    parser.add_argument('--batch_size', type=int, default=128, metavar='N', help='batch size (default: 256)')
    parser.add_argument('--num_steps', type=int, default=12001, metavar='N', help='maximum number of steps (default: 5000)')
    parser.add_argument('--hidden_size', type=int, default=128, metavar='N', help='hidden size (default: 256)')
    parser.add_argument('--updates_per_step', type=int, default=1, metavar='N', help='model updates per simulator step (default: 1)')
    parser.add_argument('--start_steps', type=int, default=1000, metavar='N',help='Steps sampling random actions (default: 10000)')
    parser.add_argument('--target_update_interval', type=int, default=1, metavar='N', help='Value target update per no. of updates per step (default: 1)')
    parser.add_argument('--replay_size', type=int, default=500000, metavar='N', help='size of replay buffer (default: 10000000)')
    parser.add_argument('--automatic_entropy_tuning', type=bool, default=False, metavar='G', help='Automaically adjust α (default: False)')
    parser.add_argument('--cuda', action="store_true", help='run on CUDA (default: False)')
    args = parser.parse_args()

    env = IsaacEnv(5.0)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(env)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = env.create_rate(2)

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    # Agent
    action_space = np.ones(1)
    env.get_logger().info(f"state_space:{len(env.next_state)}")
    env.get_logger().info(f"action_space:{action_space}")
    env.get_logger().info(f"args : {args}")
    agent = TD3(len(env.next_state), action_space, args)
    file_name = "checkpoints"

    #Tensorboard
    writer = SummaryWriter('runs/{}_TD3_{}_{}_{}'.format(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"), args.env_name, args.policy, "autotune" if args.automatic_entropy_tuning else ""))

    # Memory
    memory = ReplayMemory(args.replay_size, args.seed)
    memory.clear()
    # Training Loop
    total_numsteps = 0
    # updates = 0
    max_episode_steps = 12000
    
    try:
        while rclpy.ok():
            for i_episode in itertools.count(1):
                episode_reward = 0
                episode_steps = 0
                done = False
                
                state = env.reset()
                
                while not done:
                    print(f"episode step:{episode_steps}, in total:{total_numsteps}/{args.num_steps}")
                    if args.start_steps > total_numsteps:
                        action = np.random.uniform(0, 0.7853981633974483, 1)
                    else:
                        action = agent.select_action(state)

                    if len(memory) > args.batch_size:
                        
                        for i in range(args.batch_size):
                            av_critic_loss, av_Q, max_Q = agent.update_parameters(memory, args.batch_size,i)

                            writer.add_scalar('av_critic_loss', av_critic_loss, total_numsteps)
                            writer.add_scalar('av_Q', av_Q, total_numsteps)
                            writer.add_scalar('max_Q', max_Q, total_numsteps)
                            
                    next_state, reward, done = env.step(action, episode_steps, max_episode_steps) # Step
                    episode_steps += 1
                    total_numsteps += 1
                    episode_reward += reward

                    mask = 1 if episode_steps == max_episode_steps else float(not done)
                    memory.push(state, action, reward, next_state, mask)

                    state = next_state

                if total_numsteps > args.num_steps:
                    break

                writer.add_scalar('reward/train', episode_reward, i_episode)
                print("Episode: {}, total numsteps: {}, episode steps: {}, reward: {}".format(i_episode, total_numsteps, episode_steps, round(episode_reward, 2)))

                if i_episode % 10 == 0 and args.eval is True:
                    avg_reward = 0.
                    episodes = 10
                    for i  in range(episodes):
                        print(f"eval episode{i}")
                        state = env.reset()
                        episode_reward = 0
                        eval_steps = 0
                        done = False
                        while not done:
                            action = agent.select_action(state)

                            next_state, reward, done = env.step(action, eval_steps, max_episode_steps)
                            episode_reward += reward

                            eval_steps += 1
                            state = next_state
                        avg_reward += episode_reward
                    avg_reward /= episodes

                    writer.add_scalar('avg_reward/test', avg_reward, i_episode)

                    print("--------------------------------------------------------------------------------")
                    print("Test Episodes: {}, Avg. Reward: {}".format(episodes, round(avg_reward, 2)))
                    print("--------------------------------------------------------------------------------")
                    
                if i_episode % 20 == 0:
                    agent.save_checkpoint(pkg_path, file_name, i_episode)

    except KeyboardInterrupt:
        print("KeyboardInterrupt")
