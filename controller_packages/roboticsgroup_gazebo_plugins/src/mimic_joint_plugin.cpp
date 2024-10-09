#include <rclcpp/rclcpp.hpp>
#include <control_toolbox/pid.hpp>
#include <roboticsgroup_gazebo_plugins/mimic_joint_plugin.h>

namespace gazebo {

    void MimicJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        model_ = _parent;
        world_ = model_->GetWorld();

        // Create ROS2 node
        auto node = rclcpp::Node::make_shared("mimic_joint_plugin");
        rclcpp::Logger logger = node->get_logger();

        // Check for required parameters
        if (!_sdf->HasElement("joint_name") || !_sdf->HasElement("mimic_joint_name"))
        {
            RCLCPP_ERROR(logger, "joint_name or mimic_joint_name not provided. Plugin could not be loaded.");
            return;
        }

        joint_name_ = _sdf->Get<std::string>("joint_name");
        mimic_joint_name_ = _sdf->Get<std::string>("mimic_joint_name");

        joint_ = model_->GetJoint(joint_name_);
        mimic_joint_ = model_->GetJoint(mimic_joint_name_);

        if (!joint_)
        {
            RCLCPP_ERROR(logger, "Joint %s not found. MimicJointPlugin could not be loaded.", joint_name_.c_str());
            return;
        }

        if (!mimic_joint_)
        {
            RCLCPP_ERROR(logger, "Mimic joint %s not found. MimicJointPlugin could not be loaded.", mimic_joint_name_.c_str());
            return;
        }

        // Check for PID control
        if (_sdf->HasElement("has_pid"))
        {
            has_pid_ = _sdf->Get<bool>("has_pid");
            if (has_pid_)
            {
                double p = 0.1, i = 0.01, d = 0.01;
                double i_max = 0, i_min = 0;
                if (_sdf->HasElement("p")) p = _sdf->Get<double>("p");
                if (_sdf->HasElement("i")) i = _sdf->Get<double>("i");
                if (_sdf->HasElement("d")) d = _sdf->Get<double>("d");
                pid_.setGains(p, i, d, i_max, i_min, false);  // Use 6 parameters in setGains
            }
        }

        // Load other parameters
        if (_sdf->HasElement("multiplier"))
            multiplier_ = _sdf->Get<double>("multiplier");
        if (_sdf->HasElement("offset"))
            offset_ = _sdf->Get<double>("offset");
        if (_sdf->HasElement("max_effort"))
            max_effort_ = _sdf->Get<double>("max_effort");
        if (_sdf->HasElement("sensitiveness"))
            sensitiveness_ = _sdf->Get<double>("sensitiveness");

        RCLCPP_INFO(logger, "MimicJointPlugin loaded successfully! Joint: %s, Mimic joint: %s, Multiplier: %f, Offset: %f, MaxEffort: %f, Sensitiveness: %f", 
                    joint_name_.c_str(), mimic_joint_name_.c_str(), multiplier_, offset_, max_effort_, sensitiveness_);

        // Connect the update function to be called on world update
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MimicJointPlugin::UpdateChild, this));
    }

    void MimicJointPlugin::UpdateChild()
    {
#if GAZEBO_MAJOR_VERSION >= 8
        static rclcpp::Duration period = rclcpp::Duration::from_seconds(world_->Physics()->GetMaxStepSize());
#else
        static rclcpp::Duration period = rclcpp::Duration::from_seconds(world_->GetPhysicsEngine()->GetMaxStepSize());
#endif

        // Set the mimic joint's angle based on the joint's angle
#if GAZEBO_MAJOR_VERSION >= 8
        double angle = joint_->Position(0) * multiplier_ + offset_;
        double a = mimic_joint_->Position(0);
#else
        double angle = joint_->GetAngle(0).Radian() * multiplier_ + offset_;
        double a = mimic_joint_->GetAngle(0).Radian();
#endif

        if (fabs(angle - a) >= sensitiveness_)
        {
            if (has_pid_)
            {
                if (a != a)  // Check for NaN
                    a = angle;

                double error = angle - a;
                double effort = std::clamp(pid_.computeCommand(error, period.seconds()), -max_effort_, max_effort_);
                mimic_joint_->SetForce(0, effort);
            }
            else
            {
#if GAZEBO_MAJOR_VERSION >= 9
                mimic_joint_->SetPosition(0, angle, true);
#elif GAZEBO_MAJOR_VERSION > 2
                RCLCPP_WARN_ONCE(rclcpp::get_logger("mimic_joint_plugin"), "Using Joint::SetPosition method, link velocity will not be preserved.");
                mimic_joint_->SetPosition(0, angle);
#else
                mimic_joint_->SetAngle(0, math::Angle(angle));
#endif
            }
        }
    }
    
    GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin);
}

