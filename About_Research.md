# Research Topic
## Holistic Control-Based Real-Time SLAM Framework for Mobile Manipulation System

#### Research Topic Key Idea & Thought Experiment
    
- 내 연구주제는 Holistic Control을 통해 real-time SLAM Framework를 개발

- Mobile base와 6DOF의 manipulator arm의 결합으로 이루어진 mobile manipulator를 HW 플랫폼으로 사용해 Indoor Environment에서 실험 진행
    - Future work : 
        - Natural language로 input command를 제공하면 end effector를 활용해 additional task performing이 가능하도록 설계
        - Indoor에서 Outdoor environment로 확장
- Eqquied Sensor :
    - Jetson Orin AGX 64GB 2EA, Base Camera(RGB-D), Gripper Camera(RGB-D)
        - Mobile base에 장착된 2D LiDAR는 가능하면 사용하지 않고 vSLAM으로 진행하기 위해 사용하지 않음
- Advantage :
    - Efficiency, Accurate, Precise, Fast Execution Time
- SLAM with relatively fixed base camera and dynamic gripper camera
    - Mobile base의 높이와 제한된 움직임과 시야각의 한계로 인해 탐지하지 못하는 영역을 manipulator arm을 통해 더 많은 정보를 인지할 수 있고 6DOF의 자유로운 움직임으로 빠르게 많은 정보를 습득하고 인지할 수 있음
        - e.g. cupboard, ceiling, back of object, above desk
- Finally, show Map through 3D Reconstruction with real-time image data on Host Computer
- Metric:
    - Final execution time, Map Accuracy, Variable Features
- Applications:
    - Use at complex and large scene such as factory, warehouse, smart factory, and logistics automation to efficiently manage indoor environment
    - Useful map can be integrated with natural language and gripper task
    - Not only mobile manipulators but humanoid robots can use map data for variable tasks


#### Network Design

- Input Sensor Data :
    - Pointcloud from Base camera and Gripper Camera

- **SN**(SLAM Network) :
    - RTAB-Map과 OctoMap의 integration으로 memory efficiency가 높은 구조를 선택
    - Real-time을 충족시키기 위해 computational cost가 낮은 모델을 선정하고 rgb image 없이 only depth(pointcloud)로 vSLAM 진행

- **EN**(Exploration Network) :
    - SN에서 current pose data와 map data를 input으로 받아 explorable candidate set을 output으로 설정
    - Output = {(x_i, y_i, z_i, H_i) | i = 1, ~ n}
        - **ID**(Information Density) 자리에 원래 IG를 사용하려고 했지만, IG는 해당 위치에서 카메라가 관측할 것으로 예상하는 값을 말하고 이는 "관측 시나리오"까지 포함해서 성능 극대화에는 더 좋지만, 계산량이 늘어나고 구현 복잡도가 증가함
        - 우선 초기 연구에서는 IG를 사용하지 않고 ID나 H(Entropy)를 사용해서 진행할 예정

- **HCN**(Holistic Control Network) :
    - EN에서 explorable candidate set을 input으로 받아 v, $$omega$$



```
아직 구상중

bc와 gc를 input sensor데이터로 사용
이 데이터의 pointcloud로 RTAB-Map을 통해 vSLAM 진행
RTAB-Map과 OctoMap의 integration으로 효율적인 Map 구성

current pose data와 map data를 기반으로 explorable candidate를 select하고 이를 통해 candidate set으로 설정
=> candidate set을 HCN에 input으로 전달
=> HCN에서 candidate set을 어떻게 exploration할지 path planning을 진행 (어떤 위치로 어떤 순서로 움직일지)
=> 해당 과정을 진행하기 위해 mobile base와 manipulator arm이 어떻게 움직일지 holistic control이 되도록 control logic에 해당하는 HCN 설계


REWARD :
1. Entropy 감소
2. Viewpoint Diversity 증가 : 다양한 정보를 많이 얻도록 움직임 장려
3. Holistic 동작성(Arm과 Base가 조화롭게 동작하도록, e.g. 겹치는 frame이 없도록 혹은 arm이 자유롭게 움직일수록 보상)
4. Collision Avoidance : 안전성 필수!
5. Energy Efficiency : 불필요한 움직임 최소화
6. Efficiency in reaching new states/goal : Path Planning Metric (다음 위치를 선정하고 움직이면서 제대로 움직이고 있는지에 대한 평가 요소)
7. Final end time : 전체 episode 소요시간 단축
```