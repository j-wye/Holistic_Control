## How to Use Isaac Sim & Tips

- Move(W), Rotate(E), Scale(R)을 누르거나 키보드로 입력해서 작동 가능
    - 한 번 입력하거나 누르면 *Global Coordinate*
    - 두 번 입력하거나 누르면 가운데 박스가 주황색으로 활성화되는데, 이때가 *Local Coordinate*

- *F* 누르면 전체 Full Screen, 객체 누르고 *F* 누르면 객체 기준 Full Screen

- *우클릭 + 마우스 회전*
    - 관찰자 기준으로 시점 변환(회전)

- *Alt + 좌클릭 + 마우스 회전*
    - 물체 기준으로 시점 변환(회전)

- *Create -> Physics -> PhysicsScene*
    - Gravity Acceleration, Solver Type 설정 가능

- Body Property에서 *Add -> Physics -> Rigid Body with Colliders Preset*
    - Physics라는 칸이 하나 생성되고 여기서 Mass, Damping 등 추가 가능

- *Create -> Physics -> Ground Plane*
    - 지면 생성

- * 눈 모양 -> Draw Overlay -> Physics -> Colliders -> Selected/All*
    - 충돌 감지 Outlier 표시

- *Create -> Physics -> Physics Material*
    -ㅁㄴㅇ

- *Base Body 선택 -> Ctrl + target body 선택 -> 우클릭 -> create -> physics -> joint*
    - 원하는 joint 선택 가능

- 설정된 joint 선택 -> 하단의 property에서 *Add -> physics -> angular drive*
    - target velocity 설정으로 주행 가능 설정

- 우측의 stage에서 *빈 공간 우클릭 -> create -> camera*
    - 카메라 센서 생성

- *Window -> Viewport -> viewport2 -> viewport2 창에서 perspective 아이콘 -> cameras -> 설정한 카메라 이름*
    - 카메라 시점 영상 확인
    - 카메라를 원하는 위치에 종속시키고, parent 좌표계를 기준으로 좌표계 설정

- *play 누르고 Shift 누르면 드래그하면 force따라서 움직여짐*

- **ROS2 연동을 위해서는 Action Graph 설정 진행해야함!**