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