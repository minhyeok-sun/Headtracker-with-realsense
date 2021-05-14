# Headtracker-with-realsense

intel의 realsense 3D depth 카메라 기반 head tracking 평면 출력 코드
2D RGB image data 에서 안경에 장착된 3개의 포인트를 opencv 를 통해 좌표를 확인한 후 그 위치에 대응하는 depth image 정보를 활용하여 inverse projection을 통해 3차원 XYZ 좌표를 구한다.
3개의 point에 대해 3차원 XYZ좌표를 한후 다음 세점을 지나는 평면 방정식을 계산한다. 평면의 법선의 움직임을 통해 기준점에서 머리가 어느 각도 회전했는지 계산할 수 있다. 해당 코드는 3차원 xyz 좌표를 구하는 과정까지 작성되었으며 yaw, pitch, roll 계산하는 코드는 작성중에 있다.

https://www.notion.so/fusion-e26a1b5026d647fa8e8cebf8fe4e80eb
