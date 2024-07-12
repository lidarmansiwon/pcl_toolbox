# pcl_toolbox

#### Code using PCL Library
## <Environment>
### Ubuntu 20.04 ROS Foxy

## Tools currently available

1. Voxelization(복셀화) --> 계산량 감소 
2. PassThrough         --> 관심 영역 설정
3. Transform           --> 쿼터니언 좌표계를 이용한 회전 변환
4. Statistics-based noise remover --> 통계 기반 잡음 제거

3.1 Parameter
```
            parameters=[
                {"crop_box_x_min": -0.0},
                {"crop_box_x_max": 20.0},
                {"crop_box_y_min": -20.0},
                {"crop_box_y_max": 20.0},
                {"crop_box_z_min": -2.0},
                {"crop_box_z_max": 2.0},
                {"voxel_resolution": 0.1},
                {"setMean": 10.0},
                {"setStddevMulThresh": 0.5},
                {"rotation_quaternion_x": 1.0},
                {"rotation_quaternion_y": 0.0},
                {"rotation_quaternion_z": 0.0},
                {"rotation_quaternion_w": 0.0}, 
            ]
```
crop_box                 --> **관심 영역 설정** 
voxel_resolution         --> **복셀 해상도 크기**
setMean                  --> **분석시 고려할 이웃 점의 수**
setStddevMulThresh       --> **Outlier로 처리할 거리 정보, 표준편차에 기반한 임계치 값으로 값이 작을수록 aggressive 하게 적용**
rotation_quaternion      --> **쿼터니언 좌표계를 통한 회전변환**

## Launch 
``` ros2 launch pcl_toolbox tool_box_launch.py ```
