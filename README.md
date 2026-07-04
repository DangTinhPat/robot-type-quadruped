# superDog

Mô phỏng robot chó 4 chân Unitree Go1 trên ROS 2 Jazzy + Gazebo Sim (Harmonic). Robot đứng được và đi được (trot gait thật, không phải demo).

## Cấu trúc package (`src/`)

```
src/
├── main_bot/                  Mô tả robot + world + launch file
│   ├── description/
│   │   ├── robot.urdf.xacro   <- điểm vào chính, launch file xử lý file này
│   │   ├── go1.xacro          Khung vật lý: link/joint/mesh/material + ros2_control
│   │   └── sensors/           1 file xacro / 1 nhóm cảm biến
│   │       ├── imu.xacro
│   │       ├── foot_contacts.xacro
│   │       ├── body_cameras.xacro
│   │       └── depth_camera_left.xacro
│   ├── config/
│   │   ├── go1_controllers.yaml       Cấu hình đứng (position_controller, 100Hz)
│   │   ├── go1_controllers_walk.yaml  Cấu hình đi (leg_pd + unitree_guide, 1000Hz)
│   │   └── gz_bridge.yaml             Danh sách topic bridge gz <-> ROS2
│   ├── launch/
│   │   ├── sim.launch.py      Spawn robot, đứng 1 tư thế cố định
│   │   ├── walk.launch.py     Spawn robot với bộ điều khiển đi thật
│   │   └── rz.launch.py       Xem robot trong RViz (không có vật lý)
│   └── worlds/dog_test_world.sdf   Địa hình test (đá/bậc thang/hố/dốc/sóng/cầu thang)
│
├── gui/                        Bảng điều khiển Tkinter (`ros2 run gui gui`)
│
├── unitree_guide_controller/  Bộ điều khiển đi thật (vendor từ legubiao/quadruped_ros2_control)
│                               FSM + gait + Kalman filter + QP cân bằng + message Inputs + keyboard_input
│
└── leg_pd_controller/          Bộ điều khiển PD cấp thấp (vendor, tách riêng vì không phụ thuộc 2 package kia)
```

## Lưu đồ khối - đường đi của lệnh điều khiển (khi đi bộ)

```
┌──────────────────┐      ┌───────────────────────┐
│  keyboard_input   │      │   GUI (Tkinter)        │
│  (phím WASD/1-6)  │      │  Đứng lên / Đi / D-pad │
└─────────┬──────────┘      └───────────┬────────────┘
          │                             │
          └──────────────┬──────────────┘
                          │  topic /control_input
                          │  (unitree_guide_controller/msg/Inputs:
                          │   command, lx, ly, rx, ry)
                          ▼
      ┌─────────────────────────────────────────────┐
      │            unitree_guide_controller           │
      │                                                │
      │   FSM: Passive → FixedDown → FixedStand        │
      │                     ↕                          │
      │                 Trotting                       │
      │                                                │
      │   Trotting state mỗi chu kỳ:                   │
      │   1. WaveGenerator   - chân nào swing/stance    │
      │   2. FeetEndCalc     - chân swing đặt xuống đâu │
      │                        (Raibert heuristic)      │
      │   3. BalanceCtrl     - lực chân stance cần bao   │
      │                        nhiêu (QP tối ưu, ràng    │
      │                        buộc nón ma sát)          │
      │   4. Estimator       - ước lượng vị trí/vận tốc  │
      │                        thân robot (Kalman filter,│
      │                        fusion IMU + động học chân)│
      └──────────────────────┬─────────────────────────┘
                              │ vị trí/vận tốc/kp/kd/effort
                              │ mong muốn mỗi khớp
                              │ (chained controller interface)
                              ▼
                  ┌───────────────────────┐
                  │   leg_pd_controller    │
                  │  tau = kp*Δq + kd*Δq̇   │
                  │       + tau_feedforward │
                  └───────────┬────────────┘
                              │ effort (mô-men lực)
                              ▼
                  ┌───────────────────────┐
                  │   gz_ros2_control      │
                  │   (GazeboSimSystem)    │
                  └───────────┬────────────┘
                              ▼
                  ┌───────────────────────┐
                  │   Gazebo - 12 khớp     │
                  │   go1 vật lý thật      │
                  └───────────────────────┘
```

## Đường đi của dữ liệu cảm biến (song song, ngược chiều)

```
Gazebo (IMU + 4 foot contact)
        │
        ├──────────────────────────────┐
        │ gz-transport                 │ ros2_control state interface
        ▼                              ▼
  ros_gz_bridge (gz_bridge.yaml)   gz_ros2_control
        │                              │
        ▼                              ▼
  /imu, /FR_foot_contact...       Estimator (Kalman filter)
  (xem bằng ros2 topic echo)      trong unitree_guide_controller
```

Hai đường độc lập nhau: nhánh trái để người dùng/RViz xem dữ liệu qua ROS2 topic thường, nhánh phải là đường bộ điều khiển thật sự dùng để tính toán (nhanh hơn, đồng bộ với chu kỳ điều khiển).

## Cách chạy

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run gui gui                        # cách dễ nhất - có nút bấm cho mọi thứ
```

Hoặc chạy tay từng launch file:

```bash
ros2 launch main_bot sim.launch.py      # robot đứng 1 tư thế cố định
ros2 launch main_bot walk.launch.py     # robot đi được thật
ros2 launch main_bot rz.launch.py       # xem trong RViz, không vật lý

ros2 run unitree_guide_controller keyboard_input   # điều khiển bằng bàn phím (cần walk.launch.py đang chạy)
```

Điều khiển bàn phím / GUI: bấm `2` hai lần để đứng lên (Passive → FixedDown → FixedStand), `4` để chuyển sang đi (Trotting), W/S/A/D di chuyển, J/L xoay.

## Giới hạn đã biết

- **Gait chưa ổn định lâu dài**: gain PD và trọng số QP trong `unitree_guide_controller` là giá trị mặc định từ upstream, chưa hiệu chỉnh cho robot này. Đi thẳng liên tục hiện tại sẽ ngã sau khoảng 10-13 giây (đã verify bằng cách đo tư thế thật qua `gz model -p` theo thời gian) - đây là giới hạn thật, chưa có cách khắc phục triệt để, không phải lỗi thao tác.
- Xoay ngay sau khi vừa đi thẳng dễ ngã hơn xoay từ trạng thái đứng yên - nên dừng hẳn (nút "Dừng đi") rồi đi lại trước khi xoay.
- Sensor `depth_camera_left` (trong `sensors/`) tham chiếu 1 link không tồn tại trong `go1.xacro` - kế thừa từ file gốc Unitree, chưa verify hoạt động.
- Chưa có API dạng `cmd_vel`/nav2 - điều khiển hiện tại qua `/control_input` trực tiếp.
