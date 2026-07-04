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
│   │   ├── go1_controllers.yaml   Cấu hình đi thật (leg_pd + unitree_guide, 1000Hz)
│   │   └── gz_bridge.yaml         Danh sách topic bridge gz <-> ROS2
│   ├── launch/
│   │   ├── sim.launch.py      Spawn robot + bộ điều khiển đi thật + RViz (mặc định luôn)
│   │   └── rz.launch.py       Xem riêng trong RViz, không vật lý (kiểm mesh/khớp)
│   └── worlds/dog_test_world.sdf   Địa hình test (đá/bậc thang/hố/dốc/sóng/cầu thang)
│
├── gui/                        Bảng điều khiển Tkinter (`ros2 run gui gui`)
│                               Start/Stop Sim (+RViz kèm theo), Start/Stop RViz riêng,
│                               nút Đứng lên/Đi/Dừng đi/Nằm + 2 cần joystick ảo (Di
│                               chuyển 360 độ + Xoay), biểu đồ cân bằng roll/pitch
│                               thời gian thực, nút Tắt hết & Thoát.
│
├── unitree_guide_controller/  Bộ điều khiển đi thật (vendor từ legubiao/quadruped_ros2_control)
│                               FSM + gait + Kalman filter + QP cân bằng + message Inputs + keyboard_input
│
└── leg_pd_controller/          Bộ điều khiển PD cấp thấp (vendor, tách riêng vì không phụ thuộc 2 package kia)
```

`sim.launch.py` từng tách riêng thành `sim.launch.py` (chỉ đứng, demo) và `walk.launch.py` (đi thật) - đã gộp làm một vì đi thật bao gồm cả đứng, không cần giữ 2 file gần giống nhau. Mặc định mở kèm RViz (xem cùng lúc dữ liệu thật của mô phỏng); dùng `rviz:=false` nếu không muốn mở.

## Lưu đồ khối - đường đi của lệnh điều khiển (khi đi bộ)

```
┌──────────────────┐      ┌───────────────────────┐
│  keyboard_input   │      │   GUI (Tkinter)        │
│  (phím WASD/1-6)  │      │  Đứng lên/Đi + joystick│
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

FixedStand (đứng yên) trong sơ đồ trên cũng dùng chung đường `Estimator -> BalanceCtrl` này để chủ động cân bằng, không chỉ giữ góc khớp cố định như trước - xem mục "Cân bằng chủ động khi đứng yên" bên dưới.

## Cách chạy

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run gui gui                        # cách dễ nhất - có nút bấm cho mọi thứ
```

Hoặc chạy tay từng launch file:

```bash
ros2 launch main_bot sim.launch.py                # Gazebo (đi được thật) + RViz cùng lúc
ros2 launch main_bot sim.launch.py rviz:=false     # chỉ Gazebo, không mở RViz
ros2 launch main_bot sim.launch.py world:=<path>   # đổi world, vd. dog_test_world.sdf
ros2 launch main_bot rz.launch.py                  # xem riêng trong RViz, không vật lý

ros2 run unitree_guide_controller keyboard_input   # điều khiển bằng bàn phím (cần sim.launch.py đang chạy)
```

Điều khiển bàn phím: bấm `2` hai lần để đứng lên (Passive → FixedDown → FixedStand), `4` để chuyển sang đi (Trotting), W/S/A/D di chuyển, J/L xoay. GUI có sẵn nút cho bước đứng lên/đi/dừng/nằm, cộng 2 cần joystick ảo (kéo-thả, tự bật lại về giữa khi buông) thay cho D-pad cũ: cần "Di chuyển" kéo theo hướng bất kỳ trong vòng tròn 360 độ (đi chéo được, không chỉ 4 hướng rời rạc), cần "Xoay" kéo trái/phải để xoay - càng kéo xa tâm càng nhanh, giống joystick tay cầm game thật. Cộng biểu đồ roll/pitch thời gian thực để theo dõi khi nào robot sắp mất thăng bằng.

## Giới hạn đã biết

- **Đã sửa: gait bị ngã sau khi đi thẳng một lúc.** Nguyên nhân là `WaveGenerator` (bộ đếm nhịp gait, quyết định chân nào đang swing/stance) dùng đồng hồ **wall-clock thật** (`std::chrono::system_clock`) thay vì dùng thời gian mô phỏng mà `update(time, period)` đã truyền vào - trong khi toàn bộ phần còn lại (Estimator/Kalman filter, StateTrotting) giả định `dt` cố định = 1/tần_số của thời gian **mô phỏng**. `real_time_factor: 1.0` trong world file chỉ là mục tiêu, không phải cam kết - hễ Gazebo chạy chậm hơn thời gian thực một chút (do tải CPU) là nhịp gait bị lệch dần khỏi các phép tính dựa trên `dt` mô phỏng, lệch tích lũy theo thời gian - khớp chính xác với triệu chứng "đi ổn lúc đầu, ngã sau vài giây". Đã sửa bằng cách cho `WaveGenerator::update()` nhận `dt` (giây mô phỏng) thay vì tự đọc đồng hồ hệ thống (`src/unitree_guide_controller/src/gait/WaveGenerator.cpp`). Verify: đi thẳng liên tục 40s không ngã (trước đây ngã sau 10-13s).
- **Vẫn còn giới hạn**: chuyển hướng đột ngột khi đang đi (đi thẳng rồi xoay ngay, không dừng hẳn) đã cải thiện rõ (từ ngã gần như ngay lập tức lên ~9s xoay liên tục mới ngã, test bằng giá trị y hệt GUI dùng: ly/rx=0.3 có ramp) nhưng chưa hết hẳn - nên dừng hẳn (nút "Dừng đi") trước khi đổi hướng để an toàn nhất. Đi thẳng một mình hoặc xoay một mình (từ trạng thái đứng yên) đều đã ổn định.
- **Đã giảm mạnh: robot lắc lư (rung roll/pitch qua lại) và dạt ngang khi đi thẳng.** Đo bằng `gz model -p` khi đi thẳng liên tục phát hiện: dù roll/pitch không đủ lớn để ngã, chúng dao động thật (từng lên tới ±0.4-0.6 rad khi đi thẳng 40s) và thân robot dạt dần sang một bên dù lệnh chỉ đi thẳng (không lệnh rẽ). Nguyên nhân: vùng "tự do" cho vị trí mục tiêu thân robot trong `StateTrotting.cpp` quá rộng (±5cm quanh vị trí thực tế) và giới hạn lực điều chỉnh ngang/dọc quá thấp (3 m/s²) - khi robot lệch đi (do bất đối xứng nhỏ khi tiếp đất, nhiễu số...), bộ điều khiển "buông" theo độ lệch thay vì kéo lại, và khi có kéo lại thì lực bị giới hạn quá sớm. Đã kiểm tra CoM (trọng tâm) thật của robot qua TF - không lệch trục Y đáng kể, nên không phải do khối lượng phân bố lệch. Đã sửa: thu hẹp vùng tự do (0.05m → 0.02m), tăng giới hạn lực điều chỉnh ngang/dọc (3 → 5 m/s²), tăng gain vị trí trục Y riêng (70 → 100). Verify: đi thẳng 40s, roll/pitch/yaw giờ luôn dưới 0.04 rad (trước đó dao động ±0.4-0.6 rad) - hết lắc lư rõ rệt; dạt ngang giảm nhưng chưa hết hẳn (~0.35m sau 40s, trước là ~0.4-0.55m) - vẫn còn dư một phần chưa rõ nguyên nhân sâu (nghi ngờ do bản chất động lực học gait trot, chưa đào sâu thêm).
- Sensor `depth_camera_left` (trong `sensors/`) tham chiếu 1 link không tồn tại trong `go1.xacro` - kế thừa từ file gốc Unitree, chưa verify hoạt động.
- Chưa có API dạng `cmd_vel`/nav2 - điều khiển hiện tại qua `/control_input` trực tiếp.

## Cân bằng chủ động khi đứng yên

Trước đây, sau khi đứng lên xong (trạng thái FixedStand), robot chỉ giữ nguyên góc khớp cố định qua PD thường (`BaseFixedStand.cpp`) - hoàn toàn không tham chiếu tư thế thân/IMU, nên chỉ cần một lực đẩy ngang nhỏ là ngã (đúng như phản ánh). Trong khi đó, code đã có sẵn 1 trạng thái FSM khác (`StateBalanceTest`, command=6, trước đây không lộ ra GUI) dùng đúng cơ chế cân bằng chủ động thật: đọc tư thế/vận tốc thân thật (Estimator, fusion IMU+động học chân) rồi tính lực chân tối ưu qua QP (`BalanceCtrl`) - y hệt cơ chế đang dùng khi đi bộ.

Đã đưa cơ chế này vào thẳng `BaseFixedStand` (dùng bởi trạng thái FixedStand/"Đứng lên" - nút người dùng bấm thường xuyên nhất): sau khi hoàn tất chuyển động đứng lên (nội suy góc khớp mượt như cũ, không đổi), thay vì giữ góc khớp cố định mãi mãi, chuyển sang chế độ tính lực chân chủ động (dùng lại đúng gain của `StateBalanceTest`) để phản ứng thật với nhiễu bên ngoài.

Verify bằng cách tạo lực đẩy thật qua plugin `ApplyLinkWrench` của Gazebo (thêm vào `dog_test_world.sdf`, topic `/world/quadruped_test_lab/wrench`):
- Đẩy 2000N ngang hông (~15 lần trọng lượng robot): chỉ lệch 2.7mm rồi tự ổn định lại đúng vị trí cũ - gần như không cảm nhận được.
- Đẩy 8000N (~60 lần trọng lượng robot, rất mạnh): bị đẩy lệch hẳn (~1.5m, nghiêng ~68°) nhưng dừng lại và giữ ổn định ở tư thế mới, không đổ sập/lật hẳn.

Nghĩa là các cú đẩy thực tế (nhẹ hơn 2000N nhiều) giờ gần như không làm robot lung lay - đúng vấn đề "chỉ cần 1 lực đẩy nhẹ là ngã" đã báo cáo.

## Tốc độ di chuyển (cần joystick)

GUI trước đây giới hạn cần joystick ở |lx/ly/rx| ≤ 0.3 - trong khi bộ điều khiển thật ra cho phép tới 0.4 m/s tiến/lùi, 0.3 m/s sang ngang, 0.5 rad/s xoay (`v_x_limit_/v_y_limit_/w_yaw_limit_` trong `StateTrotting.cpp`), nghĩa là cần joystick trước đây chỉ dùng ~30% tốc độ tối đa mà bộ điều khiển hỗ trợ. Đã đo thực tế bằng `gz model -p` (không đoán) để tìm giới hạn an toàn thật của từng hướng:
- **Tiến/lùi**: rất khỏe, ổn định ngay cả ở mức tối đa 1.0 (đi được 36m liên tục không ngã).
- **Sang ngang (strafe)**: yếu hơn - mức tối đa 1.0 ngã sau ~3-4s, nhưng 0.7 đứng vững hoàn toàn suốt 25s.
- **Xoay**: gần như không còn dư địa tăng - cả 0.4 và 0.5 đều ngã sau 6-8s xoay liên tục.

Vì 1 cần joystick dùng chung cho cả 2 trục tiến/lùi và sang ngang, giới hạn phải chọn theo hướng yếu hơn (strafe). Đã tăng `MOVE_STICK_MAX` (trong `gui/gui/main_window.py`) từ 0.3 lên **0.7** - nhanh hơn rõ rệt (tốc độ tiến đo qua GUI thật: ~0.32 m/s, gấp đôi trước đây) mà vẫn an toàn ở mọi hướng. `ROTATE_STICK_MAX` giữ nguyên 0.3 vì xoay không còn dư địa an toàn để tăng.
