# UR3e Workcell — Hướng dẫn cài đặt và chạy

Package `ur3e_llm_control` mô phỏng robot UR3e với gripper hai ngón,
bàn làm việc, năm hộp màu, ba zone chính, một zone xám trung gian và camera RGB.
Có thể điều khiển bằng skill trực tiếp, yêu cầu gửi cho LLM hoặc nhiệm vụ MSSV.

## 1. Các công cụ cần cài

Môi trường của workspace hiện tại: **Ubuntu 22.04, ROS 2 Humble và
Gazebo Sim 6 (Fortress)**. Cấu hình plugin trong package sử dụng Fortress.

| Công cụ | Công dụng |
|---|---|
| ROS 2 Humble Desktop | Chạy node, topic, service, action và RViz |
| MoveIt 2 | Lập quỹ đạo, kiểm tra va chạm và thực thi chuyển động |
| Gazebo Sim Fortress | Mô phỏng robot, tiếp xúc vật lý và camera |
| `ros_gz` | Bridge dữ liệu giữa Gazebo và ROS 2 |
| `ros2_control`, `ros2_controllers`, `gz_ros2_control` | Điều khiển khớp và gripper trong Gazebo |
| `ur_description` | Mô hình UR3e của Universal Robots |
| `colcon`, `rosdep` | Build workspace và cài phụ thuộc |
| Node.js 20 và npm | Chạy/cài 9Router; chỉ cần cho chế độ LLM |
| 9Router | Cổng API kết nối model; chỉ cần cho chế độ LLM |

### ROS 2 Humble

Máy mới cần thiết lập locale và kho APT ROS theo
[hướng dẫn ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
Sau khi đã thêm kho APT:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop \
  python3-colcon-common-extensions python3-rosdep git curl wget
```

### Gazebo Sim Fortress

Nếu máy chưa có kho APT Gazebo, thêm kho theo
[hướng dẫn Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/):

```bash
sudo apt install -y lsb-release gnupg curl
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y ignition-fortress
```

### MoveIt và các thành phần ROS

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ros-gz \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gz-ros2-control \
  ros-humble-ign-ros2-control \
  ros-humble-ur-description \
  ros-humble-xacro
```

Package hiện tham chiếu plugin tương thích Ignition
`libign_ros2_control-system.so`; giữ các gói điều khiển tương ứng ở trên.
Tham khảo [gz_ros2_control cho Humble](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html).

### 9Router — chỉ cho chế độ LLM

Cài Node.js 20 và npm theo [hướng dẫn Node.js](https://nodejs.org/en/download),
sau đó kiểm tra:

```bash
node --version
npm --version
```

Workspace đã chạy với 9Router `0.5.91`. Để dùng cùng phiên bản:

```bash
npm install -g 9router@0.5.91
```

Thông tin cài đặt và cấu hình provider:
[9Router](https://github.com/decolua/9router).

## 2. Các thư viện cần cài thêm

| Thư viện/gói | Công dụng |
|---|---|
| `python3-numpy` | Tính tọa độ, ma trận và xử lý dữ liệu ảnh |
| `python3-opencv` | Lọc màu, nhận diện vật trong zone |
| `python3-yaml` | Đọc cấu hình YAML |
| `ros-humble-cv-bridge` | Chuyển đổi ảnh ROS và OpenCV |

```bash
sudo apt install -y python3-numpy python3-opencv python3-yaml ros-humble-cv-bridge
```

Các gói ROS như `rclpy`, `sensor_msgs`, `tf2_msgs` và `moveit_msgs` được cài
qua ROS/phụ thuộc package. Dùng `rosdep` ở phần tiếp theo để bổ sung đầy đủ.
Node LLM dùng `urllib` có sẵn trong Python, không cần cài SDK OpenAI.

## 3. Trước khi chạy

### Cài phụ thuộc và build workspace

Giả sử workspace ở `~/ur3`, package ở `~/ur3/src/ur3e_llm_control`.

Nếu lấy package từ GitHub trên máy mới:

```bash
mkdir -p ~/ur3/src
git clone https://github.com/quansystems/llm_camera_ur3e.git ~/ur3/src/ur3e_llm_control
```

Nếu package đã có sẵn ở vị trí trên thì bỏ qua bước clone.

Nếu máy chưa khởi tạo rosdep, chạy một lần:

```bash
sudo rosdep init
```

Nếu rosdep đã được khởi tạo thì bỏ qua lệnh trên. Tiếp tục:

```bash
cd ~/ur3
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
```

### Chuẩn bị mỗi terminal ROS

Chạy các lệnh sau trong **mỗi terminal** trước khi chạy node:

```bash
cd ~/ur3
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Các terminal phải cùng `ROS_DOMAIN_ID`. Nếu không sử dụng domain riêng,
giữ giá trị mặc định. Dừng các bản node cũ bằng `Ctrl+C` trước khi chạy lại.

### Thông tin cần nhớ

- `home`: về góc khớp khởi tạo; giữ nguyên vật đang cầm.
- `place OBJECT`: gắp, nhấc và **giữ** vật.
- `pick OBJECT ZONE`: gắp nếu cần, chuyển đến zone và **thả** vật.
- Vật hợp lệ: `red`, `yellow`, `green`, `blue`, `purple`.
- Zone hợp lệ: `zone_1_red`, `zone_2_green`, `zone_3_yellow`, `zone_Intermediate`.
- Zone 1/2/3 trong nhiệm vụ MSSV là ba zone chính theo thứ tự trên;
  màu được phân công là màu **vật**, không phải đổi màu vùng đánh dấu.
- Gazebo phải đang chạy, không pause. Chờ các controller được kích hoạt
  trước khi gửi tác vụ. Chỉ chạy một node `action` tại một thời điểm.

## 4. Câu lệnh chạy

### 4.1. Khởi động hệ thống chung

**Terminal 1 — Gazebo, robot, controller, MoveIt và RViz:**

```bash
ros2 launch ur3e_llm_control ur3e_workcell.launch.py
```

Nếu chạy không có giao diện:

```bash
ros2 launch ur3e_llm_control ur3e_workcell.launch.py \
  gazebo_gui:=false launch_rviz:=false
```

**Terminal 2 — camera:**

```bash
ros2 run ur3e_llm_control camera_check.py
```

**Terminal 3 — validator:**

```bash
ros2 run ur3e_llm_control task_validator.py
```

Camera cần cho chế độ LLM. Chế độ MSSV và skill trực tiếp dùng trạng thái
Gazebo/MoveIt nên không bắt buộc camera. Validator cần cho hàng đợi action.

### 4.2. Chạy bằng LLM

**Terminal riêng — khởi động 9Router nếu chưa chạy:**

```bash
9router --no-browser --skip-update --host 127.0.0.1
```

Mở dashboard tại `http://localhost:20128/dashboard`. Kết nối provider,
tạo/cấu hình model hoặc combo tên **`ur3e`**, rồi tạo API key 9Router.
Node mặc định sử dụng API `http://localhost:20128/v1` và model `ur3e`.

**Terminal 4 — node LLM:**

```bash
ros2 run ur3e_llm_control llm_control.py
```

Nhập API key ở lời nhắc ẩn, sau đó nhập yêu cầu, ví dụ:

```text
pick blue zone_3_yellow; home
put blue object in yellow zone
Đặt vật xanh dương vào vùng vàng
place purple
```

Node gửi yêu cầu, camera, trạng thái đầy đủ và mục tiêu cho LLM. Kế hoạch
được kiểm tra trước khi chạy; sai thì yêu cầu model sửa một lần. Nếu vẫn sai,
node báo lỗi và không gửi lệnh robot. Yêu cầu chưa rõ như `lại` bị từ chối;
hãy nhập mục tiêu đầy đủ hoặc dùng cú pháp skill.

Khi kế hoạch hợp lệ, node **tự khởi động action.py nếu cần và tự kích hoạt
thực thi**. Không cần chạy action hoặc gửi tín hiệu `action` thủ công.
Chờ tác vụ hoàn tất rồi nhập yêu cầu tiếp theo. Nhập `exit` để thoát.
API key chỉ dùng trong phiên làm việc, không được node lưu vào file.

### 4.3. Chạy bằng MSSV

Dùng hệ thống chung ở 4.1 và **không chạy tác vụ LLM đồng thời**.

**Terminal 4 — hàng đợi trống:**

```bash
ros2 run ur3e_llm_control action.py
```

**Terminal 5 — nhập MSSV:**

```bash
ros2 run ur3e_llm_control student.py
```

Nhập MSSV dạng chữ số, ít nhất hai chữ số. Node lấy hai chữ số cuối `XX`,
tính `P = XX mod 6`, in chuỗi skill và đưa kế hoạch vào action.

| P | Vật vào Zone 1 | Vật vào Zone 2 | Vật vào Zone 3 |
|---|---|---|---|
| 0 | Red | Yellow | Blue |
| 1 | Red | Blue | Yellow |
| 2 | Yellow | Red | Blue |
| 3 | Yellow | Blue | Red |
| 4 | Blue | Red | Yellow |
| 5 | Blue | Yellow | Red |

Node xét trạng thái hiện tại và dùng vùng xám để đổi chỗ khi cần. Nếu không
có chuỗi hợp lệ, nó báo lỗi và không gửi kế hoạch. Nếu vật đã đúng vị trí,
không cần di chuyển.

**Terminal 6 — sau thông báo đã đẩy chuỗi vào action:**

```bash
ros2 topic pub --once /action/command std_msgs/msg/String "data: 'action'"
```

Student **chỉ đưa kế hoạch vào hàng đợi**, không tự kích hoạt robot và không
cần 9Router/API key. Action thoát khi tác vụ kết thúc; chạy lại action cho
nhiệm vụ mới. Nhập `exit` để thoát student.

### 4.4. Chạy skill trực tiếp

Với workcell đang chạy, dùng một terminal đã source. Chạy từng lệnh sau khi
lệnh trước kết thúc; không chạy đồng thời với hàng đợi action:

```bash
ros2 run ur3e_llm_control robot_skill.py home
ros2 run ur3e_llm_control robot_skill.py place purple
ros2 run ur3e_llm_control robot_skill.py pick purple zone_Intermediate
ros2 run ur3e_llm_control robot_skill.py home
```

Zone đích phải trống; nếu đã có vật, chuyển vật đó sang zone trống trước.
Validator có thể chạy để hiển thị kết quả, nhưng skill trực tiếp không cần
validator để điều khiển robot.

### 4.5. Đưa chuỗi skill thủ công vào action

Khởi động action với hàng đợi trống, rồi gửi từ terminal khác:

```bash
ros2 run ur3e_llm_control action.py
```

```bash
ros2 topic pub --once /action/command std_msgs/msg/String \
  "data: 'place purple; pick purple zone_Intermediate; home'"
ros2 topic pub --once /action/command std_msgs/msg/String "data: 'action'"
```

Chuỗi thủ công phải có thứ tự hợp lệ và zone đích trống. Không ghép thêm
chuỗi thủ công vào kế hoạch student/LLM đã được kiểm tra.

### 4.6. Điều khiển riêng gripper và xem kết quả

Mở/đóng riêng gripper khi không có tác vụ khác đang điều khiển nó:

```bash
ros2 run ur3e_llm_control gripper_control.py open
ros2 run ur3e_llm_control gripper_control.py close --force 5.0 --wait 1.0
```

Xem kết quả từng skill hoặc toàn bộ tác vụ, mỗi lệnh ở một terminal riêng:

```bash
ros2 topic echo /task_validator/result
```

```bash
ros2 topic echo /task_validation
```

`task_validator/result` có dạng `pick blue zone_3_yellow success` hoặc
`pick blue zone_3_yellow failed PLANNING_FAILED`. `/task_validation` trả
`task success` hoặc `task failed`; mở subscriber trước khi tác vụ hoàn tất.

Nếu lệnh publish báo `Waiting for at least 1 matching subscription(s)...`,
hãy kiểm tra action.py đang chạy, các terminal đã source đúng workspace và
cùng ROS domain. Gửi `action` khi hàng đợi trống sẽ bị từ chối.
