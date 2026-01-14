Dưới đây là nội dung file `README.md` đã được cập nhật thêm phần **Hướng dẫn đấu nối (Wiring)** chi tiết cho Arduino, L298N và Encoder. Nội dung này khớp hoàn toàn với đoạn code Arduino mà tôi đã cung cấp cho bạn trước đó.

Bạn hãy copy toàn bộ nội dung dưới đây đè vào file `README.md` cũ:

---

# AMR Robot Project - ROS 2 on Raspberry Pi 5

Dự án robot tự hành sử dụng **Raspberry Pi 5** chạy **ROS 2** (Humble/Jazzy), Arduino để điều khiển động cơ, và Lidar để định vị/tạo bản đồ (SLAM). Dự án đã được tích hợp các script tự động hóa để vận hành dễ dàng.

## 1. Cấu trúc phần cứng
*   **Máy tính nhúng:** Raspberry Pi 5.
*   **Vi điều khiển:** Arduino (Uno/Nano/Mega) - Giao tiếp qua Serial (USB).
*   **Cảm biến:** Lidar (RPLidar A1/A2/A3...) - Giao tiếp qua USB.
*   **Driver động cơ:** L298N (Hoặc TB6612).
*   **Động cơ:** 2 x DC Motor có giảm tốc và Encoder (loại 6 dây: M+, M-, 5V, GND, A, B).

### 1.1. Hướng dẫn đấu nối (Wiring Diagram)
Để code Arduino hoạt động chính xác, bạn cần đấu dây theo sơ đồ sau (Khớp với `arduino_firmware.ino`):

#### A. Kết nối Driver L298N với Arduino & Động cơ

| L298N Pin | Arduino Pin | Động cơ (Motor) | Chức năng |
| :--- | :--- | :--- | :--- |
| **12V** | - | Nguồn Dương (+) | Nguồn Pin (11.1V - 24V) |
| **GND** | **GND** | Nguồn Âm (-) | **Bắt buộc nối chung mass** |
| **5V** | 5V / Vin | - | Cấp nguồn cho Arduino (nếu cần) |
| | | | |
| **ENA** | **Pin 9** | - | PWM Động cơ Trái (Left) |
| **IN1** | **Pin 8** | - | Chiều quay 1 (Left) |
| **IN2** | **Pin 7** | - | Chiều quay 2 (Left) |
| **OUT1/2** | - | Motor Trái | Dây Động lực |
| | | | |
| **ENB** | **Pin 10** | - | PWM Động cơ Phải (Right) |
| **IN3** | **Pin 12** | - | Chiều quay 1 (Right) |
| **IN4** | **Pin 13** | - | Chiều quay 2 (Right) |
| **OUT3/4** | - | Motor Phải | Dây Động lực |

#### B. Kết nối Encoder với Arduino (Quan trọng)
*Lưu ý: Arduino Uno/Nano chỉ có 2 chân ngắt (Pin 2, Pin 3), nên phải nối đúng chân Phase A vào đây.*

| Encoder Dây | Arduino Pin | Ghi chú |
| :--- | :--- | :--- |
| **VCC (Đỏ)** | 5V | Nguồn Encoder |
| **GND (Đen)** | GND | Mass Encoder |
| | | |
| **Left Phase A** | **Pin 2** | Dây vàng/trắng (Ngắt) |
| **Left Phase B** | **Pin 4** | Dây xanh (Đếm chiều) |
| | | |
| **Right Phase A** | **Pin 3** | Dây vàng/trắng (Ngắt) |
| **Right Phase B** | **Pin 5** | Dây xanh (Đếm chiều) |

> **Chú ý:** Nếu robot chạy ngược chiều hoặc tính Odom bị trừ lùi khi tiến, hãy đảo 2 dây động lực (M+, M-) hoặc đảo vị trí dây A và B của Encoder.

---

## 2. Cài đặt & Chuẩn bị

### 2.1. Cài đặt các gói phụ thuộc
Chạy các lệnh sau trên Raspberry Pi để cài đặt các thư viện ROS 2 cần thiết (chỉ cần chạy 1 lần đầu):

```bash
sudo apt update
# Đối với Ubuntu 24.04 (Jazzy)
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-xacro ros-jazzy-robot-state-publisher libserial-dev python3-serial

# Đối với Ubuntu 22.04 (Humble)
# sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-xacro ros-humble-robot-state-publisher libserial-dev python3-serial
```

### 2.2. Cấp quyền cho các Script tự động
Đảm bảo bạn đã tạo thư mục `scripts` (hoặc để ở root) chứa các file `.sh`. Ví dụ nếu để ở root workspace:

```bash
cd ~/phong_le_10_1/ros2_ws
chmod +x scripts/*.sh 2>/dev/null || chmod +x *.sh
```

### 2.3. Build Workspace
Mỗi khi bạn sửa code hoặc thay đổi cấu hình, hãy chạy script này để xóa build cũ và biên dịch lại sạch sẽ:

```bash
./scripts/build.sh
# Hoặc nếu bạn để file ở ngoài: ./build.sh
```

---

## 3. Quy trình Vận hành (Sử dụng Script)

### Bước 1: Khởi động hệ thống (Bringup)
Script này sẽ tự động **cấp quyền USB** (bạn có thể cần nhập mật khẩu sudo) và khởi động toàn bộ hệ thống (Arduino, Lidar, Odom, URDF).

**Terminal 1:**
```bash
./scripts/start_robot.sh
```
> **Kiểm tra:** Nếu không thấy dòng chữ đỏ `ERROR` nào, Lidar quay và Arduino kết nối thành công, hệ thống đã sẵn sàng.

---

### Bước 2: Điều khiển Robot (Teleop)
Mở một terminal mới và chạy script để bật bàn phím điều khiển:

**Terminal 2:**
```bash
./scripts/run_teleop.sh
```
*   **Điều khiển:** Sử dụng các phím `i`, `j`, `k`, `l` để di chuyển.
*   **Tốc độ:** Nhấn `q` hoặc `z` để tăng/giảm tốc độ.

---

### Bước 3: Kiểm tra Lidar (Tùy chọn)
Nếu muốn chắc chắn Lidar đang gửi dữ liệu, mở terminal mới:

```bash
ros2 topic hz /scan
```
Nếu thấy hiển thị `average rate: 10.xxx` nghĩa là ổn.

---

### Bước 4: Chạy SLAM & Lưu Bản Đồ

**1. Bật thuật toán SLAM:**
Mở terminal mới (giữ nguyên Terminal 1 và 2):

**Terminal 3:**
```bash
./scripts/run_slam.sh
```

**2. Theo dõi trên Laptop:**
*   Mở **RViz2** trên Laptop (kết nối cùng Wifi).
*   Add Topic `/map` và `/scan`.
*   Dùng **Terminal 2 (Teleop)** lái robot đi khắp phòng để vẽ kín bản đồ.

**3. Lưu bản đồ:**
Khi bản đồ đã đẹp, mở terminal mới để lưu lại.

**Terminal 4:**
```bash
# Cách 1: Tự động đặt tên theo ngày giờ
./scripts/save_map.sh

# Cách 2: Tự đặt tên (ví dụ: phong_khach)
./scripts/save_map.sh phong_khach
```
> Bản đồ sẽ được lưu vào thư mục `maps/` trong workspace.

---

### Bước 5: Chạy Navigation (Dẫn đường tự động)

Sau khi đã có bản đồ, tắt hết các terminal cũ (Ctrl+C).

**1. Khởi động lại Robot:**
**Terminal 1:**
```bash
./scripts/start_robot.sh
```

**2. Bật Navigation:**
Chạy script nav và trỏ đến tên bản đồ bạn muốn dùng (không cần đuôi .yaml).

**Terminal 2:**
```bash
# Nếu không nhập tên, mặc định sẽ tìm file 'my_map.yaml'
./scripts/run_nav.sh phong_khach
```

**3. Thao tác trên RViz (Laptop):**
*   **2D Pose Estimate:** Chấm vị trí robot hiện tại trên bản đồ cho khớp thực tế.
*   **Nav2 Goal:** Chọn điểm đích, robot sẽ tự tìm đường đi tới đó.

---

## 4. Xử lý lỗi thường gặp (Troubleshooting)

1.  **Lỗi `Failed to open serial port` khi chạy `./start_robot.sh`:**
    *   Script đã tự động `chmod 777`, nhưng hãy kiểm tra xem dây cáp có bị lỏng không.
    *   Kiểm tra xem tên cổng trong `bringup.launch.py` (ví dụ `/dev/ttyUSB0`) có khớp với thực tế không (kiểm tra bằng `ls /dev/tty*`).

2.  **Robot đi không thẳng hoặc bản đồ bị méo:**
    *   Kiểm tra PID trong code Arduino.
    *   Kiểm tra thông số `WHEEL_RADIUS` và `WHEEL_BASE` trong code Arduino xem đã đo đúng chưa.
    *   Đảm bảo dây Encoder không bị lỏng (nếu mất xung encoder, robot sẽ quay tròn).

3.  **Lỗi `command not found`:**
    *   Đảm bảo bạn đang đứng đúng thư mục workspace (`~/phong_le_10_1/ros2_ws`).
    *   Đảm bảo đã cấp quyền thực thi: `chmod +x scripts/*.sh`.
