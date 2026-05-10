# Webots Robot Vacuum Navigation

Dự án này là mã nguồn mô phỏng thuật toán điều hướng thông minh cho robot hút bụi (dựa trên mô hình iRobot Create truyền động vi sai) trong môi trường 3D Webots. Hệ thống điều khiển được viết hoàn toàn bằng C++.

Dự án giải quyết bài toán cốt lõi của robot tự hành trong nhà: Tối đa hóa khả năng bao phủ diện tích sàn (Coverage) và Tối ưu hóa việc di chuyển, tránh né vật cản khôn khéo (Obstacle Avoidance).

---

## ✨ Tính Năng Nổi Bật

1. **Hệ Thống FSM 22 Trạng Thái (Finite State Machine):**
   Kiến trúc điều khiển phức tạp chia thành 4 Pha chính (Bám tường, Chuẩn bị, Quét ZigZag, Phục hồi đường đi). Xử lý mượt mà mọi tình huống góc kẹt và các pha quay đầu chữ U.
   
2. **Hệ Thống Chống Va Chạm Đa Cấp Bằng LiDAR (Multi-Level Collision Avoidance):**
   Thay vì thuật toán "Xe đụng" (Bumper) ngớ ngẩn, robot có 3 vùng nhận thức không gian phía trước (An Toàn, Giảm Tốc, Bắt Buộc Rẽ). Robot sẽ lướt nhanh ở vùng thoáng và **rà phanh hãm tốc độ từ từ** khi lại gần vật cản, giúp tăng tuổi thọ động cơ và ngăn trượt bánh.
   
3. **Bản Đồ Lưới Chủ Động (Occupancy Grid Map):**
   Ghi nhớ 100% diện tích không gian xung quanh với lưới 15x15. Tích hợp bộ lọc nhiễu khoảng cách và khóa 1 chiều để các điểm mù của LiDAR không phá hỏng đường đi.

4. **Tìm Đường Phục Hồi BFS (Breadth-First Search):**
   Khắc phục điểm yếu chí tử của thuật toán ZigZag là bị "kẹt trong vòng vây chữ U". Robot có khả năng tự tính toán ma trận, tìm ra đường đi ngắn nhất lách qua vật cản để đến những "vùng đất" chưa dọn dẹp.

5. **Live Dashboard Console:**
   Hiển thị Real-time toàn bộ thông số: Tọa độ, Góc Yaw, Độ phủ (%), Bản đồ ASCII, và Cảnh báo vùng di chuyển trực tiếp trên terminal một cách khoa học.

---

## 📂 Cấu Trúc Dự Án

```text
Webots-Robot-Vacuum-Navigation/
├── controllers/
│   └── vacuum_controller/
│       ├── vacuum_controller.cpp   # File mã nguồn lõi C++ (1300+ lines)
│       └── Makefile                # Script biên dịch
├── DOC/
│   ├── Architecture.md             # Phân tích sơ đồ FSM & Grid Map
│   └── Algorithms.md               # Phân tích thuật toán BFS & Hệ thống đa cấp
├── worlds/
│   ├── room_5x5.wbt                # Phòng cơ bản 5x5m
│   └── room_5x5_vatcan.wbt         # Phòng nâng cao có nhiều hòm gỗ
└── README.md                       # Tài liệu hướng dẫn
```

*(Chi tiết thuật toán vui lòng đọc các file Markdown trong thư mục `DOC/`)*

---

## 🚀 Hướng Dẫn Cài Đặt & Chạy Mô Phỏng

### 1. Yêu cầu hệ thống
- Phần mềm **Cyberbotics Webots** (Khuyên dùng bản R2023 trở lên, dự án được build trên bản R2025a).
- Trình biên dịch C++ (Đã được Webots tích hợp sẵn MSYS2/MinGW trên Windows).

### 2. Chạy dự án
1. Mở phần mềm Webots.
2. Chọn `File` -> `Open World...` trên thanh menu.
3. Trỏ đến thư mục dự án đã tải về, mở file `worlds/room_5x5.wbt` hoặc `worlds/room_5x5_vatcan.wbt`.
4. Bấm nút Play (▶️) trên thanh công cụ Webots.
5. Xem Live Dashboard trực tiếp ở cửa sổ Webots Console phía dưới màn hình.

---

## 🔭 Hướng Phát Triển Tương Lai
Dự án được viết rõ ràng để tạo tiền đề cho các đồ án cao cấp hơn:
- 🗺️ Tích hợp SLAM thực thụ (Gmapping/Cartographer).
- 🧠 Áp dụng xử lý ảnh Camera để phân loại rác/dây điện.
- ⚡ Nâng cấp BFS thành A* hoặc D* Lite cho quãng đường dài hơn.

## 📄 Tác Giả
Được nghiên cứu và phát triển bởi: **Nguyễn Văn Hoàn**.
Dự án phục vụ cho mục đích học thuật và nghiên cứu thuật toán tự hành.
