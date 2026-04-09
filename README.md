📖 Giới thiệu Tổng quan

Dự án này là mã nguồn mô phỏng thuật toán điều hướng thông minh cho robot hút bụi (dựa trên mô hình iRobot Create truyền động vi sai) trong môi trường Webots.

Thuật toán giải quyết bài toán mâu thuẫn giữa Khả năng bao phủ toàn diện (Coverage) và Tránh va chạm linh hoạt (Obstacle Avoidance) bằng cách kết hợp:

Lập kế hoạch quỹ đạo ZigZag (Boustrophedon Path Planning): Tối ưu hóa diện tích làm sạch.

Điều hướng phản xạ (Reactive Navigation): Tránh vật cản đa lớp theo thời gian thực.

Bản đồ lưới (Occupancy Grid Map) & BFS: Tìm đường phục hồi khi quỹ đạo ZigZag bị phá vỡ.

✨ Các tính năng chính

Kiến trúc FSM 14 Trạng thái: Hệ thống điều khiển dựa trên Máy trạng thái hữu hạn (Finite State Machine) giúp robot chuyển đổi mượt mà giữa các pha: Tìm góc, Quét dọn, Tránh vật cản, và Tìm đường (Navigation).

Phát hiện vật cản đa lớp (Multi-layer Detection): Sử dụng kết hợp LiDAR 2D (LDS-01) để phát hiện sớm (ngưỡng 32cm) và Cảm biến Bumper để làm chốt chặn an toàn vật lý chống kẹt bánh.

Chiến lược "Chuyển hàng sớm" (Early Row Transition): Khi gặp vật cản trên đường ZigZag, robot tự động chuyển sang làn mới thay vì rẽ ngẫu nhiên, giúp giữ nguyên cấu trúc vùng quét.

Tự động tìm đường thông minh: Liên tục cập nhật Bản đồ Lưới (Grid Map) kết hợp thuật toán Tìm kiếm theo chiều rộng (BFS) để định tuyến robot đi dọn dẹp các "vùng mù" phía sau vật cản.

📂 Cấu trúc thư mục

Dự án được cấu trúc chuẩn hóa cho môi trường Webots:

📦 Webots-Robot-Vacuum-Navigation
 ┣ 📂 controllers
 ┃ ┗ 📂 vacuum_controller
 ┃   ┗ 📜 vacuum_controller.cpp    # Mã nguồn chính điều khiển robot (C++)
 ┣ 📂 worlds
 ┃ ┗ 📜 room_5x5.wbt               # File môi trường 3D Webots (Phòng 5x5m + Vật cản)
 ┣ 📂 docs                         # Chứa báo cáo nghiên cứu và sơ đồ thuật toán
 ┣ 📜 README.md                    # Tài liệu hướng dẫn này
 ┗ 📜 .gitignore                   # Loại bỏ file rác biên dịch của Webots


🚀 Hướng dẫn Cài đặt & Chạy mô phỏng

1. Yêu cầu hệ thống

Phần mềm Cyberbotics Webots (Khuyên dùng bản R2023 trở lên, dự án được build trên bản R2025a).

Trình biên dịch C++ (Đã được Webots tích hợp sẵn MSYS2/MinGW trên Windows).

2. Chạy dự án

Clone dự án về máy:

git clone [https://github.com/TênCủaBạn/Webots-Robot-Vacuum-Navigation.git](https://github.com/TênCủaBạn/Webots-Robot-Vacuum-Navigation.git)


Mở phần mềm Webots.

Chọn File -> Open World... trên thanh menu.

Trỏ đến thư mục dự án, mở file worlds/room_5x5.wbt.

Bấm nút Play (▶️) trên giao diện Webots để robot bắt đầu chu trình tìm góc phòng và dọn dẹp.

🧠 Chi tiết Thuật toán (4 Giai đoạn FSM)

Giai đoạn 1: Thiết lập Hệ tọa độ chuẩn (Corner Snap). Sử dụng Lidar để tìm góc tường gần nhất, di chuyển bám mép tường và căn chỉnh lại hệ tọa độ tuyệt đối nhằm triệt tiêu sai số IMU ban đầu.

Giai đoạn 2: Quét ZigZag. Sử dụng bộ điều khiển PID góc (Heading PID) để giữ hướng đi thẳng tắp. Tự động quay đầu hình chữ U khi gặp ranh giới phòng hoặc vật cản lớn.

Giai đoạn 3: Phục hồi và Tìm đường. Khi bị bao vây hoặc hết không gian, dừng động cơ, quét ma trận Grid Map bằng BFS để tìm tập hợp Waypoints dẫn đến khu vực chưa dọn dẹp gần nhất.

Giai đoạn 4: Hoàn thành. Khi mảng Grid Map xác nhận độ phủ 100%, hệ thống dừng an toàn.

🔭 Hướng phát triển trong tương lai

Dự án là nền tảng tiền đề để phát triển mở rộng các tính năng robot dịch vụ nâng cao:

🗺️ Tích hợp SLAM (Gmapping/Cartographer) thay thế Odometry thuần túy.

🧠 Tích hợp AI (YOLO) thông qua Camera để nhận diện ngữ nghĩa vật cản (Phân biệt dây điện với thảm).

⚡ Thay thế BFS bằng A* hoặc D* Lite để tối ưu hóa năng lượng di chuyển.

📄 Bản quyền & Tác giả

Phát triển bởi: [Tên của bạn]

Dự án phục vụ cho mục đích Nghiên cứu Khoa học và học tập.
