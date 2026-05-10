# Kiến Trúc Hệ Thống (System Architecture)

Dự án điều khiển robot hút bụi trong môi trường Webots sử dụng hai thành phần kiến trúc cốt lõi: **Máy trạng thái hữu hạn (Finite State Machine - FSM)** và **Hệ thống bản đồ lưới (Grid Map)**.

---

## 1. Hệ thống FSM (22 Trạng thái)

Hệ thống điều khiển được chia làm 4 Pha (Phases) với tổng cộng 22 trạng thái (States) riêng biệt. Việc chia nhỏ các trạng thái giúp hệ thống linh hoạt và có thể phục hồi khi gặp lỗi hoặc va chạm.

### Pha 1: Khởi tạo và Bám tường (Corner Snap)
Giai đoạn này giúp robot xác định vị trí tuyệt đối của bản thân trong phòng và triệt tiêu sai số của cảm biến IMU ban đầu.
1. `QUET_360`: Xoay tại chỗ 360 độ để quét LiDAR, tìm góc vuông gần nhất giữa 2 bức tường.
2. `QUAY_VE_TUONG_1`: Quay mặt về bức tường thứ nhất.
3. `TIEN_DEN_TUONG_1`: Tiến lại gần tường 1 cho đến khi cách một khoảng `DUNG_TUONG_1`.
4. `LUI_1`: Lùi lại nhẹ nếu bị quá đà.
5. `TIM_TUONG_2`: Xoay vuông góc để hướng về bức tường thứ 2 (tạo thành góc phòng).
6. `QUAY_VE_TUONG_2`: Quay mặt về bức tường thứ 2.
7. `TIEN_DEN_TUONG_2`: Tiến lại gần tường 2.
8. `LUI_2`: Lùi lại dự phòng.
9. `CAN_CHINH_GOC`: Khớp góc chuẩn với trục tọa độ của thế giới mô phỏng.

### Pha 2: Chuẩn bị quét
10. `QUAY_180`: Quay ngoắt 180 độ, đưa lưng về phía góc phòng để sẵn sàng chạy đường thẳng đầu tiên.

### Pha 3: Quét ZigZag (Boustrophedon)
Đây là pha làm sạch chính, chiếm phần lớn thời gian hoạt động.
11. `TIEN (ZigZag)`: Chạy thẳng theo hướng hàng hiện tại. Đây là nơi tích hợp hệ thống *Chống va chạm đa cấp*.
12. `LUI (ZigZag)`: Phản xạ lùi lại nếu đâm trúng vật cản bằng cảm biến Bumper.
13. `QUAY_1`: Quay 90 độ khi gặp vật cản hoặc mép tường.
14. `DICH_HANG`: Đi ngang một khoảng bằng độ rộng chổi quét (`KHOANG_DICH_HANG`).
15. `QUAY_2`: Quay 90 độ tiếp tục để trở lại hướng quét ngược lại (Hoàn thành chữ U).

### Pha 4: Điều Hướng BFS & Phục Hồi
Khi thuật toán ZigZag bị kẹt bởi một khu vực khép kín, hệ thống kích hoạt Pha 4.
16. `DUNG_CHO_BFS`: Dừng 2 giây để LiDAR làm nét bản đồ trước khi thuật toán tìm đường chạy.
17. `KIEM_TRA_BAN_DO`: Kích hoạt BFS tìm kiếm ô lưới *chưa thăm (0)* gần nhất và xuất ra đường đi (Path).
18. `DH_QUAY`: Quay mặt về phía tọa độ (Waypoint) tiếp theo trên đường đi.
19. `DH_TIEN`: Đi thẳng tới Waypoint.
20. `DH_LUI`: Xử lý ngoại lệ nếu đụng chướng ngại vật ngầm trong quá trình đi BFS.
21. `CAN_LAI_HUONG`: Sau khi đến được vùng mới, phân tích không gian xung quanh để tính toán góc bắt đầu quét ZigZag lại từ đầu.

### Pha Cuối
22. `HOAN_THANH`: Dừng động cơ, hiển thị thống kê độ phủ.

---

## 2. Bản đồ lưới (Occupancy Grid Map)

Môi trường được ánh xạ vào một ma trận số 2 chiều (Grid Map) có kích thước **15 x 15** ô. Mỗi ô vuông (Cell) có kích thước vật lý là `0.33 x 0.33 mét`.

### Các Trạng Thái Của Ô Lưới
Mỗi ô lưu trữ một `struct OLuoi`:
- `trang_thai`: 
  - `0`: Chưa thăm (Màu trắng/vàng trên màn hình)
  - `1`: Đã thăm / Đi qua (Màu xanh)
  - `2`: Vật cản (Màu đỏ)
- `diem_tin` (Confidence Score): Điểm từ 0 -> 100. Khi LiDAR quét thấy vật cản ở tọa độ này, điểm tin tăng dần. Phải vượt qua một ngưỡng (Threshold) thì ô mới chính thức biến thành trạng thái `2`.

### Hệ Tọa Độ Ánh Xạ
- Hệ tọa độ Oxyz của Webots lấy tâm (0,0) làm gốc.
- Ma trận `ban_do` của C++ lấy `(0,0)` ở góc trên bên trái.
- Hàm `the_gioi_ra_luoi(x, y, &cot, &hang)`: Biến đổi tọa độ thực `(x,y)` của robot thành chỉ số hàng/cột của ma trận bằng cách tịnh tiến một `Offset = 7` (Đưa tâm (0,0) của thế giới vào ô `(7,7)` của ma trận).
- Hàm `luoi_ra_the_gioi(cot, hang, &x, &y)`: Đảo ngược tọa độ để robot biết phải đi đến vị trí nào trong không gian vật lý.

### Tính Năng An Toàn Dữ Liệu
Mọi điểm ảnh của LiDAR hoặc Odometry đều phải được truyền qua hàm kiểm tra `Clamp` (`min`, `max`) để đảm bảo không một chỉ số mảng nào bị vượt rào (`Out of Bounds`), gây lỗi văng chương trình (Crash). Khái niệm ô `Đã thăm (1)` được bảo vệ nghiêm ngặt: Một khi robot đã đi vào ô đó, nó không thể bị "nhiễu" LiDAR đánh giá lùi thành `Vật cản (2)`.
