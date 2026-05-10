# Phân Tích Các Thuật Toán Cốt Lõi

Dự án này vượt xa khỏi một luồng điều khiển cơ bản nhờ sự kết hợp của hai thuật toán: **Chống va chạm đa cấp dựa trên LiDAR** và **Tìm đường cục bộ bằng BFS**.

---

## 1. Hệ thống Chống Va Chạm Đa Cấp Chủ Động
Phương pháp truyền thống của robot hút bụi giá rẻ là đâm sầm vào vật cản (Bumper Trigger) -> Dừng lại -> Lùi -> Đổi hướng. Điều này gây hao mòn cơ học và làm rung lắc cảm biến, gây nhiễu bản đồ odometry.

Thuật toán hiện tại sử dụng **LiDAR LDS-01** để quét phía trước (Góc 0 độ) và chia không gian thành 3 ranh giới động học (Dynamic Zones):

1. **`VUNG_AN_TOAN` (> 0.6m):** 
   - Không có nguy hiểm. Robot chạy với `V_ZIGZAG = 0.4 m/s` (Tốc độ tối đa).
   
2. **`VUNG_CANH_BAO` (0.12m -> 0.6m):**
   - Robot phát hiện bề mặt vật cản ở đằng xa và bắt đầu **nội suy giảm tốc độ (Deceleration Interpolation)**.
   - Vận tốc `cv` không bị giật về 0 ngay, mà được tính toán tỷ lệ tuyến tính với khoảng cách: `cv = toc_do_min + (V_ZIGZAG - toc_do_min) * ((kc_truoc - 0.12) / (0.6 - 0.12))`.
   - Giúp bánh xe không bị trượt (drift).
   
3. **`VUNG_NGUY_HIEM` (<= 0.12m):**
   - Ranh giới giới hạn cực hạn. Tâm robot cách vật cản 0.12m đồng nghĩa với vỏ bọc vật lý của robot đang sượt qua vật. Bắt buộc phải bẻ lái để không đâm sầm vào.
   - Động cơ tịnh tiến khóa cứng (`cv = 0`).
   - Robot đánh dấu ô lưới ngay trước mặt là **Vật cản (Trạng thái 2)** lên bản đồ ngay lập tức để né nhánh.

Việc tách mốc bắt đầu rà phanh (`0.6m`) và mốc bẻ lái (`0.12m`) ra làm hai tham số riêng biệt giúp robot vừa hãm phanh êm ái, vừa có thể tiến sát sạt vào góc tủ để không bỏ sót một cm vuông nào ở mép.

---

## 2. Tìm Đường Phục Hồi BFS (Breadth-First Search)

Thuật toán quét phòng ZigZag (Boustrophedon) truyền thống sẽ bị tê liệt hoàn toàn nếu gặp phải một hòn đảo vật cản ở giữa phòng (tạo thành một hình chữ U nhốt robot lại). Để hóa giải, hệ thống sử dụng thuật toán lan truyền `BFS`.

### 2.1 Quá trình lan truyền (Flood Fill / BFS Queue)
Khi robot bị kẹt, hàm `tim_o_chua_tham()` kích hoạt:
- Khởi tạo một hàng đợi (Queue) chứa tọa độ hiện tại của robot.
- Khám phá 4 ô liền kề (Trên, Dưới, Trái, Phải).
  - Nếu là ô `Vật cản (2)` hoặc ngoài biên: Bỏ qua.
  - Nếu là ô `Đã thăm (1)`: Bỏ qua.
  - Nếu là ô `Chưa thăm (0)`: Thuật toán DỪNG LẠI. Do đặc tính quét lan tỏa theo vòng tròn đồng tâm của BFS, ô số `0` đầu tiên tìm thấy chắc chắn là ô gần robot nhất và không bị cản.
- Lưu dấu vết đường đi (Traceback) qua mảng `cha[]` (Parent Array).

### 2.2 Quá trình di chuyển (Navigation Waypoints)
- Các tọa độ truy vết ngược lại tạo thành một mảng `duong_di`.
- FSM kích hoạt 3 pha: `DH_QUAY` (Chỉa mũi về tọa độ kế), `DH_TIEN` (Lao tới), `DH_LUI` (Dự phòng).
- Robot đi lách qua các khe hở vật cản bằng đường đi ngắn nhất để đặt chân đến được vùng đất mới.

### 2.3 Quá trình tự định chuẩn hướng (Recalibration)
Khi đến được "vùng đất mới", làm thế nào robot biết nên quét ZigZag theo chiều dọc hay chiều ngang cho tối ưu?
Hàm `dem_o_trong_theo_huong()` sẽ phóng 4 tia ảo (Raycasting) ra 4 hướng: 0°, 90°, 180°, 270°. Hướng nào có nhiều ô `0` liên tiếp nhất sẽ được chọn làm trục quét (Axis) mới của chữ S, tối đa hóa thời gian quét đường thẳng và hạn chế việc phải quay đầu vụn vặt.

---

## 3. Quản lý Nhiễu Bản Đồ (Map Noise Filtering)

Trong robot thực tế, LiDAR có độ chệch góc khi rung lắc, tạo ra các "bóng ma" (Ghost Obstacles). Thuật toán giải quyết vấn đề này qua 2 cách:
1. **Chặn tầm nhìn xa**: Hàm cập nhật bản đồ chỉ lấy các điểm `kc < 0.5m`. Tức là robot phải lại gần vật cản mới vẽ lại nó trên mảng. Điều này triệt tiêu hoàn toàn độ nhiễu góc của Odometry từ xa, giúp vật thể 2x2 ô đúng chuẩn là 2x2 ô chứ không bị nhòe ra 3x3 ô.
2. **Luật Override Một Chiều**: Một ô đã là số `1` (robot từng đi qua thành công) thì không bao giờ bị LiDAR ghi đè thành số `2` (vật cản) nữa, dù tia laser có bị lóe do gương hay kính. Điều này ngăn robot tự phong tỏa đường về của chính mình.
