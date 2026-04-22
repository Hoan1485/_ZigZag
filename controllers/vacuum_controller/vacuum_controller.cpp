// ============================================================
// vacuum_controller.cpp
// FSM Tích Hợp: Lập Bản Đồ Lưới + BFS + ZigZag
// Phase 1: Tìm góc bằng thuật toán Quét Đa Hướng
// ============================================================

#include <webots/InertialUnit.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <queue>
#include <vector>

using namespace webots;

// ============================================================
// HẰNG SỐ VẬT LÝ
// ============================================================
static constexpr double WHEEL_R = 0.031;
static constexpr double WHEEL_L = 0.258;
static constexpr double MAX_WS = 6.28;

static constexpr double APPROACH_SPD = 0.20;
static constexpr double WALL2_SPD = 0.18;
static constexpr double BACKUP_SPD = -0.10;
static constexpr double FWD_SPD = 0.25;
static constexpr double FOLLOW_SPD = 0.18;
static constexpr double NAV_SPD = 0.30;
static constexpr double SHIFT_D = 0.30;
static constexpr double OBSTACLE = 0.20;

static constexpr double WALL1_STOP = 0.15;
static constexpr double WALL2_STOP = 0.15;
static constexpr double TURN_THR = 0.012;
static constexpr double NAV_DIST_THR = 0.08;

static constexpr double KP_H = 4.5;
static constexpr double KP_T = 5.5;

// [O10] Tính sẵn hằng số chuyển đổi
static constexpr double DEG2RAD = M_PI / 180.0;

// Bản đồ lưới (phòng 5x5m, ô 0.m)
static constexpr double CELL_SIZE = 0.2;
static constexpr int GRID_W = 40;
static constexpr int GRID_H = 40;
static constexpr int OFFSET_X = 20;
static constexpr int OFFSET_Y = 20;

// ============================================================
// [O2] Gộp gridMap + hitMap thành 1 struct GridCell
// ============================================================
struct GridCell {
  int8_t status = 0; // 0=chưa thăm, 1=đã dọn, 2=vật cản
  int8_t hits = 0;   // điểm tin cậy (cap 100)
};
static GridCell grid[GRID_H][GRID_W];

// ============================================================
// TRẠNG THÁI FSM
// ============================================================
enum State {
  // --- Phase 1: Khởi động và Tìm góc ban đầu ---
  // Mục tiêu: Tìm và bám theo góc của 2 bức tường vuông góc để thiết lập hệ tọa
  // độ và góc chuẩn ban đầu.
  SCAN_360, // Xoay/Quét Lidar xung quanh để tìm chướng ngại vật (tường) gần
            // nhất
  TURN_TO_WALL1,  // Xoay robot hướng thẳng về phía tường 1
  APPROACH_WALL1, // Tiến thẳng tới gần tường 1 cho đến khi đủ khoảng cách
  BACKUP_1,       // Lùi lại một chút nếu lỡ va chạm khi đang tiến tới tường 1
  FIND_WALL2, // Quét Lidar hai bên trái/phải để tìm tường thứ 2 vuông góc với
              // tường 1
  TURN_TO_WALL2,  // Xoay robot hướng thẳng về phía tường 2
  APPROACH_WALL2, // Tiến thẳng tới gần tường 2 (góc phòng)
  BACKUP_2,       // Lùi lại một chút nếu lỡ va chạm khi đang tiến tới tường 2
  CORNER_ALIGN,   // Căn chỉnh góc quay cho chẵn (bội số của 90 độ) để chuẩn bị
                  // lưới quét vuông vức
  TURN_180_RECOVERY, // Xoay 180 độ quay lưng lại với góc phòng để chuẩn bị xuất
                     // phát hàng đầu tiên

  // --- Phase 2: Dọn dẹp theo đường ZigZag ---
  // Mục tiêu: Di chuyển qua lại theo các đường thẳng song song để phủ kín diện
  // tích phòng.
  FORWARD, // Đi thẳng dọn dẹp theo dọc 1 hàng
  BACKUP,  // Lùi lại nếu va chạm phải vật cản khi đang đi thẳng
  TURN_1,  // Quay 90 độ lần 1 (bắt đầu chuyển sang hàng mới)
  SHIFT,   // Đi ngang một đoạn ngắn tương đương bề rộng 1 hàng dọn dẹp
  TURN_2,  // Quay 90 độ lần 2 (để robot song song với hàng cũ nhưng đi theo
           // hướng ngược lại)

  // --- Phase 3: Tìm đường (Navigation) bằng thuật toán BFS ---
  // Mục tiêu: Tự động tính toán và di chuyển đến các khu vực chưa được dọn dẹp
  // bị bỏ sót hoặc vướng vật cản.
  CHECK_MAP, // Chạy thuật toán BFS trên bản đồ lưới để tìm đường tới ô chưa
             // thăm (số 0) gần nhất
  NAV_TURN,  // Xoay robot hướng về điểm (waypoint) tiếp theo trên đường dẫn BFS
  NAV_FORWARD,  // Tiến thẳng tới điểm tiếp theo trên đường dẫn BFS
  NAV_BACKUP,   // Lùi lại nếu gặp vật cản bất ngờ cản đường di chuyển BFS
  RESUME_ALIGN, // Xoay căn chỉnh lại hướng đi cho đúng góc chuẩn ZigZag ban đầu
                // sau khi tới đích BFS

  // --- Phase 4: Hoàn thành ---
  DONE // Dừng toàn bộ hoạt động của robot khi bản đồ không còn ô trống
};

struct Point2D {
  int c, r;
};

// ============================================================
// [O7] Gom biến trạng thái vào struct để dễ dàng quản lý
// ============================================================
struct RobotState {
  // --- Thông tin Vị trí và Odometry (Đo lường quãng đường) ---
  double rx = 0, ry = 0; // Tọa độ thực của robot (world-frame tính bằng mét)
  double lastEncL = 0,
         lastEncR =
             0; // Giá trị encoder (góc quay) của 2 bánh xe ở bước trước đó

  // --- Thông tin Điều hướng chung ---
  double target_yaw = 0; // Góc quay mục tiêu mà robot đang muốn xoay tới
  double base_heading =
      0; // Góc hướng chuẩn ban đầu của hệ thống lưới (lấy sau Phase 1)
  double row_heading = 0; // Góc hướng di chuyển của hàng ZigZag hiện tại
  double resume_heading =
      0; // Hướng cần phục hồi lại sau khi robot hoàn thành chạy BFS
  double last_yaw =
      0; // Góc quay thực tế ở bước trước đó (dùng để tính vận tốc góc)
  bool first_step = true; // Cờ kiểm tra vòng lặp đầu tiên để thiết lập last_yaw

  // --- Biến lưu trữ riêng cho Phase 1 (Tìm góc) ---
  double wall1_yaw = 0; // Hướng của bức tường đầu tiên tìm được
  int wall2_dir = 1; // Hướng xoay để tìm tường 2 (+1: xoay trái, -1: xoay phải)

  // --- Biến lưu trữ riêng cho Phase 2 (ZigZag) và 3 (BFS) ---
  double shift_start_x = 0,
         shift_start_y =
             0;     // Tọa độ lưu lại tại thời điểm bắt đầu chuyển hàng (SHIFT)
  int turn_dir = 1; // Hướng bẻ lái chuyển hàng ZigZag (luân phiên 1 và -1)
  bool shift_blocked = false; // Cờ xác định xem quá trình chuyển hàng có bị kẹt
                              // bởi vật cản không
  int saved_gc = 0, saved_gr = 0; // Lưu lại chỉ số dòng/cột trên lưới (Grid)
                                  // hiện tại trước khi gọi BFS

  // --- Biến quản lý đường đi thuật toán BFS (Tìm đường) ---
  std::vector<Point2D> nav_path; // Danh sách tọa độ các điểm trên lưới robot
                                 // cần đi qua để đến đích
  size_t path_idx =
      0; // Chỉ số của điểm mục tiêu (waypoint) tiếp theo trong mảng nav_path
  bool is_recovery = false; // Cờ trạng thái: Robot đã tiến vào quá trình tự
                            // phục hồi quét các vùng sót
};

// ============================================================
// TIỆN ÍCH
// ============================================================

// [O1] Hàm chuẩn hóa góc xoay: Đảm bảo góc luôn nằm trong đoạn [-PI, PI].
// Tránh việc góc quay bị cộng dồn lên quá lớn sau nhiều vòng quay của robot.
static inline double normAng(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0)
    a += 2.0 * M_PI;
  return a - M_PI;
}

// [O8] Hàm điều khiển động cơ truyền động vi sai (Differential Drive).
// Nhận vào vận tốc tới (v) và vận tốc góc (omega) để quy đổi ra vận tốc cho
// từng bánh xe trái (vL), phải (vR).
static void diffDrive(double v, double omega, Motor *mL, Motor *mR) {
  double vL = (v - omega * WHEEL_L * 0.5) / WHEEL_R;
  double vR = (v + omega * WHEEL_L * 0.5) / WHEEL_R;
  double s = std::max(std::abs(vL), std::abs(vR));
  double scale = (s > MAX_WS) ? MAX_WS / s : 1.0;
  mL->setVelocity(vL * scale);
  mR->setVelocity(vR * scale);
}

// Hàm lấy khoảng cách đo được từ Lidar tại một góc độ cụ thể (deg).
// Tính toán chỉ số mảng (0-359) và lọc nhiễu: nếu giá trị quá nhỏ, vô hạn (inf)
// hoặc lỗi (NaN), trả về 9.9f (coi như không có vật cản).
static inline float lidarAt(const float *img, int deg) {
  int idx = ((deg % 360) + 360) % 360;
  float d = img[idx];
  return (d < 0.01f || std::isinf(d) || std::isnan(d)) ? 9.9f : d;
}

// [FIX] Hàm lấy khoảng cách vật cản nằm trực diện phía trước mũi robot.
// Lấy trung bình cộng các tia Lidar trong phạm vi góc ±5° (từ -5° đến 5°) để
// giảm tín hiệu nhiễu, giúp robot nhận biết chính xác khi chuẩn bị đâm vào
// tường.
static float lidarFront(const float *img) {
  float sum = 0;
  int cnt = 0;
  for (int offset = -5; offset <= 5; offset++) {
    int idx = ((offset) % 360 + 360) % 360;
    float v = img[idx];
    if (v > 0.01f && !std::isinf(v) && !std::isnan(v)) {
      sum += v;
      cnt++;
    }
  }
  return (cnt > 0) ? sum / cnt : 9.9f;
}

// Hàm tính giá trị khoảng cách trung bình từ Lidar trong một khoảng góc từ
// deg_start tới deg_end. Thường dùng trong việc quét tìm các bức tường bên hông
// trái/phải để so sánh khoảng cách.
static float lidarAvg(const float *img, int deg_start, int deg_end) {
  float sum = 0;
  int cnt = 0;
  for (int d = deg_start; d <= deg_end; d++) {
    float v = lidarAt(img, d);
    if (v < 9.0f) {
      sum += v;
      cnt++;
    }
  }
  return (cnt > 0) ? sum / cnt : 9.9f;
}

// Chuyển đổi từ Tọa độ thực (x, y tính bằng mét) sang Tọa độ ô lưới (c: cột, r:
// dòng) Dùng để xác định vị trí của robot hoặc điểm cần đánh dấu trên mảng 2
// chiều map.
static void worldToGrid(double x, double y, int &c, int &r) {
  c = (int)std::floor(x / CELL_SIZE) + OFFSET_X;
  r = (int)std::floor(y / CELL_SIZE) + OFFSET_Y;
}

// Chuyển đổi ngược lại từ Tọa độ ô lưới (c, r) ra Tọa độ thực tâm ô (x, y tính
// bằng mét) Dùng khi BFS tìm được đích (là 1 ô) và cần trả tọa độ thực về cho
// bộ điều khiển di chuyển tới.
static void gridToWorld(int c, int r, double &x, double &y) {
  x = (c - OFFSET_X) * CELL_SIZE + (CELL_SIZE * 0.5);
  y = (r - OFFSET_Y) * CELL_SIZE + (CELL_SIZE * 0.5);
}

// [O5-FIX] Hàm Cập nhật Bản đồ Lưới (Grid Map) dựa trên dữ liệu Lidar
// Mục tiêu: Ghi nhận vị trí các bức tường (vật cản) vào ma trận lưới.
// Để tối ưu bộ nhớ và thời gian, robot chỉ lấy mẫu cảm biến ở 3 hướng (Trước
// mặt, Bên trái, Bên phải) với khoảng cách cắt ngắn (0.5m). Tránh được tình
// trạng nhận nhầm khoảng cách xa hoặc nhiễu.
static void updateMap(const float *img, double rx, double ry, double ryaw,
                      double w_vel) {
  if (!img || std::abs(w_vel) > 0.5)
    return; // Bỏ qua cập nhật map khi Lidar chưa sẵn sàng hoặc robot đang xoay
            // quá nhanh (dễ gây méo bản đồ)

  // 3 hướng tương đối với mũi robot (chỉ số tia Lidar): Trước (0°), Trái (90°),
  // Phải (270°)
  static const int robot_dirs[3] = {0, 90, 270};

  for (int d = 0; d < 3; d++) {
    int lidar_idx = robot_dirs[d];

    // Lấy khoảng cách tia trung bình ±3° quanh hướng đó để giảm nhiễu
    float sum = 0;
    int cnt = 0;
    for (int offset = -3; offset <= 3; offset++) {
      int idx = ((lidar_idx + offset) % 360 + 360) % 360;
      float v = img[idx];
      // Chỉ phát hiện vật cản trong phạm vi 0.50m
      if (v > 0.05f && v <= 0.50f && !std::isinf(v) && !std::isnan(v)) {
        sum += v;
        cnt++;
      }
    }
    if (cnt == 0)
      continue;
    float r = sum / cnt;

    // Tính góc world-frame của tia này
    double world_ang = normAng(ryaw - lidar_idx * DEG2RAD);

    // Chiếu điểm obstacle
    int gc, gr;
    worldToGrid(rx + r * std::cos(world_ang), ry + r * std::sin(world_ang), gc,
                gr);
    if ((unsigned)gc >= (unsigned)GRID_W || (unsigned)gr >= (unsigned)GRID_H)
      continue;

    GridCell &cell = grid[gr][gc];

    // Tránh ghi đè lên ô đã đi qua hoặc đã đánh dấu
    if (cell.status == 1 || cell.status == 2)
      continue;

    // Tăng hit nhanh hơn vì chỉ có 3 tia
    int weight = 4;
    cell.hits = (int8_t)std::min((int)cell.hits + weight, 100);
    if (cell.hits >= 6)
      cell.status = 2;
  }
}

// ============================================================
// [O3] Thuật toán tìm đường BFS (Breadth-First Search)
// Chức năng: Tìm đường đi ngắn nhất từ ô hiện tại (sc, sr) đến ô gần nhất chưa
// được dọn dẹp (status = 0). Thuật toán quét lan ra xung quanh, tránh các ô vật
// cản (status = 2) và đánh dấu đường đi bằng mảng 'parent'. Kết quả trả về là
// một danh sách (vector) các điểm (Point2D) tạo thành đường dẫn để robot chạy
// theo.
// ============================================================
std::vector<Point2D> findUnvisited(int sc, int sr) {
  if ((unsigned)sc >= (unsigned)GRID_W || (unsigned)sr >= (unsigned)GRID_H)
    return {};

  // Mảng parent 1 chiều (làm phẳng 2D thành 1D), dùng để lưu dấu vết ô trước
  // đó. Giá trị -1 nghĩa là chưa thăm.
  static int parent[GRID_H * GRID_W];
  std::fill(parent, parent + GRID_H * GRID_W, -1);

  auto idx = [](int c, int r) { return r * GRID_W + c; };
  int start = idx(sc, sr);
  parent[start] = start; // self-parent đánh dấu đã thăm

  std::queue<int> q;
  q.push(start);

  const int dc[] = {0, 0, -1, 1};
  const int dr[] = {-1, 1, 0, 0};
  int goal = -1;

  while (!q.empty() && goal < 0) {
    int cur = q.front();
    q.pop();
    int cc = cur % GRID_W, cr = cur / GRID_W;
    if (grid[cr][cc].status == 0) {
      goal = cur;
      break;
    }
    for (int i = 0; i < 4; i++) {
      int nc = cc + dc[i], nr = cr + dr[i];
      if ((unsigned)nc >= (unsigned)GRID_W || (unsigned)nr >= (unsigned)GRID_H)
        continue;
      int nidx = idx(nc, nr);
      if (parent[nidx] >= 0 || grid[nr][nc].status == 2)
        continue;
      parent[nidx] = cur;
      q.push(nidx);
    }
  }

  if (goal < 0)
    return {};

  // Truy vết ngược
  std::vector<Point2D> path;
  for (int cur = goal; cur != start; cur = parent[cur])
    path.push_back({cur % GRID_W, cur / GRID_W});
  path.push_back({sc, sr});
  std::reverse(path.begin(), path.end());
  return path;
}

// Hàm đếm số lượng ô lưới chưa thăm (status = 0) dọc theo một hướng (heading)
// xuất phát từ (gc, gr). Hàm này phục vụ cơ chế kiểm tra nhanh xem hướng đang
// chạy (hàng ZigZag hiện tại) có còn khu vực nào trống cần dọn dẹp không. Nếu
// trả về 0, robot có thể bỏ qua và tìm đường khác ngay lập tức.
static int countZerosInDir(int gc, int gr, double heading) {
  int dx = (int)std::round(std::cos(heading));
  int dy = (int)std::round(std::sin(heading));
  int zeros = 0;
  int cc = gc, cr = gr;
  while (true) {
    cc += dx;
    cr += dy;
    if ((unsigned)cc >= (unsigned)GRID_W || (unsigned)cr >= (unsigned)GRID_H)
      break;
    if (grid[cr][cc].status == 2)
      break;
    if (grid[cr][cc].status == 0)
      zeros++;
  }
  return zeros;
}

// ============================================================
// MAIN
// ============================================================
int main() {
  Robot *robot = new Robot();
  int ts = (int)robot->getBasicTimeStep();

  Motor *mL = robot->getMotor("left wheel motor");
  Motor *mR = robot->getMotor("right wheel motor");
  mL->setPosition(INFINITY);
  mR->setPosition(INFINITY);

  PositionSensor *psL = robot->getPositionSensor("left wheel sensor");
  PositionSensor *psR = robot->getPositionSensor("right wheel sensor");
  psL->enable(ts);
  psR->enable(ts);

  Lidar *lidar = robot->getLidar("LDS-01");
  lidar->enable(ts);

  InertialUnit *imu = robot->getInertialUnit("inertial unit");
  imu->enable(ts);

  TouchSensor *bL = robot->getTouchSensor("bumper_left");
  TouchSensor *bR = robot->getTouchSensor("bumper_right");
  if (bL)
    bL->enable(ts);
  if (bR)
    bR->enable(ts);

  State state = SCAN_360;
  RobotState rs;

  printf("[Hệ Thống] Bắt đầu khởi động (Phase 1: Tìm góc)...\n");

  while (robot->step(ts) != -1) {
    const double yaw = imu->getRollPitchYaw()[2];
    const double dt = ts / 1000.0;

    // Tính vận tốc góc (lọc bản đồ khi quay nhanh)
    if (rs.first_step) {
      rs.last_yaw = yaw;
      rs.first_step = false;
    }
    const double w_vel = normAng(yaw - rs.last_yaw) / dt;
    rs.last_yaw = yaw;

    const float *img = lidar->getRangeImage();
    if (!img)
      continue;

    // [O6] Odometry tính delta trực tiếp
    const double curL = psL->getValue(), curR = psR->getValue();
    const double delta =
        ((curL - rs.lastEncL) + (curR - rs.lastEncR)) * 0.5 * WHEEL_R;
    rs.rx += delta * std::cos(yaw);
    rs.ry += delta * std::sin(yaw);
    rs.lastEncL = curL;
    rs.lastEncR = curR;

    int gc, gr;
    worldToGrid(rs.rx, rs.ry, gc, gr);
    if ((unsigned)gc < (unsigned)GRID_W && (unsigned)gr < (unsigned)GRID_H)
      grid[gr][gc].status = 1;

    updateMap(img, rs.rx, rs.ry, yaw, w_vel);

    double cv = 0, co = 0;
    const bool bumped =
        (bL && bL->getValue() > 0.5) || (bR && bR->getValue() > 0.5);

    switch (state) {
    // ========================================================
    // PHASE 1: TÌM VÀ ÉP GÓC
    // ========================================================
    case SCAN_360: {
      // [O9] Chụp snapshot 1 bước, không cần rotate
      float min_d = 99.0f;
      int min_idx = 0;
      for (int i = 0; i < 360; i++) {
        float dv = lidarAt(img, i);
        if (dv < min_d) {
          min_d = dv;
          min_idx = i;
        }
      }

      // Nếu không nhận được tín hiệu lidar (tất cả đều ngoài phạm vi)
      if (min_d >= 9.0f) {
        if (bumped) {
          cv = BACKUP_SPD;
        } else {
          cv = FWD_SPD;
        }
        co = 0.0;
      } else {
        rs.wall1_yaw = normAng(yaw - min_idx * DEG2RAD);
        rs.target_yaw = rs.wall1_yaw;
        state = TURN_TO_WALL1;
      }
    } break;

    case TURN_TO_WALL1: {
      double err = normAng(rs.target_yaw - yaw);
      co = KP_T * err;
      if (std::abs(err) < TURN_THR)
        state = APPROACH_WALL1;
    } break;

    case APPROACH_WALL1: {
      float front = lidarFront(img);
      if (bumped)
        state = BACKUP_1;
      else if (front < WALL1_STOP)
        state = FIND_WALL2;
      else {
        cv = APPROACH_SPD;
        co = KP_H * normAng(rs.target_yaw - yaw);
      }
    } break;

    case BACKUP_1:
      cv = BACKUP_SPD;
      if (!bumped)
        state = FIND_WALL2;
      break;

    case FIND_WALL2: {
      float left_d = lidarAvg(img, 80, 100);
      float right_d = lidarAvg(img, 260, 280);
      rs.wall2_dir = (left_d <= right_d) ? +1 : -1;
      rs.target_yaw = normAng(yaw + rs.wall2_dir * M_PI * 0.5);
      state = TURN_TO_WALL2;
    } break;

    case TURN_TO_WALL2: {
      double err = normAng(rs.target_yaw - yaw);
      co = KP_T * err;
      if (std::abs(err) < TURN_THR)
        state = APPROACH_WALL2;
    } break;

    case APPROACH_WALL2: {
      float front = lidarFront(img);
      if (bumped)
        state = BACKUP_2;
      else if (front < WALL2_STOP)
        state = CORNER_ALIGN;
      else {
        cv = WALL2_SPD;
        co = KP_H * normAng(rs.target_yaw - yaw);
      }
    } break;

    case BACKUP_2:
      cv = BACKUP_SPD;
      if (!bumped)
        state = CORNER_ALIGN;
      break;

    case CORNER_ALIGN: {
      double snapped = std::round(yaw / (M_PI * 0.5)) * (M_PI * 0.5);
      double err = normAng(snapped - yaw);
      co = KP_T * err;
      if (std::abs(err) < TURN_THR) {
        rs.base_heading = normAng(snapped + M_PI);
        rs.row_heading = rs.base_heading;
        rs.turn_dir = -rs.wall2_dir;
        rs.target_yaw = rs.base_heading;
        rs.shift_blocked = false;
        printf("[Chuyển đổi] Căn góc xong. Quay 180° để quét bù hàng 1...\n");
        state = TURN_180_RECOVERY;
      }
    } break;

    case TURN_180_RECOVERY: {
      double err = normAng(rs.target_yaw - yaw);
      co = KP_T * err;
      if (std::abs(err) < TURN_THR) {
        printf("[Phase 2] Đã quay xong 180°. Bắt đầu FORWARD quét hàng 1!\n");
        state = FORWARD;
      }
    } break;

    // ========================================================
    // PHASE 2: ZIGZAG SWEEP
    // ========================================================
    case FORWARD: {
      cv = FWD_SPD;
      co = KP_H * normAng(rs.row_heading - yaw);
      if (bumped) {
        state = BACKUP;
      } else if (lidarFront(img) < (float)OBSTACLE ||
                 (rs.is_recovery &&
                  countZerosInDir(gc, gr, rs.row_heading) == 0)) {
        // Tối ưu bám viền (Zigzag hẹp): Nếu đang ở phase phục hồi và phía trước
        // không còn ô 0 nào, quẹo sớm! Dừng thông minh: Kiểm tra ngay nếu map
        // không còn ô không nào
        bool all_done = true;
        for (int i = 0; i < GRID_H; i++) {
          for (int j = 0; j < GRID_W; j++) {
            if (grid[i][j].status == 0) {
              all_done = false;
              break;
            }
          }
          if (!all_done)
            break;
        }

        if (all_done) {
          state = DONE; // Hoàn thiện 100% bản đồ, tắt máy!
        } else if (rs.is_recovery) {
          // Fix: Trong BFS recovery, khi hết ô 0, dùng BFS tìm đường thay vì
          // zigzag mù
          rs.saved_gc = gc;
          rs.saved_gr = gr;
          state = CHECK_MAP;
        } else {
          rs.target_yaw = normAng(rs.row_heading + rs.turn_dir * M_PI * 0.5);
          rs.shift_blocked = false;
          state = TURN_1;
        }
      }
    } break;

    case BACKUP:
      cv = BACKUP_SPD;
      co = KP_H * normAng(rs.row_heading - yaw);
      if (!bumped) {
        if (rs.is_recovery) {
          rs.saved_gc = gc;
          rs.saved_gr = gr;
          state = CHECK_MAP;
        } else {
          rs.target_yaw = normAng(rs.row_heading + rs.turn_dir * M_PI * 0.5);
          rs.shift_blocked = false;
          state = TURN_1;
        }
      }
      break;

    case TURN_1:
      co = KP_T * normAng(rs.target_yaw - yaw);
      if (std::abs(normAng(rs.target_yaw - yaw)) < TURN_THR) {
        if (lidarFront(img) < (float)OBSTACLE) {
          rs.shift_blocked = true;
          rs.resume_heading = rs.row_heading;
          rs.saved_gc = gc;
          rs.saved_gr = gr;
          state = CHECK_MAP;
        } else {
          rs.shift_start_x = rs.rx;
          rs.shift_start_y = rs.ry;
          state = SHIFT;
        }
      }
      break;

    case SHIFT: {
      cv = FOLLOW_SPD;
      co = KP_H * normAng(rs.target_yaw - yaw);
      double dist =
          std::hypot(rs.rx - rs.shift_start_x, rs.ry - rs.shift_start_y);
      bool blocked = lidarFront(img) < (float)OBSTACLE || bumped;
      if (blocked && dist < SHIFT_D * 0.5) {
        rs.shift_blocked = true;
        rs.resume_heading = rs.row_heading;
        rs.saved_gc = gc;
        rs.saved_gr = gr;
        state = CHECK_MAP;
      } else if (dist >= SHIFT_D || blocked) {
        rs.target_yaw = normAng(rs.target_yaw + rs.turn_dir * M_PI * 0.5);
        state = TURN_2;
      }
    } break;

    case TURN_2:
      co = KP_T * normAng(rs.target_yaw - yaw);
      if (std::abs(normAng(rs.target_yaw - yaw)) < TURN_THR) {
        rs.row_heading = rs.target_yaw;
        rs.turn_dir *= -1;
        if (lidarFront(img) < (float)OBSTACLE) {
          rs.resume_heading = rs.row_heading;
          rs.saved_gc = gc;
          rs.saved_gr = gr;
          state = CHECK_MAP;
        } else {
          state = FORWARD;
        }
      }
      break;

    // ========================================================
    // PHASE 3: BFS NAVIGATION
    // ========================================================
    case CHECK_MAP:
      cv = co = 0;
      rs.nav_path = findUnvisited(rs.saved_gc, rs.saved_gr);
      if (rs.nav_path.empty()) {
        state = DONE;
      } else {
        rs.path_idx = 1;
        state = (rs.path_idx >= rs.nav_path.size()) ? RESUME_ALIGN : NAV_TURN;
      }
      break;

    case NAV_TURN: {
      double tx, ty;
      gridToWorld(rs.nav_path[rs.path_idx].c, rs.nav_path[rs.path_idx].r, tx,
                  ty);
      double ang = std::atan2(ty - rs.ry, tx - rs.rx);
      co = KP_T * normAng(ang - yaw);
      if (std::abs(normAng(ang - yaw)) < TURN_THR)
        state = NAV_FORWARD;
    } break;

    case NAV_FORWARD: {
      double tx, ty;
      gridToWorld(rs.nav_path[rs.path_idx].c, rs.nav_path[rs.path_idx].r, tx,
                  ty);
      double ang = std::atan2(ty - rs.ry, tx - rs.rx);
      cv = NAV_SPD;
      co = KP_H * normAng(ang - yaw);

      if (bumped || lidarFront(img) < (float)OBSTACLE) {
        state = NAV_BACKUP;
      } else if (std::hypot(tx - rs.rx, ty - rs.ry) < NAV_DIST_THR) {
        rs.path_idx++;
        state = NAV_TURN;
      }
    } break;

    case NAV_BACKUP:
      cv = BACKUP_SPD;
      if (!bumped) {
        rs.saved_gc = gc;
        rs.saved_gr = gr;
        state = CHECK_MAP;
      }
      break;

    case RESUME_ALIGN:
      cv = 0;
      rs.target_yaw = rs.resume_heading;
      co = KP_T * normAng(rs.target_yaw - yaw);
      if (std::abs(normAng(rs.target_yaw - yaw)) < TURN_THR) {
        rs.row_heading = rs.resume_heading;
        state = FORWARD;
      }
      break;

    // ========================================================
    // PHASE 4: HOÀN THÀNH
    // ========================================================
    case DONE:
      cv = co = 0;
      {
        static bool printed = false;
        if (!printed) {
          printf("✓ Hoàn thành nhiệm vụ! Toàn bộ diện tích đã được phủ.\n");
          printed = true;
        }
      }
      break;
    }

    diffDrive(cv, co, mL, mR);
  }

  delete robot;
  return 0;
}