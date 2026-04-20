// ============================================================
// vacuum_controller_optimized.cpp
// FSM Tích Hợp: Lập Bản Đồ Lưới + BFS + ZigZag
// Phase 1: Tìm góc bằng thuật toán Quét Đa Hướng
//
// CHANGELOG (tối ưu so với bản gốc):
//  [O1] Dùng std::fmod thay cho vòng while trong normAng()
//  [O2] Gộp 2 mảng gridMap/hitMap thành 1 struct GridCell
//  [O3] BFS dùng mảng visited stack-based, tránh copy vector path
//       -> lưu parent array, truy vết ngược
//  [O4] lidarAt() chuẩn hóa index bằng modulo, giữ nguyên
//  [O5] updateMap() chỉ cập nhật khi img hợp lệ, kiểm tra bound 1 lần
//  [O6] Odometry tính delta trực tiếp, tránh lưu 2 biến lastL/lastR riêng lẻ
//  [O7] Tất cả biến trạng thái FSM gom vào struct RobotState
//  [O8] Hàm diffDrive() tránh branch thừa bằng cách tính scale trực tiếp
//  [O9] SCAN_360 thoát ngay sau 1 bước (stateless snapshot)
//  [O10] Hằng số toán học tính sẵn (DEG2RAD)
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
#include <cstdio>
#include <queue>
#include <vector>

using namespace webots;

// ============================================================
// HẰNG SỐ
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

// Bản đồ lưới (phòng 5x5m, ô 0.4m)
static constexpr double CELL_SIZE = 0.4;
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
  // Phase 1: Tìm góc
  SCAN_360,
  TURN_TO_WALL1,
  APPROACH_WALL1,
  BACKUP_1,
  FIND_WALL2,
  TURN_TO_WALL2,
  APPROACH_WALL2,
  BACKUP_2,
  CORNER_ALIGN,
  TURN_180_RECOVERY,
  // Phase 2: ZigZag
  FORWARD,
  BACKUP,
  TURN_1,
  SHIFT,
  TURN_2,
  // Phase 3: BFS
  CHECK_MAP,
  NAV_TURN,
  NAV_FORWARD,
  NAV_BACKUP,
  RESUME_ALIGN,
  // Phase 4
  DONE
};

struct Point2D {
  int c, r;
};

// ============================================================
// [O7] Gom biến trạng thái vào struct
// ============================================================
struct RobotState {
  // Pose
  double rx = 0, ry = 0;
  double lastEncL = 0, lastEncR = 0;

  // Điều hướng chung
  double target_yaw = 0;
  double base_heading = 0;
  double row_heading = 0;
  double resume_heading = 0;
  double last_yaw = 0;
  bool first_step = true;

  // Phase 1
  double wall1_yaw = 0;
  int wall2_dir = 1;

  // Phase 2/3
  double shift_start_x = 0, shift_start_y = 0;
  int turn_dir = 1;
  bool shift_blocked = false;
  int saved_gc = 0, saved_gr = 0;

  // BFS path
  std::vector<Point2D> nav_path;
  size_t path_idx = 0;
  bool is_recovery =
      false; // Cờ theo dõi: Đã tiến vào quá trình phục hồi các phần còn sót
};

// ============================================================
// TIỆN ÍCH
// ============================================================

// [O1] normAng dùng fmod thay cho while-loop
static inline double normAng(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0)
    a += 2.0 * M_PI;
  return a - M_PI;
}

// [O8] diffDrive không có branch thừa
static void diffDrive(double v, double omega, Motor *mL, Motor *mR) {
  double vL = (v - omega * WHEEL_L * 0.5) / WHEEL_R;
  double vR = (v + omega * WHEEL_L * 0.5) / WHEEL_R;
  double s = std::max(std::abs(vL), std::abs(vR));
  double scale = (s > MAX_WS) ? MAX_WS / s : 1.0;
  mL->setVelocity(vL * scale);
  mR->setVelocity(vR * scale);
}

static inline float lidarAt(const float *img, int deg) {
  int idx = ((deg % 360) + 360) % 360;
  float d = img[idx];
  return (d < 0.01f || std::isinf(d) || std::isnan(d)) ? 9.9f : d;
}

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

static void worldToGrid(double x, double y, int &c, int &r) {
  c = (int)std::floor(x / CELL_SIZE) + OFFSET_X;
  r = (int)std::floor(y / CELL_SIZE) + OFFSET_Y;
}

static void gridToWorld(int c, int r, double &x, double &y) {
  x = (c - OFFSET_X) * CELL_SIZE + (CELL_SIZE * 0.5);
  y = (r - OFFSET_Y) * CELL_SIZE + (CELL_SIZE * 0.5);
}

// [O5] updateMap: kiểm tra hợp lệ trước, gom bound-check
static void updateMap(const float *img, double rx, double ry, double ryaw,
                      double w_vel) {
  if (!img || std::abs(w_vel) > 1.0)
    return;
  for (int i = 0; i < 360; i += 5) {
    float r = img[i];
    if (r <= 0.05f || r >= 2.0f)
      continue;
    double th = normAng(ryaw - i * DEG2RAD);
    int gc, gr;
    worldToGrid(rx + r * std::cos(th), ry + r * std::sin(th), gc, gr);
    if ((unsigned)gc >= (unsigned)GRID_W || (unsigned)gr >= (unsigned)GRID_H)
      continue;
    GridCell &cell = grid[gr][gc];
    if (cell.status == 2)
      continue; // đã xác nhận, không cần cập nhật
    int weight = (r < 1.0f) ? 3 : 1;
    cell.hits = (int8_t)std::min((int)cell.hits + weight, 100);
    if (cell.hits > 5)
      cell.status = 2;
  }
}

// ============================================================
// [O3] BFS tối ưu: dùng mảng parent thay vì copy vector path
// ============================================================
std::vector<Point2D> findUnvisited(int sc, int sr) {
  if ((unsigned)sc >= (unsigned)GRID_W || (unsigned)sr >= (unsigned)GRID_H)
    return {};

  // Mảng parent phẳng, -1 = chưa thăm
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

// countZerosInDir: không thay đổi logic, chỉ dùng DEG2RAD được bỏ
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
      rs.wall1_yaw = normAng(yaw - min_idx * DEG2RAD);
      rs.target_yaw = rs.wall1_yaw;
      state = TURN_TO_WALL1;
    } break;

    case TURN_TO_WALL1: {
      double err = normAng(rs.target_yaw - yaw);
      co = KP_T * err;
      if (std::abs(err) < TURN_THR)
        state = APPROACH_WALL1;
    } break;

    case APPROACH_WALL1: {
      float front = lidarAt(img, 0);
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
      float front = lidarAt(img, 0);
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
      } else if (lidarAt(img, 0) < (float)OBSTACLE ||
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
        rs.target_yaw = normAng(rs.row_heading + rs.turn_dir * M_PI * 0.5);
        rs.shift_blocked = false;
        state = TURN_1;
      }
      break;

    case TURN_1:
      co = KP_T * normAng(rs.target_yaw - yaw);
      if (std::abs(normAng(rs.target_yaw - yaw)) < TURN_THR) {
        if (lidarAt(img, 0) < (float)OBSTACLE) {
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
      bool blocked = lidarAt(img, 0) < (float)OBSTACLE || bumped;
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
        if (lidarAt(img, 0) < (float)OBSTACLE) {
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

      if (bumped || lidarAt(img, 0) < (float)OBSTACLE) {
        state = NAV_BACKUP;
      } else if (std::hypot(tx - rs.rx, ty - rs.ry) < NAV_DIST_THR) {
        rs.path_idx++;
        if (rs.path_idx >= rs.nav_path.size()) {
          rs.is_recovery =
              true; // Bật cờ đánh dấu bắt đầu phase dọn phần còn sót
          // Chọn hướng sweep tốt nhất sau khi BFS hoàn thành
          double best_heading = rs.base_heading;
          int max_zeros = -1;
          for (int i = 0; i < 4; i++) {
            double h = rs.base_heading + i * M_PI * 0.5;
            int z = countZerosInDir(gc, gr, h);
            if (z > max_zeros) {
              max_zeros = z;
              best_heading = h;
            }
          }
          rs.resume_heading = normAng(best_heading);
          double lh = normAng(best_heading + M_PI * 0.5);
          double rh = normAng(best_heading - M_PI * 0.5);
          rs.turn_dir =
              (countZerosInDir(gc, gr, lh) >= countZerosInDir(gc, gr, rh)) ? 1
                                                                           : -1;
          state = RESUME_ALIGN;
        } else {
          state = NAV_TURN;
        }
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