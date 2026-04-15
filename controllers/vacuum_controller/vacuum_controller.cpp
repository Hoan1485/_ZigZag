// ============================================================
// vacuum_controller_final.cpp
// FSM Đã Tích Hợp: Lập Bản Đồ Lưới + BFS + ZigZag
// Tối ưu Phase 1: Tìm góc bằng thuật toán Quét Đa Hướng
// ============================================================

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Lidar.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/TouchSensor.hpp>

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>
#include <queue>

using namespace webots;

// --- HẰNG SỐ VẬT LÝ & ĐIỀU KHIỂN ---
static constexpr double WHEEL_R = 0.031;
static constexpr double WHEEL_L = 0.258;
static constexpr double MAX_WS  = 6.28;

static constexpr double APPROACH_SPD = 0.20;
static constexpr double WALL2_SPD    = 0.18;
static constexpr double BACKUP_SPD   = -0.10;
static constexpr double FWD_SPD      = 0.25;
static constexpr double FOLLOW_SPD   = 0.18;
static constexpr double NAV_SPD      = 0.30;
static constexpr double SHIFT_D      = 0.30;

static constexpr double WALL1_STOP   = 0.26;
static constexpr double WALL2_STOP   = 0.24;
static constexpr double TURN_THR     = 0.012; 
static constexpr double NAV_DIST_THR = 0.08;

static constexpr double KP_H    = 4.5;
static constexpr double KP_T    = 5.5;

// --- HẰNG SỐ BẢN ĐỒ (PHÒNG 5x5m) ---
static constexpr double CELL_SIZE = 0.2;
static constexpr int GRID_W = 40;
static constexpr int GRID_H = 40;
static constexpr int OFFSET_X = 20;
static constexpr int OFFSET_Y = 20;

// 0=chưa thăm, 1=đã dọn, 2=vật cản
int gridMap[GRID_H][GRID_W] = {0};

enum State {
    // Phase 1: Logic tìm góc ưu việt (Từ File 2)
    SCAN_360, TURN_TO_WALL1, APPROACH_WALL1, BACKUP_1,
    FIND_WALL2, TURN_TO_WALL2, APPROACH_WALL2, BACKUP_2, CORNER_ALIGN,
    
    // Phase 2: ZigZag Sweep (Từ File 1)
    FORWARD, BACKUP, TURN_1, SHIFT, TURN_2,
    
    // Phase 3: Phục hồi và điều hướng BFS (Từ File 1)
    CHECK_MAP, NAV_TURN, NAV_FORWARD, NAV_BACKUP, RESUME_ALIGN,
    
    // Phase 4: Hoàn thành
    DONE
};

struct Point2D { int c, r; };

// --- CÁC HÀM TIỆN ÍCH MÔI TRƯỜNG & TOÁN HỌC ---
static double normAng(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static void diffDrive(double v, double omega, Motor *mL, Motor *mR) {
    double vL = (v - omega * WHEEL_L / 2.0) / WHEEL_R;
    double vR = (v + omega * WHEEL_L / 2.0) / WHEEL_R;
    double s = std::max(std::abs(vL), std::abs(vR));
    if (s > MAX_WS) { vL *= MAX_WS / s; vR *= MAX_WS / s; }
    mL->setVelocity(vL);
    mR->setVelocity(vR);
}

static float lidarAt(const float *img, int deg) {
    int idx = ((deg % 360) + 360) % 360;
    float d = img[idx];
    if (d < 0.01f || std::isinf(d) || std::isnan(d)) return 9.9f;
    return d;
}

static float lidarAvg(const float *img, int deg_start, int deg_end) {
    float sum = 0; int cnt = 0;
    for (int d = deg_start; d <= deg_end; d++) {
        float v = lidarAt(img, d);
        if (v < 9.0f) { sum += v; cnt++; }
    }
    return (cnt > 0) ? sum / cnt : 9.9f;
}

static void worldToGrid(double x, double y, int &c, int &r) {
    c = (int)std::floor(x / CELL_SIZE) + OFFSET_X;
    r = (int)std::floor(y / CELL_SIZE) + OFFSET_Y;
}

static void gridToWorld(int c, int r, double &x, double &y) {
    x = (c - OFFSET_X) * CELL_SIZE + (CELL_SIZE / 2.0);
    y = (r - OFFSET_Y) * CELL_SIZE + (CELL_SIZE / 2.0);
}

static void updateMap(const float *img, double rx, double ry, double ryaw) {
    for (int i = 0; i < 360; i += 5) {
        float r = img[i];
        if (r > 0.05f && r < 3.5f) {
            double th = normAng(ryaw - (double)i * M_PI / 180.0);
            int gc, gr;
            worldToGrid(rx + r * std::cos(th), ry + r * std::sin(th), gc, gr);
            if (gc >= 0 && gc < GRID_W && gr >= 0 && gr < GRID_H)
                if (gridMap[gr][gc] == 0) gridMap[gr][gc] = 2; // Ghi nhận vật cản
        }
    }
}

// --- BFS TÌM ĐƯỜNG ĐẾN Ô TRỐNG ---
std::vector<Point2D> findUnvisited(int sc, int sr) {
    if (sc < 0 || sc >= GRID_W || sr < 0 || sr >= GRID_H) return {};
    static bool visited[GRID_H][GRID_W];
    for (int i = 0; i < GRID_H; i++)
        for (int j = 0; j < GRID_W; j++)
            visited[i][j] = false;

    struct Node { int c, r; std::vector<Point2D> path; };
    std::queue<Node> q;
    q.push({sc, sr, {{sc, sr}}});
    visited[sr][sc] = true;

    int dc[] = {0, 0, -1, 1}, dr[] = {-1, 1, 0, 0};
    while (!q.empty()) {
        Node curr = q.front(); q.pop();

        if (gridMap[curr.r][curr.c] == 0) return curr.path; 

        for (int i = 0; i < 4; i++) {
            int nc = curr.c + dc[i], nr = curr.r + dr[i];
            if (nc >= 0 && nc < GRID_W && nr >= 0 && nr < GRID_H
                && !visited[nr][nc] && gridMap[nr][nc] != 2) {
                visited[nr][nc] = true;
                std::vector<Point2D> nextP = curr.path;
                nextP.push_back({nc, nr});
                q.push({nc, nr, nextP});
            }
        }
    }
    return {}; 
}

// ============================================================
int main() {
    Robot *robot = new Robot();
    int ts = (int)robot->getBasicTimeStep();

    Motor *mL = robot->getMotor("left wheel motor");
    Motor *mR = robot->getMotor("right wheel motor");
    mL->setPosition(INFINITY); mR->setPosition(INFINITY);

    PositionSensor *psL = robot->getPositionSensor("left wheel sensor");
    psL->enable(ts);
    PositionSensor *psR = robot->getPositionSensor("right wheel sensor");
    psR->enable(ts);

    Lidar *lidar = robot->getLidar("LDS-01");
    lidar->enable(ts);

    InertialUnit *imu = robot->getInertialUnit("inertial unit");
    imu->enable(ts);

    TouchSensor *bL = robot->getTouchSensor("bumper_left");
    TouchSensor *bR = robot->getTouchSensor("bumper_right");
    if (bL) bL->enable(ts);
    if (bR) bR->enable(ts);

    State state = SCAN_360;
    
    // Biến điều hướng dùng chung
    double target_yaw = 0.0, base_heading = 0.0, row_heading = 0.0;
    double rx = 0, ry = 0, lastL = 0, lastR = 0;
    
    // Biến Phase 1
    double wall1_yaw = 0.0;
    int wall2_dir = 1;

    // Biến Phase 2 & 3
    double shift_start_x = 0, shift_start_y = 0;
    int turn_dir = 1;
    bool shift_blocked = false; 
    std::vector<Point2D> nav_path;
    size_t path_idx = 0;

    printf("[Hệ Thống] Bắt đầu khởi động (Phase 1: Tìm góc)...\n");

    while (robot->step(ts) != -1) {
        double yaw = imu->getRollPitchYaw()[2];
        const float *img = lidar->getRangeImage();
        if (!img) continue;

        // Cập nhật Odometry & Map
        double curL = psL->getValue(), curR = psR->getValue();
        double d = ((curL - lastL) + (curR - lastR)) / 2.0 * WHEEL_R;
        rx += d * std::cos(yaw); ry += d * std::sin(yaw);
        lastL = curL; lastR = curR;

        int gc, gr;
        worldToGrid(rx, ry, gc, gr);
        if (gc >= 0 && gc < GRID_W && gr >= 0 && gr < GRID_H)
            gridMap[gr][gc] = 1; // Đánh dấu ô đã đi qua
        updateMap(img, rx, ry, yaw);

        double cv = 0, co = 0;
        bool bumped = (bL && bL->getValue() > 0.5) || (bR && bR->getValue() > 0.5);

        switch (state) {
            // ========================================================
            // PHASE 1: TÌM VÀ ÉP GÓC (Được tối ưu từ File 2)
            // ========================================================
            case SCAN_360: {
                float min_d = 99.0f; int min_idx = 0;
                for (int i = 0; i < 360; i++) {
                    float d_val = lidarAt(img, i);
                    if (d_val < min_d) { min_d = d_val; min_idx = i; }
                }
                wall1_yaw  = normAng(yaw - min_idx * M_PI / 180.0);
                target_yaw = wall1_yaw;
                state = TURN_TO_WALL1;
            } break;

            case TURN_TO_WALL1: {
                double err = normAng(target_yaw - yaw);
                co = KP_T * err;
                if (std::abs(err) < TURN_THR) state = APPROACH_WALL1;
            } break;

            case APPROACH_WALL1: {
                float front = lidarAt(img, 0);
                if (bumped) { state = BACKUP_1; }
                else if (front < WALL1_STOP) { state = FIND_WALL2; }
                else {
                    cv = APPROACH_SPD;
                    co = KP_H * normAng(target_yaw - yaw);
                }
            } break;

            case BACKUP_1: {
                cv = BACKUP_SPD;
                if (!bumped) state = FIND_WALL2;
            } break;

            case FIND_WALL2: {
                float left_d  = lidarAvg(img, 80, 100);
                float right_d = lidarAvg(img, 260, 280);
                if (left_d <= right_d) {
                    wall2_dir  = +1; // Tường trái gần -> Xoay trái
                    target_yaw = normAng(yaw + M_PI / 2.0);
                } else {
                    wall2_dir  = -1; // Tường phải gần -> Xoay phải
                    target_yaw = normAng(yaw - M_PI / 2.0);
                }
                state = TURN_TO_WALL2;
            } break;

            case TURN_TO_WALL2: {
                double err = normAng(target_yaw - yaw);
                co = KP_T * err;
                if (std::abs(err) < TURN_THR) state = APPROACH_WALL2;
            } break;

            case APPROACH_WALL2: {
                float front = lidarAt(img, 0);
                if (bumped) { state = BACKUP_2; }
                else if (front < WALL2_STOP) { state = CORNER_ALIGN; }
                else {
                    cv = WALL2_SPD;
                    co = KP_H * normAng(target_yaw - yaw);
                }
            } break;

            case BACKUP_2: {
                cv = BACKUP_SPD;
                if (!bumped) state = CORNER_ALIGN;
            } break;

            case CORNER_ALIGN: {
                double snapped = std::round(yaw / (M_PI / 2.0)) * (M_PI / 2.0);
                double err = normAng(snapped - yaw);
                co = KP_T * err;
                if (std::abs(err) < TURN_THR) {
                    // TRUYỀN PARAM TỪ PHASE 1 SANG PHASE 2
                    base_heading = snapped;
                    row_heading = base_heading;
                    // Hướng mở rộng vùng quét sẽ cùng chiều với chiều đã phát hiện tường thứ 2
                    turn_dir = wall2_dir; 
                    target_yaw = normAng(row_heading + turn_dir * M_PI / 2.0);
                    shift_blocked = false;
                    printf("[Chuyển đổi] Căn góc xong. Bắt đầu Phase 2: ZigZag Sweep\n");
                    state = TURN_1; // Bắt đầu hàng ZigZag đầu tiên bằng cách quay vào không gian trống
                }
            } break;

            // ========================================================
            // PHASE 2: ZIGZAG SWEEP (Từ File 1)
            // ========================================================
            case FORWARD:
                cv = FWD_SPD;
                co = KP_H * normAng(row_heading - yaw);
                if (bumped) {
                    state = BACKUP;
                } else if (lidarAt(img, 0) < 0.25f) {
                    target_yaw = normAng(row_heading + turn_dir * M_PI / 2.0);
                    shift_blocked = false;
                    state = TURN_1;
                }
                break;

            case BACKUP:
                cv = -0.10;
                co = KP_H * normAng(row_heading - yaw);
                if (!bumped) {
                    target_yaw = normAng(row_heading + turn_dir * M_PI / 2.0);
                    shift_blocked = false;
                    state = TURN_1;
                }
                break;

            case TURN_1:
                co = KP_T * normAng(target_yaw - yaw);
                if (std::abs(normAng(target_yaw - yaw)) < TURN_THR) {
                    if (lidarAt(img, 0) < 0.15f) {
                        shift_blocked = true;
                        state = CHECK_MAP;
                    } else {
                        shift_start_x = rx; shift_start_y = ry;
                        state = SHIFT;
                    }
                }
                break;

            case SHIFT:
                cv = FOLLOW_SPD;
                co = KP_H * normAng(target_yaw - yaw);
                {
                    double dist_shifted = std::hypot(rx - shift_start_x, ry - shift_start_y);
                    bool front_blocked = lidarAt(img, 0) < 0.15f || bumped;

                    if (front_blocked && dist_shifted < SHIFT_D * 0.5) {
                        shift_blocked = true;
                        state = CHECK_MAP;
                    } else if (dist_shifted >= SHIFT_D || front_blocked) {
                        target_yaw = normAng(target_yaw + turn_dir * M_PI / 2.0);
                        state = TURN_2;
                    }
                }
                break;

            case TURN_2:
                co = KP_T * normAng(target_yaw - yaw);
                if (std::abs(normAng(target_yaw - yaw)) < TURN_THR) {
                    row_heading = target_yaw;
                    turn_dir *= -1; // Đảo chiều cho hàng chữ U tiếp theo
                    if (lidarAt(img, 0) < 0.25f) {
                        state = CHECK_MAP;
                    } else {
                        state = FORWARD;
                    }
                }
                break;

            // ========================================================
            // PHASE 3: BFS NAVIGATION (Từ File 1)
            // ========================================================
            case CHECK_MAP:
                cv = 0; co = 0;
                nav_path = findUnvisited(gc, gr);
                if (nav_path.empty()) {
                    state = DONE;
                } else {
                    path_idx = 1;
                    if (path_idx >= nav_path.size()) state = RESUME_ALIGN;
                    else state = NAV_TURN;
                }
                break;

            case NAV_TURN: {
                double tx, ty;
                gridToWorld(nav_path[path_idx].c, nav_path[path_idx].r, tx, ty);
                double ang_to_target = std::atan2(ty - ry, tx - rx);
                co = KP_T * normAng(ang_to_target - yaw);
                if (std::abs(normAng(ang_to_target - yaw)) < TURN_THR)
                    state = NAV_FORWARD;
            } break;

            case NAV_FORWARD: {
                double tx, ty;
                gridToWorld(nav_path[path_idx].c, nav_path[path_idx].r, tx, ty);
                double ang_to_target = std::atan2(ty - ry, tx - rx);
                cv = NAV_SPD;
                co = KP_H * normAng(ang_to_target - yaw);

                if (bumped || lidarAt(img, 0) < 0.12f) {
                    state = NAV_BACKUP;
                } else if (std::hypot(tx - rx, ty - ry) < NAV_DIST_THR) {
                    path_idx++;
                    if (path_idx >= nav_path.size()) state = RESUME_ALIGN;
                    else state = NAV_TURN;
                }
            } break;

            case NAV_BACKUP:
                cv = -0.10;
                if (!bumped) state = CHECK_MAP;
                break;

            case RESUME_ALIGN:
                cv = 0;
                target_yaw = base_heading;
                co = KP_T * normAng(target_yaw - yaw);
                if (std::abs(normAng(target_yaw - yaw)) < TURN_THR) {
                    row_heading = base_heading;
                    state = FORWARD;
                }
                break;

            // ========================================================
            // PHASE 4: HOÀN THÀNH (Từ File 1)
            // ========================================================
            case DONE:
                cv = 0; co = 0;
                {
                    static bool printed = false;
                    if (!printed) {
                        printf("✓ Hoàn thành nhiệm vụ dọn dẹp! Toàn bộ diện tích đã được phủ.\n");
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