// ============================================================
// vacuum_controller_final.cpp
// FSM 15 Trạng thái - Bao phủ ZigZag & Phục hồi BFS
// Đã tối ưu GridMap 40x40 và logic tìm góc (Wall Hunting)
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

static constexpr double FWD_SPD    = 0.25;
static constexpr double FOLLOW_SPD = 0.18;
static constexpr double NAV_SPD    = 0.30;
static constexpr double SHIFT_D    = 0.30;

static constexpr double CORNER_DIST  = 0.22;
static constexpr double FOLLOW_DIST  = 0.25;
static constexpr double TURN_THR     = 0.012; 
static constexpr double NAV_DIST_THR = 0.08;

static constexpr double KP_H    = 4.5;
static constexpr double KP_T    = 5.5;
static constexpr double KP_WALL = 1.5;

// --- HẰNG SỐ BẢN ĐỒ (ĐÃ TỐI ƯU CHO PHÒNG 5x5m) ---
static constexpr double CELL_SIZE = 0.2;
static constexpr int GRID_W = 40;
static constexpr int GRID_H = 40;
static constexpr int OFFSET_X = 20;
static constexpr int OFFSET_Y = 20;

// 0=chưa thăm, 1=đã dọn, 2=vật cản
int gridMap[GRID_H][GRID_W] = {0};

enum State {
    // Phase 1
    FIND_NEAREST_WALL, APPROACH_WALL, TURN_ALIGN_WALL, WALL_HUNTING, CORNER_SNAP,
    // Phase 2
    FORWARD, BACKUP, TURN_1, SHIFT, TURN_2,
    // Phase 3
    CHECK_MAP, NAV_TURN, NAV_FORWARD, NAV_BACKUP, RESUME_ALIGN,
    // Phase 4
    DONE
};

struct Point2D { int c, r; };

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

static float getLidarDist(const float *img, int deg) {
    int idx = (deg + 360) % 360;
    float d = img[idx];
    if (d < 0.01f || std::isinf(d) || std::isnan(d)) return 9.9f;
    return d;
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

// --- BFS TÌM Ô CHƯA THĂM GẦN NHẤT ---
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

        if (gridMap[curr.r][curr.c] == 0) {
            return curr.path; 
        }

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

    State state = FIND_NEAREST_WALL;
    double target_yaw = 0.0, base_heading = 0.0, row_heading = 0.0;
    double rx = 0, ry = 0, lastL = 0, lastR = 0;
    double shift_start_x = 0, shift_start_y = 0;
    int turn_dir = 1;
    bool shift_blocked = false; 
    std::vector<Point2D> nav_path;
    size_t path_idx = 0;

    while (robot->step(ts) != -1) {
        double yaw = imu->getRollPitchYaw()[2];
        const float *img = lidar->getRangeImage();
        if (!img) continue;

        // Cập nhật Odometry
        double curL = psL->getValue(), curR = psR->getValue();
        double d = ((curL - lastL) + (curR - lastR)) / 2.0 * WHEEL_R;
        rx += d * std::cos(yaw); ry += d * std::sin(yaw);
        lastL = curL; lastR = curR;

        // Cập nhật bản đồ
        int gc, gr;
        worldToGrid(rx, ry, gc, gr);
        if (gc >= 0 && gc < GRID_W && gr >= 0 && gr < GRID_H)
            gridMap[gr][gc] = 1; // Đã dọn dẹp
        updateMap(img, rx, ry, yaw);

        double cv = 0, co = 0;
        bool bumped = (bL && bL->getValue() > 0) || (bR && bR->getValue() > 0);

        switch (state) {
            // ---- PHASE 1: TÌM VÀ ÉP GÓC ----
            case FIND_NEAREST_WALL: {
                float min_d = 5.0; int min_idx = 0;
                for (int i = 0; i < 360; i++)
                    if (img[i] > 0.05f && img[i] < min_d) { min_d = img[i]; min_idx = i; }
                target_yaw = normAng(yaw - min_idx * M_PI / 180.0);
                double err = normAng(target_yaw - yaw);
                co = KP_T * err;
                if (std::abs(err) < TURN_THR) state = APPROACH_WALL;
            } break;

            case APPROACH_WALL:
                cv = FOLLOW_SPD;
                co = KP_H * normAng(target_yaw - yaw);
                if (getLidarDist(img, 0) < 0.25f || bumped) {
                    target_yaw = normAng(yaw - M_PI / 2);
                    state = TURN_ALIGN_WALL; // Sửa lỗi: Chuyển sang xoay tại chỗ
                }
                break;

            case TURN_ALIGN_WALL:
                cv = 0; // Dừng lại để xoay
                co = KP_T * normAng(target_yaw - yaw);
                if (std::abs(normAng(target_yaw - yaw)) < TURN_THR) {
                    state = WALL_HUNTING;
                }
                break;

            case WALL_HUNTING:
                cv = FOLLOW_SPD;
                // Bám tường trái
                co = KP_WALL * (getLidarDist(img, 90) - FOLLOW_DIST);
                if (getLidarDist(img, 0) < CORNER_DIST || bumped)
                    state = CORNER_SNAP; // Phát hiện tường thứ 2 -> Góc phòng
                break;

            case CORNER_SNAP:
                cv = 0; co = 0;
                base_heading = std::round(yaw / (M_PI / 2.0)) * (M_PI / 2.0);
                row_heading = base_heading;
                target_yaw = normAng(base_heading + M_PI / 2);
                shift_blocked = false;
                state = TURN_1;
                break;

            // ---- PHASE 2: ZIGZAG SWEEP ----
            case FORWARD:
                cv = FWD_SPD;
                co = KP_H * normAng(row_heading - yaw);
                if (bumped) {
                    state = BACKUP;
                } else if (getLidarDist(img, 0) < 0.25f) {
                    target_yaw = normAng(row_heading + turn_dir * M_PI / 2);
                    shift_blocked = false;
                    state = TURN_1;
                }
                break;

            case BACKUP:
                cv = -0.10;
                co = KP_H * normAng(row_heading - yaw);
                if (!bumped) {
                    target_yaw = normAng(row_heading + turn_dir * M_PI / 2);
                    shift_blocked = false;
                    state = TURN_1;
                }
                break;

            case TURN_1:
                co = KP_T * normAng(target_yaw - yaw);
                if (std::abs(normAng(target_yaw - yaw)) < TURN_THR) {
                    if (getLidarDist(img, 0) < 0.15f) {
                        shift_blocked = true;
                        state = CHECK_MAP;
                    } else {
                        shift_start_x = rx;
                        shift_start_y = ry;
                        state = SHIFT;
                    }
                }
                break;

            case SHIFT:
                cv = FOLLOW_SPD;
                co = KP_H * normAng(target_yaw - yaw);
                {
                    double dist_shifted = std::hypot(rx - shift_start_x, ry - shift_start_y);
                    bool front_blocked = getLidarDist(img, 0) < 0.15f || bumped;

                    if (front_blocked && dist_shifted < SHIFT_D * 0.5) {
                        shift_blocked = true;
                        state = CHECK_MAP;
                    } else if (dist_shifted >= SHIFT_D || front_blocked) {
                        target_yaw = normAng(target_yaw + turn_dir * M_PI / 2);
                        state = TURN_2;
                    }
                }
                break;

            case TURN_2:
                co = KP_T * normAng(target_yaw - yaw);
                if (std::abs(normAng(target_yaw - yaw)) < TURN_THR) {
                    row_heading = target_yaw;
                    turn_dir *= -1;

                    if (getLidarDist(img, 0) < 0.25f) {
                        state = CHECK_MAP;
                    } else {
                        state = FORWARD;
                    }
                }
                break;

            // ---- PHASE 3: BFS NAVIGATION ----
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

                if (bumped || getLidarDist(img, 0) < 0.12f) {
                    state = NAV_BACKUP;
                } else if (std::hypot(tx - rx, ty - ry) < NAV_DIST_THR) {
                    path_idx++;
                    if (path_idx >= nav_path.size()) state = RESUME_ALIGN;
                    else state = NAV_TURN;
                }
            } break;

            case NAV_BACKUP:
                cv = -0.10;
                if (!bumped) {
                    state = CHECK_MAP;
                }
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

            // ---- PHASE 4: HOÀN THÀNH ----
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