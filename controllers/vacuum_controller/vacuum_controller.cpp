// ============================================================
// Robot_Vacuum_Webots_CornerSnap_Full.cpp
// Tối ưu cho sàn 5x5m - Ép góc chuẩn và điều hướng thông minh
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
static constexpr double TURN_THR     = 0.01;
static constexpr double NAV_DIST_THR = 0.08;

static constexpr double KP_H    = 4.5; 
static constexpr double KP_T    = 5.5; 
static constexpr double KP_WALL = 1.5; 

// --- HẰNG SỐ BẢN ĐỒ (GRID MAP) ---
static constexpr double CELL_SIZE = 0.2;
static constexpr int GRID_W = 100;
static constexpr int GRID_H = 100;
static constexpr int OFFSET_X = 50;
static constexpr int OFFSET_Y = 50;

int gridMap[GRID_H][GRID_W] = {0}; 

enum State {
    FIND_NEAREST_WALL, APPROACH_WALL, WALL_HUNTING, CORNER_SNAP,
    FORWARD, BACKUP, TURN_1, SHIFT, TURN_2,
    CHECK_MAP, NAV_TURN, NAV_FORWARD, NAV_BACKUP, RESUME_ALIGN,
    DONE
};

struct Point2D { int c, r; };

// --- HÀM HỖ TRỢ TOÁN HỌC ---
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

// --- HÀM XỬ LÝ LIDAR & BẢN ĐỒ ---
static float getLidarDist(const float *img, int deg) {
    int idx = (deg + 360) % 360;
    return img[idx];
}

static void worldToGrid(double x, double y, int &c, int &r) {
    c = (int)std::floor(x / CELL_SIZE) + OFFSET_X;
    r = (int)std::floor(y / CELL_SIZE) + OFFSET_Y;
}

static void gridToWorld(int c, int r, double &x, double &y) {
    x = (c - OFFSET_X) * CELL_SIZE + (CELL_SIZE / 2.0);
    y = (r - OFFSET_Y) * CELL_SIZE + (CELL_SIZE / 2.0);
}

static void updateMap(const float* img, double rx, double ry, double ryaw) {
    for (int i = 0; i < 360; i += 5) {
        float r = img[i];
        if (r > 0.05f && r < 3.5f) {
            double th = normAng(ryaw - (double)i * M_PI / 180.0);
            int gc, gr;
            worldToGrid(rx + r * std::cos(th), ry + r * std::sin(th), gc, gr);
            if (gc >= 0 && gc < GRID_W && gr >= 0 && gr < GRID_H) {
                if(gridMap[gr][gc] == 0) gridMap[gr][gc] = 2; // Đánh dấu vật cản
            }
        }
    }
}

// --- THUẬT TOÁN TÌM VÙNG TRỐNG (BFS) ---
std::vector<Point2D> findUnvisited(int sc, int sr) {
    static bool visited[GRID_H][GRID_W];
    for(int i=0; i<GRID_H; i++) for(int j=0; j<GRID_W; j++) visited[i][j] = false;

    struct Node { int c, r; std::vector<Point2D> path; };
    std::queue<Node> q;
    q.push({sc, sr, {}});
    visited[sr][sc] = true;
    
    int dc[] = {0, 0, -1, 1}, dr[] = {-1, 1, 0, 0};
    while (!q.empty()) {
        Node curr = q.front(); q.pop();
        if (gridMap[curr.r][curr.c] == 0) {
            curr.path.push_back({curr.c, curr.r});
            return curr.path;
        }
        for (int i = 0; i < 4; i++) {
            int nc = curr.c + dc[i], nr = curr.r + dr[i];
            if (nc>=0 && nc<GRID_W && nr>=0 && nr<GRID_H && !visited[nr][nc] && gridMap[nr][nc] != 2) {
                visited[nr][nc] = true;
                std::vector<Point2D> nextP = curr.path;
                nextP.push_back({nc, nr});
                q.push({nc, nr, nextP});
            }
        }
    }
    return {};
}

// --- MAIN ---
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
    if(bL) bL->enable(ts); if(bR) bR->enable(ts);

    State state = FIND_NEAREST_WALL;
    double target_yaw = 0.0, base_heading = 0.0, row_heading = 0.0;
    double rx = 0, ry = 0, lastL = 0, lastR = 0;
    double shift_start_x = 0, shift_start_y = 0;
    int turn_dir = 1;
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

        // Cập nhật bản đồ: Đã dọn (1) và Vật cản (2)
        int gc, gr; worldToGrid(rx, ry, gc, gr);
        if (gc>=0 && gc<GRID_W && gr>=0 && gr<GRID_H) gridMap[gr][gc] = 1;
        updateMap(img, rx, ry, yaw);

        double cv = 0, co = 0;
        bool bumped = (bL && bL->getValue() > 0) || (bR && bR->getValue() > 0);

        switch (state) {
            case FIND_NEAREST_WALL: {
                float min_d = 5.0; int min_idx = 0;
                for(int i=0; i<360; i++) if(img[i]>0.05 && img[i]<min_d) { min_d=img[i]; min_idx=i; }
                target_yaw = normAng(yaw - min_idx * M_PI / 180.0);
                double err = normAng(target_yaw - yaw);
                co = KP_T * err;
                if (std::abs(err) < TURN_THR) state = APPROACH_WALL;
            } break;

            case APPROACH_WALL:
                cv = FOLLOW_SPD; co = KP_H * normAng(target_yaw - yaw);
                if (getLidarDist(img, 0) < 0.2 || bumped) { 
                    target_yaw = normAng(yaw - M_PI/2); 
                    state = WALL_HUNTING; 
                }
                break;

            case WALL_HUNTING:
                cv = FOLLOW_SPD; co = KP_WALL * (getLidarDist(img, 90) - FOLLOW_DIST);
                if (getLidarDist(img, 0) < CORNER_DIST || bumped) state = CORNER_SNAP;
                break;

            case CORNER_SNAP:
                base_heading = std::round(yaw / (M_PI/2.0)) * (M_PI/2.0); 
                row_heading = base_heading;
                target_yaw = normAng(base_heading + M_PI/2);
                state = TURN_1;
                break;

            case FORWARD:
                cv = FWD_SPD; co = KP_H * normAng(row_heading - yaw);
                if (getLidarDist(img, 0) < 0.25 || bumped) { 
                    state = bumped ? BACKUP : TURN_1; 
                    target_yaw = normAng(row_heading + turn_dir * M_PI/2); 
                }
                break;

            case BACKUP:
                cv = -0.1; if (!bumped) state = TURN_1;
                break;

            case TURN_1:
                co = KP_T * normAng(target_yaw - yaw);
                if (std::abs(normAng(target_yaw - yaw)) < TURN_THR) {
                    state = SHIFT;
                    shift_start_x = rx; shift_start_y = ry;
                }
                break;

            case SHIFT:
                cv = FOLLOW_SPD; co = KP_H * normAng(target_yaw - yaw);
                if (std::hypot(rx - shift_start_x, ry - shift_start_y) >= SHIFT_D || getLidarDist(img, 0) < 0.2 || bumped) { 
                    target_yaw = normAng(target_yaw + turn_dir * M_PI/2); 
                    state = TURN_2; 
                }
                break;

            case TURN_2:
                co = KP_T * normAng(target_yaw - yaw);
                if (std::abs(normAng(target_yaw - yaw)) < TURN_THR) { 
                    row_heading = target_yaw; 
                    turn_dir *= -1; 
                    state = FORWARD; 
                }
                break;

            case CHECK_MAP:
                nav_path = findUnvisited(gc, gr);
                if (nav_path.empty()) state = DONE;
                else { path_idx = 0; state = NAV_TURN; }
                break;

            case NAV_TURN: {
                double tx, ty; gridToWorld(nav_path[path_idx].c, nav_path[path_idx].r, tx, ty);
                target_yaw = std::atan2(ty - ry, tx - rx);
                co = KP_T * normAng(target_yaw - yaw);
                if (std::abs(normAng(target_yaw - yaw)) < TURN_THR) state = NAV_FORWARD;
            } break;

            case NAV_FORWARD: {
                double tx, ty; gridToWorld(nav_path[path_idx].c, nav_path[path_idx].r, tx, ty);
                cv = NAV_SPD; co = KP_H * normAng(std::atan2(ty-ry, tx-rx) - yaw);
                if (std::hypot(tx-rx, ty-ry) < NAV_DIST_THR) {
                    if (++path_idx >= nav_path.size()) state = RESUME_ALIGN;
                    else state = NAV_TURN;
                }
            } break;

            case RESUME_ALIGN:
                target_yaw = base_heading; 
                co = KP_T * normAng(target_yaw - yaw);
                if (std::abs(normAng(target_yaw - yaw)) < TURN_THR) state = FORWARD;
                break;

            case DONE:
                cv = 0; co = 0;
                static bool printed = false;
                if(!printed) { printf("Hoàn thành nhiệm vụ dọn dẹp!\n"); printed = true; }
                break;
        }
        diffDrive(cv, co, mL, mR);
    }
    delete robot;
    return 0;
}