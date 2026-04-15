// ============================================================
// corner_finder.cpp
// Nhiệm vụ duy nhất: Đi từ vị trí bất kỳ đến góc tường gần nhất
//
// Logic 4 pha:
//   Phase 1 - SCAN_360    : Quét 360° LiDAR, chọn tường gần nhất
//   Phase 2 - APPROACH    : Tiến thẳng về tường thứ nhất
//   Phase 3 - FIND_WALL2  : Quét trái/phải LiDAR, chọn bên có tường gần hơn
//                           -> quay sang bên đó, tiến vào góc
//   Phase 4 - CORNER_ALIGN: Điều chỉnh hướng chuẩn cho ZigZag
//   Phase 5 - DONE        : Đứng yên tại góc
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

using namespace webots;

// --- HẰNG SỐ VẬT LÝ ---
static constexpr double WHEEL_R   = 0.031;
static constexpr double WHEEL_L   = 0.258;
static constexpr double MAX_WS    = 6.28;

// --- TỐC ĐỘ ---
static constexpr double APPROACH_SPD = 0.20;   // Tiến về tường 1
static constexpr double WALL2_SPD    = 0.18;   // Tiến về tường 2
static constexpr double BACKUP_SPD   = -0.10;  // Lùi khi va chạm

// --- NGƯỠNG KHOẢNG CÁCH ---
static constexpr double WALL1_STOP   = 0.26;   // Dừng trước tường thứ nhất (m)
static constexpr double WALL2_STOP   = 0.24;   // Dừng trước tường thứ hai - góc (m)
static constexpr double CORNER_DIST  = 0.22;   // Phát hiện đã vào góc (m)
static constexpr double SIDE_THR     = 0.60;   // Ngưỡng so sánh tường trái/phải (m)

// --- ĐIỀU KHIỂN ---
static constexpr double TURN_THR  = 0.012;  // Sai số yaw coi như đã xoay xong (rad)
static constexpr double KP_H      = 4.5;    // Hệ số P giữ hướng
static constexpr double KP_T      = 5.5;    // Hệ số P xoay tại chỗ

// --- TRẠNG THÁI FSM ---
enum State {
    SCAN_360,       // Quét 360°, tính hướng tường gần nhất
    TURN_TO_WALL1,  // Xoay mặt về tường thứ nhất
    APPROACH_WALL1, // Tiến về tường thứ nhất
    BACKUP_1,       // Lùi nếu va chạm bumper ở tường 1
    FIND_WALL2,     // So sánh LiDAR trái/phải -> chọn hướng quay
    TURN_TO_WALL2,  // Xoay mặt về tường thứ hai
    APPROACH_WALL2, // Tiến vào góc (tường thứ hai)
    BACKUP_2,       // Lùi nếu va chạm ở tường 2
    CORNER_ALIGN,   // Xoay hướng chuẩn bị ZigZag
    DONE            // Hoàn thành, đứng yên tại góc
};

// --- HÀM TIỆN ÍCH ---
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

// Đọc khoảng cách LiDAR tại góc deg (0 = phía trước robot)
// LiDAR quay ngược chiều kim đồng hồ: 90=trái, 270=phải
static float lidarAt(const float *img, int deg) {
    int idx = ((deg % 360) + 360) % 360;
    float d = img[idx];
    if (d < 0.01f || std::isinf(d) || std::isnan(d)) return 9.9f;
    return d;
}

// Lấy khoảng cách trung bình trong cung [deg_start, deg_end] (step=1°)
static float lidarAvg(const float *img, int deg_start, int deg_end) {
    float sum = 0; int cnt = 0;
    for (int d = deg_start; d <= deg_end; d++) {
        float v = lidarAt(img, d);
        if (v < 9.0f) { sum += v; cnt++; }
    }
    return (cnt > 0) ? sum / cnt : 9.9f;
}

// ============================================================
int main() {
    Robot *robot = new Robot();
    int ts = (int)robot->getBasicTimeStep();

    // --- Khởi tạo thiết bị ---
    Motor *mL = robot->getMotor("left wheel motor");
    Motor *mR = robot->getMotor("right wheel motor");
    mL->setPosition(INFINITY);
    mR->setPosition(INFINITY);

    Lidar *lidar = robot->getLidar("LDS-01");
    lidar->enable(ts);

    InertialUnit *imu = robot->getInertialUnit("inertial unit");
    imu->enable(ts);

    TouchSensor *bL = robot->getTouchSensor("bumper_left");
    TouchSensor *bR = robot->getTouchSensor("bumper_right");
    if (bL) bL->enable(ts);
    if (bR) bR->enable(ts);

    // --- Biến trạng thái ---
    State state      = SCAN_360;
    double target_yaw = 0.0;
    double wall1_yaw  = 0.0;   // Hướng đến tường thứ nhất (world frame)
    int    wall2_dir  = 1;     // +1 = trái (90°), -1 = phải (270°)
    double zigzag_yaw = 0.0;   // Hướng chuẩn ZigZag sau khi vào góc

    printf("[CornerFinder] Khởi động - Bắt đầu SCAN_360\n");

    while (robot->step(ts) != -1) {
        double yaw = imu->getRollPitchYaw()[2];
        const float *img = lidar->getRangeImage();
        if (!img) continue;

        bool bumped = (bL && bL->getValue() > 0.5) ||
                      (bR && bR->getValue() > 0.5);

        double cv = 0.0, co = 0.0;

        switch (state) {

            // --------------------------------------------------------
            // SCAN_360: Quét toàn bộ 360° LiDAR
            //   -> Tìm chỉ số góc có khoảng cách nhỏ nhất
            //   -> Tính target_yaw = hướng thế giới tới tường đó
            // --------------------------------------------------------
            case SCAN_360: {
                float min_d = 99.0f;
                int   min_idx = 0;
                for (int i = 0; i < 360; i++) {
                    float d = lidarAt(img, i);
                    if (d < min_d) { min_d = d; min_idx = i; }
                }
                // LiDAR index i (độ) tính ngược chiều kim từ trục robot
                // yaw thế giới = yaw_robot - i * π/180
                wall1_yaw  = normAng(yaw - min_idx * M_PI / 180.0);
                target_yaw = wall1_yaw;
                printf("[SCAN_360] Tường gần nhất: %.2fm tại chỉ số %d° | Target yaw: %.2f rad\n",
                       min_d, min_idx, target_yaw);
                state = TURN_TO_WALL1;
            } break;

            // --------------------------------------------------------
            // TURN_TO_WALL1: Xoay tại chỗ về tường thứ nhất
            // --------------------------------------------------------
            case TURN_TO_WALL1: {
                double err = normAng(target_yaw - yaw);
                co = KP_T * err;
                if (std::abs(err) < TURN_THR) {
                    printf("[TURN_TO_WALL1] Xoay xong -> APPROACH_WALL1\n");
                    state = APPROACH_WALL1;
                }
            } break;

            // --------------------------------------------------------
            // APPROACH_WALL1: Tiến thẳng đến tường 1
            //   Dừng khi LiDAR phía trước < WALL1_STOP hoặc bumper
            // --------------------------------------------------------
            case APPROACH_WALL1: {
                float front = lidarAt(img, 0);
                if (bumped) {
                    printf("[APPROACH_WALL1] Va chạm bumper -> BACKUP_1\n");
                    state = BACKUP_1;
                } else if (front < WALL1_STOP) {
                    printf("[APPROACH_WALL1] Đã đến tường 1 (%.2fm) -> FIND_WALL2\n", front);
                    state = FIND_WALL2;
                } else {
                    cv = APPROACH_SPD;
                    co = KP_H * normAng(target_yaw - yaw);
                }
            } break;

            // --------------------------------------------------------
            // BACKUP_1: Lùi ngắn khi bị bumper ở tường 1
            // --------------------------------------------------------
            case BACKUP_1: {
                cv = BACKUP_SPD;
                if (!bumped) {
                    printf("[BACKUP_1] Thoát bumper -> FIND_WALL2\n");
                    state = FIND_WALL2;
                }
            } break;

            // --------------------------------------------------------
            // FIND_WALL2: So sánh LiDAR trái vs phải
            //   Trái  = cung [80°, 100°]  (gần 90°)
            //   Phải  = cung [260°, 280°] (gần 270°)
            //   Chọn bên có khoảng cách nhỏ hơn -> quay sang đó
            // --------------------------------------------------------
            case FIND_WALL2: {
                float left_d  = lidarAvg(img, 80,  100);   // Tường bên trái
                float right_d = lidarAvg(img, 260, 280);   // Tường bên phải

                printf("[FIND_WALL2] Trái=%.2fm | Phải=%.2fm\n", left_d, right_d);

                if (left_d <= right_d) {
                    // Tường trái gần hơn: quay trái +90°
                    wall2_dir  = +1;
                    target_yaw = normAng(yaw + M_PI / 2.0);
                    printf("[FIND_WALL2] Chọn TRÁI -> quay +90°, target=%.2f rad\n", target_yaw);
                } else {
                    // Tường phải gần hơn: quay phải -90°
                    wall2_dir  = -1;
                    target_yaw = normAng(yaw - M_PI / 2.0);
                    printf("[FIND_WALL2] Chọn PHẢI -> quay -90°, target=%.2f rad\n", target_yaw);
                }
                state = TURN_TO_WALL2;
            } break;

            // --------------------------------------------------------
            // TURN_TO_WALL2: Xoay về hướng tường thứ 2
            // --------------------------------------------------------
            case TURN_TO_WALL2: {
                double err = normAng(target_yaw - yaw);
                co = KP_T * err;
                if (std::abs(err) < TURN_THR) {
                    printf("[TURN_TO_WALL2] Xoay xong -> APPROACH_WALL2\n");
                    state = APPROACH_WALL2;
                }
            } break;

            // --------------------------------------------------------
            // APPROACH_WALL2: Tiến vào góc (tường thứ 2)
            //   Vừa đi vừa kiểm tra tường phía trước < WALL2_STOP
            //   hoặc cả hai phía (front + side bên kia) cho thấy đã vào góc
            // --------------------------------------------------------
            case APPROACH_WALL2: {
                float front = lidarAt(img, 0);
                if (bumped) {
                    printf("[APPROACH_WALL2] Va chạm bumper -> BACKUP_2\n");
                    state = BACKUP_2;
                } else if (front < WALL2_STOP) {
                    printf("[APPROACH_WALL2] Đã vào góc (front=%.2fm) -> CORNER_ALIGN\n", front);
                    state = CORNER_ALIGN;
                } else {
                    cv = WALL2_SPD;
                    co = KP_H * normAng(target_yaw - yaw);
                }
            } break;

            // --------------------------------------------------------
            // BACKUP_2: Lùi ngắn khi bị bumper ở tường 2
            // --------------------------------------------------------
            case BACKUP_2: {
                cv = BACKUP_SPD;
                if (!bumped) {
                    printf("[BACKUP_2] Thoát bumper -> CORNER_ALIGN\n");
                    state = CORNER_ALIGN;
                }
            } break;

            // --------------------------------------------------------
            // CORNER_ALIGN: Điều chỉnh hướng chuẩn ZigZag
            //   Snap yaw về bội số gần nhất của 90° (0, ±π/2, ±π)
            //   Hướng ZigZag = song song với tường vừa tiến vào
            //   (tức là tiếp tục hướng target_yaw hiện tại)
            // --------------------------------------------------------
            case CORNER_ALIGN: {
                // Snap về bội số 90° gần nhất
                double snapped = std::round(yaw / (M_PI / 2.0)) * (M_PI / 2.0);
                zigzag_yaw = snapped;
                double err  = normAng(snapped - yaw);
                co = KP_T * err;
                if (std::abs(err) < TURN_THR) {
                    printf("[CORNER_ALIGN] Hướng ZigZag = %.2f rad (%.0f°) -> DONE\n",
                           zigzag_yaw, zigzag_yaw * 180.0 / M_PI);
                    printf("[CornerFinder] ✓ Robot đã vào góc tường gần nhất. Sẵn sàng ZigZag!\n");
                    state = DONE;
                }
            } break;

            // --------------------------------------------------------
            // DONE: Đứng yên, xuất thông báo một lần
            // --------------------------------------------------------
            case DONE: {
                cv = 0; co = 0;
                static bool printed = false;
                if (!printed) {
                    printf("========================================\n");
                    printf("[DONE] Góc tường đã đến.\n");
                    printf("       Hướng ZigZag chuẩn: %.2f rad (%.0f°)\n",
                           zigzag_yaw, zigzag_yaw * 180.0 / M_PI);
                    printf("       Khoảng cách tường trước: %.2fm\n",
                           lidarAt(img, 0));
                    printf("       Khoảng cách tường bên  : %.2fm\n",
                           lidarAt(img, (wall2_dir > 0) ? 270 : 90));
                    printf("========================================\n");
                    printed = true;
                }
            } break;
        }

        diffDrive(cv, co, mL, mR);
    }

    delete robot;
    return 0;
}