// ============================================================
//  vacuum_controller.cpp
//  Robot Hút Bụi Tự Động - Webots Simulation
//
//  Kiến trúc: FSM 4 pha
//    Phase 1 – Tìm góc tường (QUET_360 → CAN_CHINH_GOC)
//    Phase 2 – Quay 180° chuẩn bị quét  (QUAY_180)
//    Phase 3 – Quét ZigZag              (TIEN … QUAY_2)
//    Phase 4 – Điều hướng BFS + phục hồi (KIEM_TRA_BAN_DO … CAN_LAI_HUONG)
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
using namespace std;

// ============================================================
//  HẰNG SỐ CẤU HÌNH
// ============================================================

// --- Thông số cơ học ---
static constexpr double BANH_XE_BAN_KINH = 0.031; // [m]
static constexpr double BANH_XE_KHOANG = 0.258;   // [m]  khoảng cách tâm 2 bánh
static constexpr double MOTOR_TOC_DO_MAX = 6.28;  // [rad/s]

// --- Tốc độ tịnh tiến theo từng pha ---
static constexpr double V_TIEN_TUONG_1 = 0.20; // [m/s]  tiến đến tường thứ nhất
static constexpr double V_TIEN_TUONG_2 = 0.18; // [m/s]  tiến đến tường thứ hai
static constexpr double V_LUI = -0.10;         // [m/s]  lùi sau va chạm
static constexpr double V_ZIGZAG = 0.25;       // [m/s]  quét zigzag
static constexpr double V_DICH_HANG = 0.18;    // [m/s]  dịch sang hàng mới
static constexpr double V_DIEU_HUONG = 0.30;   // [m/s]  di chuyển theo BFS

// --- Ngưỡng không gian ---
static constexpr double KHOANG_DICH_HANG = 0.30; // [m]  độ rộng mỗi dải quét
static constexpr double NGUONG_VAT_CAN =
    0.20; // [m]  khoảng cách phát hiện vật cản
static constexpr double DUNG_TUONG_1 = 0.15;   // [m]  dừng cách tường 1
static constexpr double DUNG_TUONG_2 = 0.15;   // [m]  dừng cách tường 2
static constexpr double NGUONG_DEN_NOI = 0.08; // [m]  coi là đã đến điểm BFS

// --- Tham số bộ điều khiển ---
static constexpr double KP_GIU_THANG = 4.5; // Hệ số P giữ thẳng hướng
static constexpr double KP_QUAY = 5.5;      // Hệ số P điều khiển góc quay
static constexpr double NGUONG_SAI_SO_GOC =
    0.012; // [rad]  sai số góc chấp nhận được

// --- Hệ số chuyển đổi ---
static constexpr double DO_SANG_RAD = M_PI / 180.0;

// --- Thông số lưới bản đồ ---
static constexpr double O_LUOI_KICH_THUOC = 0.20; // [m/ô]
static constexpr int LUOI_SO_COT = 100;
static constexpr int LUOI_SO_HANG = 100;
static constexpr int LUOI_DICH_COT = 50;  // gốc tọa độ lưới (cột)
static constexpr int LUOI_DICH_HANG = 50; // gốc tọa độ lưới (hàng)

// ============================================================
//  CẤU TRÚC DỮ LIỆU
// ============================================================

// Một ô trong lưới bản đồ
struct OLuoi {
  int8_t trang_thai = 0; // 0 = chưa thăm, 1 = đã thăm, 2 = vật cản
  int8_t diem_tin = 0;   // điểm tích lũy để xác nhận vật cản
};

// Bản đồ lưới toàn cục
static OLuoi ban_do[LUOI_SO_HANG][LUOI_SO_COT];

// Tọa độ một ô lưới (hệ số nguyên)
struct DiemLuoi {
  int cot, hang;
};

// Tất cả các trạng thái FSM
enum TrangThai {
  // Phase 1: Tìm góc tường
  QUET_360,
  QUAY_VE_TUONG_1,
  TIEN_DEN_TUONG_1,
  LUI_1,
  TIM_TUONG_2,
  QUAY_VE_TUONG_2,
  TIEN_DEN_TUONG_2,
  LUI_2,
  CAN_CHINH_GOC,

  // Phase 2: Chuẩn bị zigzag
  QUAY_180,

  // Phase 3: Quét ZigZag
  TIEN,
  LUI,
  QUAY_1,
  DICH_HANG,
  QUAY_2,

  // Phase 4: Điều hướng BFS
  KIEM_TRA_BAN_DO,
  DH_QUAY,
  DH_TIEN,
  DH_LUI,
  CAN_LAI_HUONG,

  HOAN_THANH
};

// Toàn bộ biến trạng thái của robot
struct TrangThaiRobot {
  // Odometry
  double vi_tri_x = 0.0;
  double vi_tri_y = 0.0;
  double enc_trai_cu = 0.0;
  double enc_phai_cu = 0.0;
  double goc_truoc = 0.0;
  bool buoc_dau = true; // true = chưa đọc encoder lần đầu

  // Góc điều hướng
  double goc_muc_tieu = 0.0;   // góc đích hiện tại khi quay
  double huong_hang = 0.0;     // hướng chạy dọc của dải zigzag
  double huong_goc = 0.0;      // hướng gốc ban đầu của zigzag
  double huong_tiep_tuc = 0.0; // hướng sẽ tiếp tục sau BFS
  double goc_tuong_1 = 0.0;    // hướng về phía tường đầu tiên

  // Thông số Phase 1
  int chieu_tuong_2 = 1; // +1 hoặc -1: chiều quay tìm tường 2

  // Thông số zigzag
  double dich_bat_dau_x = 0.0;
  double dich_bat_dau_y = 0.0;
  int chieu_queo = 1; // +1: rẽ trái, -1: rẽ phải
  bool dich_bi_chan = false;

  // BFS và phục hồi
  int luu_cot = 0;
  int luu_hang = 0;
  vector<DiemLuoi> duong_di; // đường đi BFS đến ô chưa thăm
  size_t chi_so = 0;
  bool dang_phuc_hoi = false;
};

// ============================================================
//  HÀM TIỆN ÍCH
// ============================================================

/**
 * Chuẩn hóa góc về khoảng (-π, π].
 */
static inline double chuan_hoa_goc(double a) {
  a = fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0)
    a += 2.0 * M_PI;
  return a - M_PI;
}

/**
 * Điều khiển vi sai: từ vận tốc tịnh tiến (m/s) và vận tốc góc (rad/s)
 * tính tốc độ góc từng bánh rồi gửi xuống motor.
 * Tự động scale nếu vượt giới hạn MOTOR_TOC_DO_MAX.
 */
static void dieu_khien_vi_sai(double v_tien, double v_goc, Motor *dc_trai,
                              Motor *dc_phai) {
  // Công thức vi sai
  double w_trai = (v_tien - v_goc * BANH_XE_KHOANG * 0.5) / BANH_XE_BAN_KINH;
  double w_phai = (v_tien + v_goc * BANH_XE_KHOANG * 0.5) / BANH_XE_BAN_KINH;

  // Scale nếu vượt giới hạn
  double w_max = max(abs(w_trai), abs(w_phai));
  double ty_le = (w_max > MOTOR_TOC_DO_MAX) ? MOTOR_TOC_DO_MAX / w_max : 1.0;

  dc_trai->setVelocity(w_trai * ty_le);
  dc_phai->setVelocity(w_phai * ty_le);
}

/**
 * Lấy giá trị LiDAR tại góc [goc] độ (0–359).
 * Trả về 9.9 nếu dữ liệu không hợp lệ (inf, nan, quá gần).
 */
static inline float lidar_tai(const float *anh, int goc) {
  int vi_tri = ((goc % 360) + 360) % 360;
  float d = anh[vi_tri];
  return (d < 0.01f || isinf(d) || isnan(d)) ? 9.9f : d;
}

/**
 * Tính trung bình các giá trị LiDAR trong phạm vi [goc_dau, goc_cuoi].
 * Bỏ qua các giá trị không hợp lệ (≥ 9.0).
 */
static float lidar_trung_binh(const float *anh, int goc_dau, int goc_cuoi) {
  float tong = 0.0f;
  int dem = 0;
  for (int g = goc_dau; g <= goc_cuoi; g++) {
    float v = lidar_tai(anh, g);
    if (v < 9.0f) {
      tong += v;
      dem++;
    }
  }
  return (dem > 0) ? tong / dem : 9.9f;
}

/**
 * Chuyển tọa độ thế giới (m) → chỉ số ô lưới (cột, hàng).
 */
static void the_gioi_ra_luoi(double x, double y, int &cot, int &hang) {
  cot = (int)floor(x / O_LUOI_KICH_THUOC) + LUOI_DICH_COT;
  hang = (int)floor(y / O_LUOI_KICH_THUOC) + LUOI_DICH_HANG;
}

/**
 * Chuyển chỉ số ô lưới → tọa độ tâm ô trong thế giới (m).
 */
static void luoi_ra_the_gioi(int cot, int hang, double &x, double &y) {
  x = (cot - LUOI_DICH_COT) * O_LUOI_KICH_THUOC + (O_LUOI_KICH_THUOC * 0.5);
  y = (hang - LUOI_DICH_HANG) * O_LUOI_KICH_THUOC + (O_LUOI_KICH_THUOC * 0.5);
}

/**
 * Cập nhật bản đồ lưới từ dữ liệu LiDAR.
 * Bỏ qua khi robot đang quay nhanh (van_toc_goc > 1 rad/s) để tránh lỗi.
 */
static void cap_nhat_ban_do(const float *anh, double rx, double ry,
                            double goc_robot, double van_toc_goc) {
  if (!anh || abs(van_toc_goc) > 1.0)
    return;

  for (int i = 0; i < 360; i += 5) {
    float kc = anh[i];
    if (kc <= 0.05f || kc >= 2.0f)
      continue; // ngoài tầm hữu ích

    // Tọa độ thế giới của điểm phản xạ
    double goc_the_gioi = chuan_hoa_goc(goc_robot - i * DO_SANG_RAD);
    int cot, hang;
    the_gioi_ra_luoi(rx + kc * cos(goc_the_gioi), ry + kc * sin(goc_the_gioi),
                     cot, hang);

    // Kiểm tra trong biên lưới
    if ((unsigned)cot >= (unsigned)LUOI_SO_COT ||
        (unsigned)hang >= (unsigned)LUOI_SO_HANG)
      continue;

    OLuoi &o = ban_do[hang][cot];
    if (o.trang_thai == 2)
      continue; // ô đã là vật cản – bỏ qua

    // Tích lũy điểm tin cậy
    int them = (kc < 1.0f) ? 3 : 1;
    o.diem_tin = (int8_t)min((int)o.diem_tin + them, 100);
    if (o.diem_tin > 5)
      o.trang_thai = 2; // xác nhận là vật cản
  }
}

/**
 * BFS từ ô (cot_dau, hang_dau), tìm ô chưa thăm (trang_thai == 0) gần nhất.
 * Trả về đường đi dạng vector<DiemLuoi> từ điểm xuất phát → điểm đích.
 * Trả về vector rỗng nếu không còn ô nào chưa thăm.
 */
vector<DiemLuoi> tim_o_chua_tham(int cot_dau, int hang_dau) {
  if ((unsigned)cot_dau >= (unsigned)LUOI_SO_COT ||
      (unsigned)hang_dau >= (unsigned)LUOI_SO_HANG)
    return {};

  // Mảng lưu cha (index 1D) để truy vết đường đi
  static int cha[LUOI_SO_HANG * LUOI_SO_COT];
  fill(cha, cha + LUOI_SO_HANG * LUOI_SO_COT, -1);

  auto chi_so_1d = [](int c, int h) { return h * LUOI_SO_COT + c; };

  int bat_dau = chi_so_1d(cot_dau, hang_dau);
  cha[bat_dau] = bat_dau; // điểm gốc tự trỏ về mình

  queue<int> hang_doi;
  hang_doi.push(bat_dau);

  // 4 hướng di chuyển (lên, xuống, trái, phải)
  const int dc[] = {0, 0, -1, 1};
  const int dh[] = {-1, 1, 0, 0};

  int muc_tieu = -1;

  while (!hang_doi.empty() && muc_tieu < 0) {
    int hien = hang_doi.front();
    hang_doi.pop();

    int cc = hien % LUOI_SO_COT;
    int ch = hien / LUOI_SO_COT;

    // Tìm thấy ô chưa thăm
    if (ban_do[ch][cc].trang_thai == 0) {
      muc_tieu = hien;
      break;
    }

    // Mở rộng BFS sang 4 ô lân cận
    for (int i = 0; i < 4; i++) {
      int nc = cc + dc[i];
      int nh = ch + dh[i];

      if ((unsigned)nc >= (unsigned)LUOI_SO_COT ||
          (unsigned)nh >= (unsigned)LUOI_SO_HANG)
        continue;

      int cs = chi_so_1d(nc, nh);
      if (cha[cs] >= 0 || ban_do[nh][nc].trang_thai == 2)
        continue;

      cha[cs] = hien;
      hang_doi.push(cs);
    }
  }

  if (muc_tieu < 0)
    return {}; // không tìm thấy ô nào

  // Truy vết đường đi từ đích về gốc, rồi đảo ngược
  vector<DiemLuoi> duong;
  for (int hien = muc_tieu; hien != bat_dau; hien = cha[hien])
    duong.push_back({hien % LUOI_SO_COT, hien / LUOI_SO_COT});
  duong.push_back({cot_dau, hang_dau});
  reverse(duong.begin(), duong.end());
  return duong;
}

/**
 * Đếm số ô chưa có vật cản (trang_thai != 2) theo một hướng nhất định.
 * Dùng để chọn hướng zigzag tối ưu sau khi BFS.
 */
static int dem_o_trong_theo_huong(int cot, int hang, double huong) {
  int dc = (int)round(cos(huong));
  int dh = (int)round(sin(huong));
  int so_o = 0;
  int cc = cot, ch = hang;

  while (true) {
    cc += dc;
    ch += dh;
    if ((unsigned)cc >= (unsigned)LUOI_SO_COT ||
        (unsigned)ch >= (unsigned)LUOI_SO_HANG ||
        ban_do[ch][cc].trang_thai == 2)
      break;
    if (ban_do[ch][cc].trang_thai == 0)
      so_o++;
  }
  return so_o;
}

// ============================================================
//  HÀM MAIN
// ============================================================
int main() {
  Robot *robot = new Robot();
  int buoc_ms = (int)robot->getBasicTimeStep(); // [ms]

  // --- Khởi tạo thiết bị ---
  Motor *dc_trai = robot->getMotor("left wheel motor");
  Motor *dc_phai = robot->getMotor("right wheel motor");
  dc_trai->setPosition(INFINITY); // chế độ điều khiển vận tốc
  dc_phai->setPosition(INFINITY);

  PositionSensor *enc_trai = robot->getPositionSensor("left wheel sensor");
  PositionSensor *enc_phai = robot->getPositionSensor("right wheel sensor");
  enc_trai->enable(buoc_ms);
  enc_phai->enable(buoc_ms);

  Lidar *lidar = robot->getLidar("LDS-01");
  InertialUnit *imu = robot->getInertialUnit("inertial unit");
  lidar->enable(buoc_ms);
  imu->enable(buoc_ms);

  TouchSensor *bumper_trai = robot->getTouchSensor("bumper_left");
  TouchSensor *bumper_phai = robot->getTouchSensor("bumper_right");
  if (bumper_trai)
    bumper_trai->enable(buoc_ms);
  if (bumper_phai)
    bumper_phai->enable(buoc_ms);

  // --- Khởi tạo FSM ---
  TrangThai trang_thai = QUET_360;
  TrangThaiRobot rs;
  printf("[Hệ Thống] Khởi động – Phase 1: Tìm góc tường...\n");

  // ============================================================
  //  VÒNG LẶP ĐIỀU KHIỂN CHÍNH
  // ============================================================
  while (robot->step(buoc_ms) != -1) {

    // --- Đọc cảm biến ---
    const double yaw = imu->getRollPitchYaw()[2]; // [rad]
    const double dt = buoc_ms / 1000.0;           // [s]

    // Tính vận tốc góc (dùng để lọc bản đồ khi quay nhanh)
    if (rs.buoc_dau) {
      rs.goc_truoc = yaw;
      rs.buoc_dau = false;
    }
    const double van_toc_goc = chuan_hoa_goc(yaw - rs.goc_truoc) / dt;
    rs.goc_truoc = yaw;

    const float *anh_lidar = lidar->getRangeImage();
    if (!anh_lidar)
      continue;

    // --- Odometry vi sai ---
    const double enc_L = enc_trai->getValue();
    const double enc_R = enc_phai->getValue();
    const double delta = ((enc_L - rs.enc_trai_cu) + (enc_R - rs.enc_phai_cu)) *
                         0.5 * BANH_XE_BAN_KINH;
    rs.vi_tri_x += delta * cos(yaw);
    rs.vi_tri_y += delta * sin(yaw);
    rs.enc_trai_cu = enc_L;
    rs.enc_phai_cu = enc_R;

    // --- Cập nhật bản đồ ---
    int gc, gr;
    the_gioi_ra_luoi(rs.vi_tri_x, rs.vi_tri_y, gc, gr);

    // Đánh dấu ô robot đứng và 8 ô xung quanh là "đã thăm"
    for (int di = -1; di <= 1; di++) {
      for (int dj = -1; dj <= 1; dj++) {
        int hc = gc + di, hr = gr + dj;
        if ((unsigned)hc < (unsigned)LUOI_SO_COT &&
            (unsigned)hr < (unsigned)LUOI_SO_HANG &&
            ban_do[hr][hc].trang_thai != 2)
          ban_do[hr][hc].trang_thai = 1;
      }
    }
    cap_nhat_ban_do(anh_lidar, rs.vi_tri_x, rs.vi_tri_y, yaw, van_toc_goc);

    // Phát hiện va chạm qua bumper
    const bool va_cham = (bumper_trai && bumper_trai->getValue() > 0.5) ||
                         (bumper_phai && bumper_phai->getValue() > 0.5);

    // Đầu ra điều khiển (reset mỗi bước)
    double cv = 0.0; // vận tốc tịnh tiến [m/s]
    double co = 0.0; // vận tốc góc       [rad/s]

    // ============================================================
    //  FSM – XỬ LÝ TỪNG TRẠNG THÁI
    // ============================================================
    switch (trang_thai) {

    // ------------------------------------------------------------
    //  PHASE 1: TÌM GÓC TƯỜNG
    // ------------------------------------------------------------

    // Quét toàn bộ 360° LiDAR, tìm hướng về tường gần nhất
    case QUET_360: {
      float kc_nho_nhat = 99.0f;
      int goc_nho_nhat = 0;
      for (int i = 0; i < 360; i++) {
        float d = lidar_tai(anh_lidar, i);
        if (d < kc_nho_nhat) {
          kc_nho_nhat = d;
          goc_nho_nhat = i;
        }
      }
      rs.goc_muc_tieu = rs.goc_tuong_1 =
          chuan_hoa_goc(yaw - goc_nho_nhat * DO_SANG_RAD);
      trang_thai = QUAY_VE_TUONG_1;
    } break;

    // Quay về hướng tường thứ nhất
    case QUAY_VE_TUONG_1:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC)
        trang_thai = TIEN_DEN_TUONG_1;
      break;

    // Tiến thẳng đến tường thứ nhất
    case TIEN_DEN_TUONG_1:
      if (va_cham) {
        trang_thai = LUI_1;
      } else if (lidar_tai(anh_lidar, 0) < DUNG_TUONG_1) {
        trang_thai = TIM_TUONG_2;
      } else {
        cv = V_TIEN_TUONG_1;
        co = KP_GIU_THANG * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      }
      break;

    // Lùi ra sau khi va chạm tường 1
    case LUI_1:
      cv = V_LUI;
      if (!va_cham)
        trang_thai = TIM_TUONG_2;
      break;

    // Xác định chiều quay để tìm tường thứ hai (vuông góc)
    case TIM_TUONG_2: {
      float kc_phai = lidar_trung_binh(anh_lidar, 80, 100);
      float kc_trai = lidar_trung_binh(anh_lidar, 260, 280);
      rs.chieu_tuong_2 = (kc_phai <= kc_trai) ? +1 : -1;
      rs.goc_muc_tieu = chuan_hoa_goc(yaw + rs.chieu_tuong_2 * M_PI * 0.5);
      trang_thai = QUAY_VE_TUONG_2;
    } break;

    // Quay về hướng tường thứ hai
    case QUAY_VE_TUONG_2:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC)
        trang_thai = TIEN_DEN_TUONG_2;
      break;

    // Tiến đến tường thứ hai
    case TIEN_DEN_TUONG_2:
      if (va_cham) {
        trang_thai = LUI_2;
      } else if (lidar_tai(anh_lidar, 0) < DUNG_TUONG_2) {
        trang_thai = CAN_CHINH_GOC;
      } else {
        cv = V_TIEN_TUONG_2;
        co = KP_GIU_THANG * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      }
      break;

    // Lùi ra sau khi va chạm tường 2
    case LUI_2:
      cv = V_LUI;
      if (!va_cham)
        trang_thai = CAN_CHINH_GOC;
      break;

    // Căn chỉnh góc robot về bội số 90° gần nhất
    case CAN_CHINH_GOC: {
      double goc_gan_nhat = round(yaw / (M_PI * 0.5)) * (M_PI * 0.5);
      double sai_so = chuan_hoa_goc(goc_gan_nhat - yaw);
      co = KP_QUAY * sai_so;
      if (abs(sai_so) < NGUONG_SAI_SO_GOC) {
        // Lưu hướng hàng zigzag (ngược lại 180°)
        rs.goc_muc_tieu = rs.huong_hang = rs.huong_goc =
            chuan_hoa_goc(yaw + sai_so + M_PI);
        rs.chieu_queo = -rs.chieu_tuong_2;
        rs.dich_bi_chan = false;
        printf("[Phase 1 → 2] Căn góc xong. Quay 180°...\n");
        trang_thai = QUAY_180;
      }
    } break;

    // ------------------------------------------------------------
    //  PHASE 2: QUAY 180° ĐỂ CHUẨN BỊ QUÉT
    // ------------------------------------------------------------
    case QUAY_180:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        printf("[Phase 2 → 3] Bắt đầu quét ZigZag!\n");
        trang_thai = TIEN;
      }
      break;

    // ------------------------------------------------------------
    //  PHASE 3: QUÉT ZIGZAG
    // ------------------------------------------------------------

    // Tiến thẳng theo hàng hiện tại
    case TIEN: {
      cv = V_ZIGZAG;
      co = KP_GIU_THANG * chuan_hoa_goc(rs.huong_hang - yaw);

      if (va_cham) {
        trang_thai = LUI;
        break;
      }

      bool gap_vat_can = (lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN);
      bool het_hang = rs.dang_phuc_hoi &&
                      dem_o_trong_theo_huong(gc, gr, rs.huong_hang) == 0;

      if (gap_vat_can || het_hang) {
        // Kiểm tra còn ô nào chưa thăm không
        bool xong_het = true;
        for (int i = 0; i < LUOI_SO_HANG && xong_het; i++)
          for (int j = 0; j < LUOI_SO_COT && xong_het; j++)
            if (ban_do[i][j].trang_thai == 0)
              xong_het = false;

        if (xong_het) {
          trang_thai = HOAN_THANH;
        } else {
          rs.goc_muc_tieu =
              chuan_hoa_goc(rs.huong_hang + rs.chieu_queo * M_PI * 0.5);
          rs.dich_bi_chan = false;
          trang_thai = QUAY_1;
        }
      }
    } break;

    // Lùi ra sau khi va chạm ở phase zigzag
    case LUI:
      cv = V_LUI;
      co = KP_GIU_THANG * chuan_hoa_goc(rs.huong_hang - yaw);
      if (!va_cham) {
        rs.goc_muc_tieu =
            chuan_hoa_goc(rs.huong_hang + rs.chieu_queo * M_PI * 0.5);
        rs.dich_bi_chan = false;
        trang_thai = QUAY_1;
      }
      break;

    // Quay 90° sang hướng hàng kế tiếp
    case QUAY_1:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        if (lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN) {
          // Bị chặn ngay khi quay xong → chuyển BFS
          rs.dich_bi_chan = true;
          rs.huong_tiep_tuc = rs.huong_hang;
          rs.luu_cot = gc;
          rs.luu_hang = gr;
          trang_thai = KIEM_TRA_BAN_DO;
        } else {
          // Bình thường → dịch sang hàng mới
          rs.dich_bat_dau_x = rs.vi_tri_x;
          rs.dich_bat_dau_y = rs.vi_tri_y;
          trang_thai = DICH_HANG;
        }
      }
      break;

    // Dịch chuyển ngang KHOANG_DICH_HANG để sang hàng kế tiếp
    case DICH_HANG: {
      cv = V_DICH_HANG;
      co = KP_GIU_THANG * chuan_hoa_goc(rs.goc_muc_tieu - yaw);

      double khoang_da_di = hypot(rs.vi_tri_x - rs.dich_bat_dau_x,
                                  rs.vi_tri_y - rs.dich_bat_dau_y);
      bool bi_chan = lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN || va_cham;

      if (bi_chan && khoang_da_di < KHOANG_DICH_HANG * 0.5) {
        // Chưa dịch được nửa đoạn mà đã bị chặn → BFS
        rs.dich_bi_chan = true;
        rs.huong_tiep_tuc = rs.huong_hang;
        rs.luu_cot = gc;
        rs.luu_hang = gr;
        trang_thai = KIEM_TRA_BAN_DO;
      } else if (khoang_da_di >= KHOANG_DICH_HANG || bi_chan) {
        // Đủ khoảng dịch (hoặc bị chặn sau nửa đoạn) → quay về hàng
        rs.goc_muc_tieu =
            chuan_hoa_goc(rs.goc_muc_tieu + rs.chieu_queo * M_PI * 0.5);
        trang_thai = QUAY_2;
      }
    } break;

    // Quay 90° lần 2 để quay lại hướng chạy hàng
    case QUAY_2:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        rs.huong_hang = rs.goc_muc_tieu;
        rs.chieu_queo *= -1; // đổi chiều queo để hình thành chữ S
        if (lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN) {
          rs.huong_tiep_tuc = rs.huong_hang;
          rs.luu_cot = gc;
          rs.luu_hang = gr;
          trang_thai = KIEM_TRA_BAN_DO;
        } else {
          trang_thai = TIEN;
        }
      }
      break;

    // ------------------------------------------------------------
    //  PHASE 4: ĐIỀU HƯỚNG BFS + PHỤC HỒI
    // ------------------------------------------------------------

    // Chạy BFS tìm ô chưa thăm gần nhất
    case KIEM_TRA_BAN_DO:
      cv = co = 0;
      rs.duong_di = tim_o_chua_tham(rs.luu_cot, rs.luu_hang);
      if (rs.duong_di.empty()) {
        trang_thai = HOAN_THANH;
      } else {
        rs.chi_so = 1;
        trang_thai =
            (rs.chi_so >= rs.duong_di.size()) ? CAN_LAI_HUONG : DH_QUAY;
      }
      break;

    // Quay về hướng điểm BFS tiếp theo
    case DH_QUAY: {
      double tx, ty;
      luoi_ra_the_gioi(rs.duong_di[rs.chi_so].cot, rs.duong_di[rs.chi_so].hang,
                       tx, ty);
      double goc_den = atan2(ty - rs.vi_tri_y, tx - rs.vi_tri_x);
      co = KP_QUAY * chuan_hoa_goc(goc_den - yaw);
      if (abs(chuan_hoa_goc(goc_den - yaw)) < NGUONG_SAI_SO_GOC)
        trang_thai = DH_TIEN;
    } break;

    // Tiến đến điểm BFS tiếp theo
    case DH_TIEN: {
      double tx, ty;
      luoi_ra_the_gioi(rs.duong_di[rs.chi_so].cot, rs.duong_di[rs.chi_so].hang,
                       tx, ty);
      double goc_den = atan2(ty - rs.vi_tri_y, tx - rs.vi_tri_x);
      cv = V_DIEU_HUONG;
      co = KP_GIU_THANG * chuan_hoa_goc(goc_den - yaw);

      if (va_cham || lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN) {
        trang_thai = DH_LUI;
        break;
      }

      if (hypot(tx - rs.vi_tri_x, ty - rs.vi_tri_y) < NGUONG_DEN_NOI) {
        if (++rs.chi_so >= rs.duong_di.size()) {
          // Đã đến đích BFS – chọn hướng zigzag tốt nhất
          rs.dang_phuc_hoi = true;
          double huong_tot_nhat = rs.huong_goc;
          int so_o_nhieu_nhat = -1;
          for (int i = 0; i < 4; i++) {
            double h = rs.huong_goc + i * M_PI * 0.5;
            int so_o = dem_o_trong_theo_huong(gc, gr, h);
            if (so_o > so_o_nhieu_nhat) {
              so_o_nhieu_nhat = so_o;
              huong_tot_nhat = h;
            }
          }
          rs.huong_tiep_tuc = chuan_hoa_goc(huong_tot_nhat);
          // Chọn chiều queo: bên nào còn nhiều ô hơn
          int so_trai = dem_o_trong_theo_huong(
              gc, gr, chuan_hoa_goc(huong_tot_nhat + M_PI * 0.5));
          int so_phai = dem_o_trong_theo_huong(
              gc, gr, chuan_hoa_goc(huong_tot_nhat - M_PI * 0.5));
          rs.chieu_queo = (so_trai >= so_phai) ? 1 : -1;
          trang_thai = CAN_LAI_HUONG;
        } else {
          trang_thai = DH_QUAY; // tiếp tục đến điểm kế
        }
      }
    } break;

    // Lùi khi gặp vật cản trong lúc BFS
    case DH_LUI:
      cv = V_LUI;
      if (!va_cham) {
        rs.luu_cot = gc;
        rs.luu_hang = gr;
        trang_thai = KIEM_TRA_BAN_DO;
      }
      break;

    // Căn lại hướng zigzag rồi tiếp tục quét
    case CAN_LAI_HUONG:
      cv = 0;
      rs.goc_muc_tieu = rs.huong_tiep_tuc;
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        rs.huong_hang = rs.huong_tiep_tuc;
        trang_thai = TIEN;
      }
      break;

    // ------------------------------------------------------------
    //  HOÀN THÀNH
    // ------------------------------------------------------------
    case HOAN_THANH:
      cv = co = 0;
      static bool da_in = false;
      if (!da_in) {
        printf("✓ Hoàn thành! Toàn bộ diện tích đã được phủ.\n");
        da_in = true;
      }
      break;

    } // end switch

    // Gửi lệnh điều khiển xuống motor
    dieu_khien_vi_sai(cv, co, dc_trai, dc_phai);

  } // end while

  delete robot;
  return 0;
}