// ============================================================
//  Robot Hút Bụi Tự Động - Webots Simulation
//  Kiến trúc: FSM 4 pha
//    Phase 1 – Tìm góc tường
//    Phase 2 – Quay 180° chuẩn bị quét
//    Phase 3 – Quét ZigZag
//    Phase 4 – Điều hướng BFS + phục hồi
// ============================================================

#include <webots/Display.hpp>
#include <webots/GPS.hpp>
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
#include <cstring>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

using namespace webots;
using namespace std;

// ============================================================
//  HẰNG SỐ CẤU HÌNH
// ============================================================

// --- Thông số cơ học ---
static constexpr double BANH_XE_BAN_KINH = 0.031; // [m]
static constexpr double BANH_XE_KHOANG = 0.258;   // [m]
static constexpr double MOTOR_TOC_DO_MAX = 6.28;  // [rad/s]
static constexpr double ROBOT_BAN_KINH = 0.17; // [m]  bán kính robot (d=0.34m)

// --- Tốc độ tịnh tiến theo từng pha ---
static constexpr double V_TIEN_TUONG_1 = 0.15; // giảm để dừng an toàn
static constexpr double V_TIEN_TUONG_2 = 0.12; // giảm để dừng an toàn
static constexpr double V_LUI = -0.10;         // [m/s]
static constexpr double V_ZIGZAG = 0.25;       // giảm: robot rộng hơn ô lưới
static constexpr double V_DICH_HANG = 0.15;
static constexpr double V_DIEU_HUONG = 0.20; // giảm để phản ứng kịp

// --- Ngưỡng không gian ---
// Ghi chú: LiDAR đo từ tâm robot.
//   Khoảng cách an toàn tối thiểu = ROBOT_BAN_KINH + biên dự phòng
static constexpr double KHOANG_DICH_HANG = 1.0 / 3.0; // [m]
static constexpr double VUNG_AN_TOAN = 0.6;           // [m]
static constexpr double VUNG_CANH_BAO =
    0.40; // [m]  tăng: bắt đầu phanh sớm hơn
static constexpr double VUNG_NGUY_HIEM = 0.25; // [m]  = ROBOT_BAN_KINH + 0.08m
static constexpr double NGUONG_VAT_CAN = 0.25; // [m]  = ROBOT_BAN_KINH + 0.08m
static constexpr double DUNG_TUONG_1 = 0.27;   // [m]  = ROBOT_BAN_KINH + 0.10m
static constexpr double DUNG_TUONG_2 = 0.22;   // [m]  = ROBOT_BAN_KINH + 0.05m
static constexpr double NGUONG_DEN_NOI = 0.06; // [m]  tăng: tránh BFS overshoot

// --- Tham số bộ điều khiển ---
static constexpr double KP_GIU_THANG = 4.5; // Hệ số P giữ thẳng hướng
static constexpr double KP_QUAY = 5.5;      // Hệ số P điều khiển góc quay
static constexpr double NGUONG_SAI_SO_GOC =
    0.012; // [rad]  sai số góc chấp nhận được
// --- Hệ số chuyển đổi ---
static constexpr double DO_SANG_RAD = M_PI / 180.0;

static constexpr double O_LUOI_KICH_THUOC = 1.0 / 3.0;
static constexpr int LUOI_SO_COT = 15;
static constexpr int LUOI_SO_HANG = 15;
static constexpr int LUOI_DICH_COT = 7;
static constexpr int LUOI_DICH_HANG = 7;

// Một ô trong lưới bản đồ
struct OLuoi {
  int8_t trang_thai = 0; // Trạng thái của ô
  int8_t diem_tin = 0;   // Điểm tích lũy của ô
};

// Bản đồ lưới toàn cục
static OLuoi ban_do[LUOI_SO_HANG][LUOI_SO_COT];

// ============================================================
//  ANSI DASHBOARD
// ============================================================
#define ANSI_SAVE "\033[s"
#define ANSI_RESTORE "\033[u"
#define ANSI_CLEAR_EOL "\033[K"
#define ANSI_UP(n) "\033[" #n "A"
#define ANSI_BOLD "\033[1m"
#define ANSI_RESET "\033[0m"
#define ANSI_CYAN "\033[36m"
#define ANSI_GREEN "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_RED "\033[31m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_WHITE "\033[37m"

static constexpr int DASHBOARD_DONG_TRANG_THAI = 7;
static constexpr int DASHBOARD_TONG_DONG =
    DASHBOARD_DONG_TRANG_THAI + LUOI_SO_HANG + 3;

static int g_buoc = 0;
static int g_so_va_cham = 0;
static bool g_bumper_cu = false;
static bool g_dashboard_da_ve = false;

static const char *ten_trang_thai(int tt) {
  switch (tt) {
  case 0:
    return "QUET_360";
  case 1:
    return "QUAY_VE_TUONG_1";
  case 2:
    return "TIEN_DEN_TUONG_1";
  case 3:
    return "LUI_1";
  case 4:
    return "TIM_TUONG_2";
  case 5:
    return "QUAY_VE_TUONG_2";
  case 6:
    return "TIEN_DEN_TUONG_2";
  case 7:
    return "LUI_2";
  case 8:
    return "CAN_CHINH_GOC";
  case 9:
    return "QUAY_180";
  case 10:
    return "TIEN (ZigZag)";
  case 11:
    return "LUI (ZigZag)";
  case 12:
    return "QUAY_1";
  case 13:
    return "DICH_HANG";
  case 14:
    return "QUAY_2";
  case 15:
    return "DUNG_CHO_BFS";
  case 16:
    return "KIEM_TRA_BAN_DO";
  case 17:
    return "DH_QUAY";
  case 18:
    return "DH_TIEN";
  case 19:
    return "DH_LUI";
  case 20:
    return "CAN_LAI_HUONG";
  case 21:
    return "HOAN_THANH";
  default:
    return "UNKNOWN";
  }
}

static inline float lidar_tai(const float *anh, int goc);
static float lidar_trung_binh(const float *anh, int goc_dau, int goc_cuoi);

static inline void di_chuyen_len_dau_dashboard() {
  cout << "\033[" << DASHBOARD_TONG_DONG << "A\r";
}

static inline void dong_dashboard(const string &noi_dung) {
  cout << noi_dung << ANSI_CLEAR_EOL << "\n";
}

static void ve_dashboard(int trang_thai_fsm, double x, double y, double yaw,
                         int gc, int gr, const float *anh_lidar,
                         const OLuoi ban_do[][LUOI_SO_COT]) {
  if (!g_dashboard_da_ve) {
    for (int i = 0; i < DASHBOARD_TONG_DONG; i++)
      cout << "\n";
    g_dashboard_da_ve = true;
  }
  di_chuyen_len_dau_dashboard();

  {
    ostringstream s;
    s << ANSI_BOLD << ANSI_RED << "══ ROBOT HUT BUI │ Buoc:" << setw(6)
      << g_buoc << " ══" << ANSI_RESET;
    dong_dashboard(s.str());
  }

  {
    const char *mau = ANSI_WHITE;
    if (trang_thai_fsm <= 8)
      mau = ANSI_YELLOW;
    else if (trang_thai_fsm == 9)
      mau = ANSI_MAGENTA;
    else if (trang_thai_fsm <= 14)
      mau = ANSI_GREEN;
    else if (trang_thai_fsm <= 19)
      mau = ANSI_CYAN;

    ostringstream s;
    s << "  FSM     : " << mau << ANSI_BOLD << left << setw(20)
      << ten_trang_thai(trang_thai_fsm) << ANSI_RESET;
    dong_dashboard(s.str());
  }

  {
    ostringstream s;
    s << "  Vị trí  : X=" << ANSI_GREEN << fixed << setprecision(4) << setw(9)
      << x << ANSI_RESET << "  Y=" << ANSI_GREEN << setw(9) << y << ANSI_RESET
      << " m";
    dong_dashboard(s.str());
  }

  {
    ostringstream s;
    s << "  Yaw: " << ANSI_RED << fixed << setprecision(4) << setw(9)
      << yaw * 180.0 / M_PI << ANSI_RESET << " deg"
      << "   Ô lưới: (" << gc << ", " << gr << ")";
    dong_dashboard(s.str());
  }

  {
    string vung_str = ANSI_WHITE + string("Không rõ");
    if (anh_lidar) {
      float kc_truoc = lidar_tai(anh_lidar, 0);
      if (kc_truoc > VUNG_AN_TOAN)
        vung_str = ANSI_GREEN + string("An toàn");
      else if (kc_truoc > VUNG_CANH_BAO)
        vung_str = ANSI_YELLOW + string("Cảnh báo");
      else
        vung_str = ANSI_RED + string("Nguy hiểm");
    }

    ostringstream s;
    s << "  Va chạm : " << ANSI_RED << ANSI_BOLD << setw(4) << g_so_va_cham
      << ANSI_RESET << " lần"
      << "   Vùng: " << vung_str << ANSI_RESET;
    dong_dashboard(s.str());
  }

  dong_dashboard("  ─────────────────────────────────────────");
  {
    int da_tham = 0, chua_tham = 0, vat_can = 0;
    for (int h = 0; h < LUOI_SO_HANG; h++)
      for (int c = 0; c < LUOI_SO_COT; c++) {
        switch (ban_do[h][c].trang_thai) {
        case 0:
          chua_tham++;
          break;
        case 1:
          da_tham++;
          break;
        case 2:
          vat_can++;
          break;
        }
      }
    int tong = LUOI_SO_HANG * LUOI_SO_COT;
    double pct = 100.0 * da_tham / tong;

    int filled = (int)(pct / 5.0);
    string thanh = "[" + string(filled, '#') + string(20 - filled, '.') + "]";

    {
      ostringstream s;
      s << "  Bản đồ : " << ANSI_GREEN << thanh << ANSI_RESET << " "
        << ANSI_BOLD << fixed << setprecision(1) << pct << "%" << ANSI_RESET
        << "  (" << da_tham << "/" << tong << " Ô đi được)";
      dong_dashboard(s.str());
    }

    {
      ostringstream s;
      s << ANSI_GREEN << ANSI_BOLD << "[1] Đã thăm: " << setw(4) << da_tham
        << ANSI_RESET << "   " << ANSI_WHITE << ANSI_BOLD
        << "[0] Chưa thăm: " << setw(4) << chua_tham << ANSI_RESET << "   "
        << ANSI_RED << ANSI_BOLD << "[2] Vật cản: " << setw(4) << vat_can
        << ANSI_RESET;
      dong_dashboard(s.str());
    }
  }

  {
    ostringstream s;
    s << "  ";
    for (int c = 0; c < LUOI_SO_COT; c++)
      s << (c >= 10 ? (c / 10) : 0) << " ";
    dong_dashboard(s.str());
  }

  {
    ostringstream s;
    s << "  ";
    for (int c = 0; c < LUOI_SO_COT; c++)
      s << (c % 10) << " ";
    dong_dashboard(s.str());
  }

  for (int h = LUOI_SO_HANG - 1; h >= 0; h--) {
    ostringstream s;
    s << "  ";
    for (int c = 0; c < LUOI_SO_COT; c++) {
      if (c == gc && h == gr) {
        s << ANSI_BOLD << ANSI_MAGENTA << "R " << ANSI_RESET;
        continue;
      }
      switch (ban_do[h][c].trang_thai) {
      case 0:
        s << "0 ";
        break;
      case 1:
        s << ANSI_GREEN << "1 " << ANSI_RESET;
        break;
      case 2:
        s << ANSI_RED << "2 " << ANSI_RESET;
        break;
      default:
        s << "? ";
        break;
      }
    }
    s << " |" << setw(2) << h;
    dong_dashboard(s.str());
  }

  dong_dashboard("  ─────────────────────────────────────────");

  cout.flush();
}

static inline float lidar_tai(const float *anh, int goc) {
  int vi_tri = ((goc % 360) + 360) % 360;
  float d = anh[vi_tri];
  return (d < 0.01f || isinf(d) || isnan(d)) ? 9.9f : d;
}

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
  DUNG_CHO_BFS,
  KIEM_TRA_BAN_DO,
  DH_QUAY,
  DH_TIEN,
  DH_LUI,
  CAN_LAI_HUONG,

  HOAN_THANH
};

struct TrangThaiRobot {
  double vi_tri_x = 0.0;
  double vi_tri_y = 0.0;
  double enc_trai_cu = 0.0;
  double enc_phai_cu = 0.0;
  double goc_truoc = 0.0;
  bool buoc_dau = true;

  double goc_muc_tieu = 0.0;   // góc đích hiện tại khi quay
  double huong_hang = 0.0;     // hướng chạy dọc của dải zigzag
  double huong_goc = 0.0;      // hướng gốc ban đầu của zigzag
  double huong_tiep_tuc = 0.0; // hướng sẽ tiếp tục sau BFS
  double goc_tuong_1 = 0.0;    // hướng về phía tường đầu tiên

  int chieu_tuong_2 = 1; // +1 hoặc -1: chiều quay tìm tường 2

  double dich_bat_dau_x = 0.0;
  double dich_bat_dau_y = 0.0;
  int chieu_queo = 1; // +1: rẽ trái, -1: rẽ phải
  bool dich_bi_chan = false;

  int luu_cot = 0;
  int luu_hang = 0;
  vector<DiemLuoi> duong_di;
  size_t chi_so = 0;
  bool dang_phuc_hoi = false;
  int dem_cho_bfs = 0;
};

// ============================================================
//  HÀM TIỆN ÍCH
// ============================================================

// Chuẩn hóa góc về khoảng (-π, π].
static inline double chuan_hoa_goc(double a) {
  a = fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0)
    a += 2.0 * M_PI;
  return a - M_PI;
}

static void dieu_khien_vi_sai(double v_tien, double v_goc, Motor *dc_trai,
                              Motor *dc_phai) {
  double w_trai = (v_tien - v_goc * BANH_XE_KHOANG * 0.5) / BANH_XE_BAN_KINH;
  double w_phai = (v_tien + v_goc * BANH_XE_KHOANG * 0.5) / BANH_XE_BAN_KINH;
  double w_max = max(abs(w_trai), abs(w_phai));
  double ty_le = (w_max > MOTOR_TOC_DO_MAX) ? MOTOR_TOC_DO_MAX / w_max : 1.0;
  dc_trai->setVelocity(w_trai * ty_le);
  dc_phai->setVelocity(w_phai * ty_le);
}

static void the_gioi_ra_luoi(double x, double y, int &cot, int &hang) {
  cot = (int)floor((x + O_LUOI_KICH_THUOC * 0.5) / O_LUOI_KICH_THUOC) +
        LUOI_DICH_COT;
  hang = (int)floor((y + O_LUOI_KICH_THUOC * 0.5) / O_LUOI_KICH_THUOC) +
         LUOI_DICH_HANG;
}

static void luoi_ra_the_gioi(int cot, int hang, double &x, double &y) {
  x = (cot - LUOI_DICH_COT) * O_LUOI_KICH_THUOC;
  y = (hang - LUOI_DICH_HANG) * O_LUOI_KICH_THUOC;
}

// Cập nhật Display để vẽ bản đồ
static void ve_hien_thi_ban_do(Display *display) {
  if (!display)
    return;
  int w = display->getWidth();
  int h = display->getHeight();

  // Bản đồ lưới logic là 15x15 ô, mỗi ô 1/3m => kích thước đúng 5x5m, tâm ở 0,0
  double floor_x = 5.0;
  double floor_y = 5.0;
  double center_x = 0.0;
  double center_y = 0.0;

  double start_x = center_x - floor_x / 2.0;
  double start_y = center_y - floor_y / 2.0;

  double scale_x = w / floor_x;
  double scale_y = h / floor_y;

  display->setAlpha(0.0);
  display->setColor(0xFFFFFF);
  display->fillRectangle(0, 0, w, h);
  display->setAlpha(1.0);
  display->setColor(0x000000);
  for (int hang = 0; hang <= LUOI_SO_HANG; hang++) {
    double cell_top =
        (hang - LUOI_DICH_HANG) * O_LUOI_KICH_THUOC - (O_LUOI_KICH_THUOC * 0.5);
    int py = (int)((cell_top - start_y) * scale_y);
    display->drawLine(0, py, w, py);
  }
  for (int cot = 0; cot <= LUOI_SO_COT; cot++) {
    double cell_left =
        (cot - LUOI_DICH_COT) * O_LUOI_KICH_THUOC - (O_LUOI_KICH_THUOC * 0.5);
    int px = (int)((cell_left - start_x) * scale_x);
    display->drawLine(px, 0, px, h);
  }
}

static void cap_nhat_ban_do(const float *anh, double rx, double ry,
                            double goc_robot, double van_toc_goc) {
  if (!anh || abs(van_toc_goc) > 1.0)
    return;

  for (int i = 0; i < 360; i += 5) {
    float kc = anh[i];
    if (kc <= 0.05f || kc >= 1.0f)
      continue;

    double goc_the_gioi = chuan_hoa_goc(goc_robot - i * DO_SANG_RAD);
    int cot, hang;
    the_gioi_ra_luoi(rx + kc * cos(goc_the_gioi), ry + kc * sin(goc_the_gioi),
                     cot, hang);

    if ((unsigned)cot >= (unsigned)LUOI_SO_COT ||
        (unsigned)hang >= (unsigned)LUOI_SO_HANG)
      continue;

    if (cot == 0 || hang == 0 || cot == LUOI_SO_COT - 1 ||
        hang == LUOI_SO_HANG - 1)
      continue;

    OLuoi &o = ban_do[hang][cot];
    if (o.trang_thai == 2 || o.trang_thai == 1)
      continue;

    int them = (kc < 0.5f) ? 2 : 1;
    o.diem_tin = (int8_t)min((int)o.diem_tin + them, 150);
    if (o.diem_tin > 100)
      o.trang_thai = 2;
  }
}

static void lap_day_vung_kin() {
  bool tiep_can[LUOI_SO_HANG][LUOI_SO_COT];
  memset(tiep_can, 0, sizeof(tiep_can));

  queue<int> hang_doi;
  auto push_neu_hop_le = [&](int c, int h) {
    if ((unsigned)c >= (unsigned)LUOI_SO_COT ||
        (unsigned)h >= (unsigned)LUOI_SO_HANG)
      return;
    if (ban_do[h][c].trang_thai == 2 || tiep_can[h][c])
      return;
    tiep_can[h][c] = true;
    hang_doi.push(h * LUOI_SO_COT + c);
  };

  // Khởi tạo BFS từ tất cả ô thuộc 4 cạnh biên
  for (int c = 0; c < LUOI_SO_COT; c++) {
    push_neu_hop_le(c, 0);
    push_neu_hop_le(c, LUOI_SO_HANG - 1);
  }
  for (int h = 1; h < LUOI_SO_HANG - 1; h++) {
    push_neu_hop_le(0, h);
    push_neu_hop_le(LUOI_SO_COT - 1, h);
  }

  // BFS lan rộng ra toàn bộ vùng tiếp cận được từ biên
  const int dc[] = {0, 0, -1, 1};
  const int dh[] = {-1, 1, 0, 0};
  while (!hang_doi.empty()) {
    int hien = hang_doi.front();
    hang_doi.pop();
    int cc = hien % LUOI_SO_COT;
    int ch = hien / LUOI_SO_COT;
    for (int i = 0; i < 4; i++)
      push_neu_hop_le(cc + dc[i], ch + dh[i]);
  }

  // Lấp đầy: mọi ô 0 không tiếp cận được từ biên → bị bao kín → đánh dấu = 2
  int so_lap = 0;
  for (int h = 0; h < LUOI_SO_HANG; h++) {
    for (int c = 0; c < LUOI_SO_COT; c++) {
      if (ban_do[h][c].trang_thai == 0 && !tiep_can[h][c]) {
        ban_do[h][c].trang_thai = 2;
        ban_do[h][c].diem_tin = 100; // đánh dấu chắc chắn
        so_lap++;
      }
    }
  }
  if (so_lap > 0)
    printf("[BanDo] Lap day %d o bi bao kin boi vat can.\n", so_lap);
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

  // Lấy thiết bị IMU (đảm bảo tên "inertial unit" khớp với tên trong file
  // .wbt)
  InertialUnit *imu = robot->getInertialUnit("inertial unit");
  imu->enable(buoc_ms); // Kích hoạt IMU để đọc dữ liệu cảm biến
  // --- Khởi tạo thiết bị ---
  Motor *dc_trai = robot->getMotor("left wheel motor");
  Motor *dc_phai = robot->getMotor("right wheel motor");
  dc_trai->setPosition(INFINITY); // chế độ điều khiển vận tốc
  dc_phai->setPosition(INFINITY);

  PositionSensor *enc_trai = robot->getPositionSensor("left wheel sensor");
  PositionSensor *enc_phai = robot->getPositionSensor("right wheel sensor");
  enc_trai->enable(buoc_ms);
  enc_phai->enable(buoc_ms);

  // GPS để đọc vị trí thực tế ban đầu của robot
  GPS *gps = robot->getGPS("gps");
  if (gps)
    gps->enable(buoc_ms);

  // 2. KHỞI TẠO LIDAR
  // ============================================================
  Lidar *lidar = robot->getLidar("LDS-01");
  lidar->enable(buoc_ms);
  lidar->enablePointCloud();
  // Bật chế độ Point Cloud: trả về tọa độ (x, y, z) của
  // từng điểm vật cản so với robot (dùng để visualize)

  // ============================================================
  // 3. KHỞI TẠO MOTOR QUAY CỤM LIDAR (quét 360°)
  // ============================================================
  Motor *lidar_main_motor = robot->getMotor("LDS-01_main_motor");
  Motor *lidar_secondary_motor = robot->getMotor("LDS-01_secondary_motor");

  if (lidar_main_motor && lidar_secondary_motor) {
    lidar_main_motor->setPosition(INFINITY);
    lidar_secondary_motor->setPosition(INFINITY);
    // INFINITY → chế độ quay liên tục (không dừng ở góc cố định)

    lidar_main_motor->setVelocity(30.0);
    lidar_secondary_motor->setVelocity(60.0);
    // Tốc độ quay của cụm LiDAR (rad/s)
    // Motor phụ quay nhanh gấp đôi để tạo chuyển động quét ổn định
  }

  TouchSensor *bumper_trai = robot->getTouchSensor("bumper_left");
  TouchSensor *bumper_phai = robot->getTouchSensor("bumper_right");
  if (bumper_trai)
    bumper_trai->enable(buoc_ms);
  if (bumper_phai)
    bumper_phai->enable(buoc_ms);

  Display *display = robot->getDisplay("display");

  TrangThai trang_thai = QUET_360;
  TrangThaiRobot rs;

  if (gps) {
    robot->step(buoc_ms);
    const double *vi_tri_gps = gps->getValues();
    if (vi_tri_gps) {
      rs.vi_tri_x = vi_tri_gps[0];
      rs.vi_tri_y = vi_tri_gps[1];
      rs.enc_trai_cu = enc_trai->getValue();
      rs.enc_phai_cu = enc_phai->getValue();
    }
  }

  const int BUOC_CHO_KHOI_DONG = 2000 / buoc_ms;
  int dem_khoi_dong = 0;

  while (robot->step(buoc_ms) != -1) {

    const double yaw = imu->getRollPitchYaw()[2];
    const double dt = buoc_ms / 1000.0;

    if (rs.buoc_dau) {
      rs.goc_truoc = yaw;
      rs.buoc_dau = false;
    }
    const double van_toc_goc = chuan_hoa_goc(yaw - rs.goc_truoc) / dt;
    rs.goc_truoc = yaw;

    const float *anh_lidar = lidar->getRangeImage();
    if (!anh_lidar)
      continue;

    const double enc_L = enc_trai->getValue();
    const double enc_R = enc_phai->getValue();
    const double delta = ((enc_L - rs.enc_trai_cu) + (enc_R - rs.enc_phai_cu)) *
                         0.5 * BANH_XE_BAN_KINH;

    if (gps && gps->getValues() && !isnan(gps->getValues()[0])) {
      const double *vi_tri_gps = gps->getValues();
      rs.vi_tri_x = vi_tri_gps[0];
      rs.vi_tri_y = vi_tri_gps[1];
    } else {
      rs.vi_tri_x += delta * cos(yaw);
      rs.vi_tri_y += delta * sin(yaw);
    }

    rs.enc_trai_cu = enc_L;
    rs.enc_phai_cu = enc_R;

    int gc, gr;
    the_gioi_ra_luoi(rs.vi_tri_x, rs.vi_tri_y, gc, gr);

    if ((unsigned)gc < (unsigned)LUOI_SO_COT &&
        (unsigned)gr < (unsigned)LUOI_SO_HANG) {
      double cx, cy;
      luoi_ra_the_gioi(gc, gr, cx, cy);
      if (hypot(rs.vi_tri_x - cx, rs.vi_tri_y - cy) < 0.05) {
        OLuoi &o_hien_tai = ban_do[gr][gc];
        if (o_hien_tai.trang_thai == 2)
          o_hien_tai.diem_tin =
              (o_hien_tai.diem_tin > 0) ? o_hien_tai.diem_tin - 200 : 0;
        o_hien_tai.trang_thai = 1;
      }
    }
    if (rs.dang_phuc_hoi && rs.chi_so < rs.duong_di.size()) {
      const DiemLuoi &wp = rs.duong_di[rs.chi_so];
      if ((unsigned)wp.cot < (unsigned)LUOI_SO_COT &&
          (unsigned)wp.hang < (unsigned)LUOI_SO_HANG) {
        OLuoi &o_wp = ban_do[wp.hang][wp.cot];
        if (o_wp.trang_thai == 2) {
          o_wp.trang_thai = 1;
          o_wp.diem_tin = (o_wp.diem_tin > 0) ? o_wp.diem_tin - 200 : 0;
        }
      }
    }
    cap_nhat_ban_do(anh_lidar, rs.vi_tri_x, rs.vi_tri_y, yaw, van_toc_goc);

    if (g_buoc % 50 == 0)
      lap_day_vung_kin();

    const bool va_cham = (bumper_trai && bumper_trai->getValue() > 0.5) ||
                         (bumper_phai && bumper_phai->getValue() > 0.5);
    if (va_cham && !g_bumper_cu)
      g_so_va_cham++;
    g_bumper_cu = va_cham;

    g_buoc++;
    ve_dashboard((int)trang_thai, rs.vi_tri_x, rs.vi_tri_y, yaw, gc, gr,
                 anh_lidar, ban_do);
    ve_hien_thi_ban_do(display);

    // ============================================================
    //  GIAI ĐOẠN KHỞI ĐỘNG: Robot đứng yên 2 giây để LiDAR quét 360°
    // ============================================================
    if (dem_khoi_dong < BUOC_CHO_KHOI_DONG) {
      dem_khoi_dong++;
      // Giữ motor dừng hoàn toàn, bỏ qua FSM
      dieu_khien_vi_sai(0.0, 0.0, dc_trai, dc_phai);
      continue;
    }

    // Đầu ra điều khiển (reset mỗi bước)
    double cv = 0.0; // vận tốc tịnh tiến [m/s]
    double co = 0.0; // vận tốc góc       [rad/s]

    // ============================================================
    //  FSM – XỬ LÝ TỪNG TRẠNG THÁI
    // ============================================================
    switch (trang_thai) {

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

    case QUAY_VE_TUONG_1:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC)
        trang_thai = TIEN_DEN_TUONG_1;
      break;

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
      rs.chieu_tuong_2 = (kc_phai <= kc_trai) ? -1 : +1;
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
        cout << "\n[Phase 1 → 2] Can goc xong!"
             << "  Huong ZigZag=" << fixed << setprecision(2)
             << (rs.huong_goc * 180.0 / M_PI) << " deg"
             << "  Chieu queo=" << (rs.chieu_queo > 0 ? "TRAI" : "PHAI")
             << "  → Quay 180°...\n";
        trang_thai = QUAY_180;
      }
    } break;

    // ------------------------------------------------------------
    //  PHASE 2: QUAY 180° ĐỂ CHUẨN BỊ QUÉT
    // ------------------------------------------------------------
    case QUAY_180:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        cout << "\n[Phase 2 → 3] Quay xong! Bat dau quet ZigZag!"
             << "  Yaw=" << fixed << setprecision(2) << (yaw * 180.0 / M_PI)
             << " deg\n";
        trang_thai = TIEN;
      }
      break;

    // ------------------------------------------------------------
    //  PHASE 3: QUÉT ZIGZAG
    // ------------------------------------------------------------

    // Tiến thẳng theo hàng hiện tại
    case TIEN: {
      float kc_truoc = lidar_tai(anh_lidar, 0);

      // --- Chống va chạm đa cấp ---
      if (kc_truoc > VUNG_AN_TOAN) {
        cv = V_ZIGZAG;
      } else if (kc_truoc > VUNG_NGUY_HIEM) {
        double toc_do_min = 0.05;
        cv = toc_do_min +
             (V_ZIGZAG - toc_do_min) * ((kc_truoc - VUNG_NGUY_HIEM) /
                                        (VUNG_AN_TOAN - VUNG_NGUY_HIEM));
      } else {
        cv = 0;
      }

      // Điều hướng PID: Bám theo đường tâm của các ô lưới
      double center_x, center_y;
      luoi_ra_the_gioi(gc, gr, center_x, center_y);
      double cte = (rs.vi_tri_x - center_x) * (-sin(rs.huong_hang)) +
                   (rs.vi_tri_y - center_y) * cos(rs.huong_hang);

      co = KP_GIU_THANG * chuan_hoa_goc(rs.huong_hang - yaw) - 4.5 * cte;

      if (va_cham) {
        trang_thai = LUI;
        break;
      }

      // het_hang:
      //  - Zigzag ban đầu (dang_phuc_hoi=false): dừng theo LiDAR + lưới ô 0
      //  phía trước
      //  - BFS recovery (dang_phuc_hoi=true) : dừng ngay khi ô kế tiếp ≠ 0
      {
        bool het_hang;
        if (rs.dang_phuc_hoi) {
          int dc_fwd = (int)round(cos(rs.huong_hang));
          int dh_fwd = (int)round(sin(rs.huong_hang));
          int nc_fwd = gc + dc_fwd;
          int nh_fwd = gr + dh_fwd;
          bool o_ke_khong_phai_0 = true; // ngoài biên → dừng
          if ((unsigned)nc_fwd < (unsigned)LUOI_SO_COT &&
              (unsigned)nh_fwd < (unsigned)LUOI_SO_HANG)
            o_ke_khong_phai_0 = (ban_do[nh_fwd][nc_fwd].trang_thai != 0);
          het_hang = o_ke_khong_phai_0;
        } else {
          // Zigzag thường: dừng khi không còn ô 0 phía trước và LiDAR xác nhận
          het_hang = dem_o_trong_theo_huong(gc, gr, rs.huong_hang) == 0 &&
                     kc_truoc <= (float)NGUONG_VAT_CAN;
        }

        // Đảm bảo robot đi vào đủ sâu trong ô hiện tại (gần tâm ô) trước khi
        // quay Tính khoảng cách từ vị trí hiện tại đến tâm ô dọc theo hướng di
        // chuyển
        double khoang_cach_den_tam =
            (center_x - rs.vi_tri_x) * cos(rs.huong_hang) +
            (center_y - rs.vi_tri_y) * sin(rs.huong_hang);
        bool da_di_het_o = (khoang_cach_den_tam <= 0.05);

        if ((kc_truoc <= VUNG_NGUY_HIEM || het_hang) && da_di_het_o) {
          if (kc_truoc <= VUNG_NGUY_HIEM) {
            // Ép kiểu trạng thái ô phía trước thành Vật Cản (2) ngay lập tức
            int dc = (int)round(cos(rs.huong_hang));
            int dh = (int)round(sin(rs.huong_hang));
            int nc = max(0, min(gc + dc, LUOI_SO_COT - 1));
            int nh = max(0, min(gr + dh, LUOI_SO_HANG - 1));
            ban_do[nh][nc].trang_thai = 2;
            ban_do[nh][nc].diem_tin = 100;
          }

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
      } // end scope het_hang
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
          // Bị chặn vật lý ngay khi quay xong → BFS
          rs.dich_bi_chan = true;
          rs.huong_tiep_tuc = rs.huong_hang;
          rs.luu_cot = max(0, min(gc, LUOI_SO_COT - 1));
          rs.luu_hang = max(0, min(gr, LUOI_SO_HANG - 1));
          cout << "  [QUAY_1] Bi chan! -> BFS tu o (" << rs.luu_cot << ","
               << rs.luu_hang << ")\n";
          trang_thai = DUNG_CHO_BFS;
          rs.dem_cho_bfs = 0;
        } else {
          // Không bị chặn → dịch sang hàng mới (BFS sẽ xử lý nếu hết ô)
          rs.dich_bat_dau_x = rs.vi_tri_x;
          rs.dich_bat_dau_y = rs.vi_tri_y;
          trang_thai = DICH_HANG;
        }
      }
      break;

    // Dịch chuyển ngang KHOANG_DICH_HANG để sang hàng kế tiếp
    case DICH_HANG: {
      cv = V_DICH_HANG;

      // Bám tâm lưới khi dịch chuyển ngang
      double center_x, center_y;
      luoi_ra_the_gioi(gc, gr, center_x, center_y);
      double cte = (rs.vi_tri_x - center_x) * (-sin(rs.goc_muc_tieu)) +
                   (rs.vi_tri_y - center_y) * cos(rs.goc_muc_tieu);

      co = KP_GIU_THANG * chuan_hoa_goc(rs.goc_muc_tieu - yaw) - 4.5 * cte;

      double khoang_da_di = hypot(rs.vi_tri_x - rs.dich_bat_dau_x,
                                  rs.vi_tri_y - rs.dich_bat_dau_y);
      bool bi_chan = lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN || va_cham;

      if (bi_chan && khoang_da_di < KHOANG_DICH_HANG * 0.5) {
        // Chưa dịch được nửa đoạn mà đã bị chặn → BFS
        rs.dich_bi_chan = true;
        rs.huong_tiep_tuc = rs.huong_hang;
        rs.luu_cot = max(0, min(gc, LUOI_SO_COT - 1));
        rs.luu_hang = max(0, min(gr, LUOI_SO_HANG - 1));
        trang_thai = DUNG_CHO_BFS;
        rs.dem_cho_bfs = 0;
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
          rs.luu_cot = max(0, min(gc, LUOI_SO_COT - 1));
          rs.luu_hang = max(0, min(gr, LUOI_SO_HANG - 1));
          trang_thai = DUNG_CHO_BFS;
          rs.dem_cho_bfs = 0;
        } else if (rs.dang_phuc_hoi) {
          // BFS recovery: kiểm tra ô tiếp theo có phải ô 0 không
          int dc_fwd2 = (int)round(cos(rs.huong_hang));
          int dh_fwd2 = (int)round(sin(rs.huong_hang));
          int nc_fwd2 = gc + dc_fwd2;
          int nh_fwd2 = gr + dh_fwd2;
          bool co_o_0 = (ban_do[gr][gc].trang_thai == 0); // ô hiện tại
          if ((unsigned)nc_fwd2 < (unsigned)LUOI_SO_COT &&
              (unsigned)nh_fwd2 < (unsigned)LUOI_SO_HANG)
            co_o_0 |= (ban_do[nh_fwd2][nc_fwd2].trang_thai == 0);
          if (!co_o_0) {
            rs.huong_tiep_tuc = rs.huong_hang;
            rs.luu_cot = max(0, min(gc, LUOI_SO_COT - 1));
            rs.luu_hang = max(0, min(gr, LUOI_SO_HANG - 1));
            cout << "  [QUAY_2] Khong co o 0 phia truoc! → BFS\n";
            trang_thai = DUNG_CHO_BFS;
            rs.dem_cho_bfs = 0;
          } else {
            trang_thai = TIEN;
          }
        } else {
          // Zigzag thường: tiếp tục bình thường
          trang_thai = TIEN;
        }
      }
      break;

    // ------------------------------------------------------------
    //  PHASE 4: ĐIỀU HƯỚNG BFS + PHỤC HỒI
    // ------------------------------------------------------------

    // Dừng lại 2 giây để LiDAR quét nét bản đồ trước khi BFS
    case DUNG_CHO_BFS:
      cv = co = 0;
      if (rs.dem_cho_bfs < 500 / buoc_ms) {
        rs.dem_cho_bfs++;
      } else {
        trang_thai = KIEM_TRA_BAN_DO;
      }
      break;

    // Chạy BFS tìm ô chưa thăm gần nhất
    case KIEM_TRA_BAN_DO:
      cv = co = 0;
      // Nếu luu_cot/luu_hang ngoài biên → dùng vị trí robot hiện tại
      {
        int bfs_c = rs.luu_cot;
        int bfs_h = rs.luu_hang;
        if ((unsigned)bfs_c >= (unsigned)LUOI_SO_COT ||
            (unsigned)bfs_h >= (unsigned)LUOI_SO_HANG) {
          // Clamp về biên hợp lệ gần nhất
          bfs_c = max(0, min(gc, LUOI_SO_COT - 1));
          bfs_h = max(0, min(gr, LUOI_SO_HANG - 1));
          printf("  [BFS] luu ngoai bien → fallback den vi tri hien tai "
                 "(%d,%d)\n",
                 bfs_c, bfs_h);
        }
        rs.luu_cot = bfs_c;
        rs.luu_hang = bfs_h;
        rs.duong_di = tim_o_chua_tham(bfs_c, bfs_h);
      }
      if (rs.duong_di.empty()) {
        cout << "\n  [BFS] Khong con o chua tham! → HOAN THANH\n";
        trang_thai = HOAN_THANH;
      } else {
        rs.chi_so = 1;
        cout << "  [BFS] Tim thay duong di! " << rs.duong_di.size()
             << " buoc, dich = o(" << rs.duong_di.back().cot << ","
             << rs.duong_di.back().hang << ")\n";
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
          cout << "  [BFS] Da den dich! o(" << gc << "," << gr << ")"
               << "  Huong tiep tuc=" << fixed << setprecision(1)
               << (rs.huong_tiep_tuc * 180.0 / M_PI) << " deg"
               << "  Chieu queo=" << (rs.chieu_queo > 0 ? "TRAI" : "PHAI")
               << "\n";
          trang_thai = CAN_LAI_HUONG;
        } else {
          trang_thai = DH_QUAY; // tiếp tục đến điểm kế
        }
      }
    } break;

    // Lùi khi gặp vật cản trong lúc BFS
    case DH_LUI:
      cv = V_LUI;
      if (!va_cham &&
          lidar_tai(anh_lidar, 0) > (float)(NGUONG_VAT_CAN + 0.05)) {
        rs.luu_cot = max(0, min(gc, LUOI_SO_COT - 1));
        rs.luu_hang = max(0, min(gr, LUOI_SO_HANG - 1));
        trang_thai = DUNG_CHO_BFS;
        rs.dem_cho_bfs = 0;
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
        // Thống kê cuối cùng
        int da_tham_f = 0, chua_tham_f = 0, vat_can_f = 0;
        for (int i = 0; i < LUOI_SO_HANG; i++)
          for (int j = 0; j < LUOI_SO_COT; j++) {
            switch (ban_do[i][j].trang_thai) {
            case 0:
              chua_tham_f++;
              break;
            case 1:
              da_tham_f++;
              break;
            case 2:
              vat_can_f++;
              break;
            }
          }
        int tong_f = LUOI_SO_HANG * LUOI_SO_COT;
        cout << "\n╔═══════════════════════════════════════════╗\n"
             << "║         HOAN THANH - KET QUA QUET         ║\n"
             << "╠═══════════════════════════════════════════╣\n"
             << "║  Tong so buoc       : " << setw(8) << g_buoc
             << "             ║\n"
             << "║  Vị trí  cuoi (X,Y)  : (" << fixed << setprecision(3)
             << setw(7) << rs.vi_tri_x << ", " << setw(7) << rs.vi_tri_y
             << ") m       ║\n"
             << "║  O DA THAM  (#)     : " << setw(4) << da_tham_f << " / "
             << setw(4) << tong_f << "             ║\n"
             << "║  O CHUA THAM(.)     : " << setw(4) << chua_tham_f
             << "                    ║\n"
             << "║  VAT CAN    (X)     : " << setw(4) << vat_can_f
             << "                    ║\n"
             << "║  Do phu            : " << fixed << setprecision(1) << setw(5)
             << (100.0 * da_tham_f / tong_f) << " %               ║\n"
             << "╚═══════════════════════════════════════════╝\n";
        // Dashboard lần cuối sẽ tự cập nhật qua ve_dashboard ở đầu vòng lặp
        da_in = true;
      }
      break;

    } // end switch

    dieu_khien_vi_sai(cv, co, dc_trai, dc_phai);

  } // end while

  delete robot;
  return 0;
}