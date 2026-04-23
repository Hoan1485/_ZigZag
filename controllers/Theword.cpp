// ============================================================
// may_hut_bui.cpp
// Máy Điều Khiển Robot Hút Bụi
// FSM Tích Hợp: Lập Bản Đồ Lưới + BFS + Quét ZigZag
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
using namespace std; // Đã thêm để rút gọn cú pháp

// ============================================================
// HẰNG SỐ CẤU HÌNH
// ============================================================

static constexpr double BAN_KINH_BANH_XE = 0.031; 
static constexpr double KHOANG_CACH_BANH = 0.258; 
static constexpr double TOC_DO_GOC_TOI_DA = 6.28; 

static constexpr double TOC_DO_TIEN_TUONG_1 = 0.20;
static constexpr double TOC_DO_TIEN_TUONG_2 = 0.18;
static constexpr double TOC_DO_LUI          = -0.10;
static constexpr double TOC_DO_TIEN_ZIGZAG  = 0.25;
static constexpr double TOC_DO_DICH_HANG    = 0.18;
static constexpr double TOC_DO_DIEU_HUONG   = 0.30;

static constexpr double KHOANG_DICH_HANG    = 0.20;
static constexpr double NGUONG_VAT_CAN      = 0.20;
static constexpr double DUNG_CACH_TUONG_1   = 0.15;
static constexpr double DUNG_CACH_TUONG_2   = 0.15;

static constexpr double NGUONG_SAI_SO_GOC   = 0.012;
static constexpr double NGUONG_DEN_NOI      = 0.08;

static constexpr double HE_SO_GIU_THANG     = 4.5;
static constexpr double HE_SO_QUAY          = 5.5;

static constexpr double DO_SANG_RAD = M_PI / 180.0;

static constexpr double KICH_THUOC_O_LUOI = 0.15;
static constexpr int SO_COT_LUOI = 100;
static constexpr int SO_HANG_LUOI = 100;
static constexpr int DICH_NGUON_COT = 50;
static constexpr int DICH_NGUON_HANG = 50;

struct OLuoi {
  int8_t trang_thai = 0; 
  int8_t diem_tin   = 0; 
};

static OLuoi ban_do[SO_HANG_LUOI][SO_COT_LUOI];

enum TrangThai {
  QUET_360, QUAY_VE_TUONG_1, TIEN_DEN_TUONG_1, LUI_1,
  TIM_TUONG_2, QUAY_VE_TUONG_2, TIEN_DEN_TUONG_2, LUI_2,
  CAN_CHINH_GOC, QUAY_180,
  TIEN, LUI, QUAY_1, DICH_HANG, QUAY_2,
  KIEM_TRA_BAN_DO, DH_QUAY, DH_TIEN, DH_LUI, CAN_LAI_HUONG,
  HOAN_THANH
};

struct DiemLuoi { int cot, hang; };

struct TrangThaiRobot {
  double vi_tri_x = 0, vi_tri_y = 0;
  double enc_trai_cu = 0, enc_phai_cu = 0;
  double goc_muc_tieu = 0, huong_goc = 0, huong_hang = 0, huong_tiep_tuc = 0, goc_truoc = 0;
  bool buoc_dau = true;
  double goc_tuong_1 = 0;
  int chieu_tuong_2  = 1;
  double dich_bat_dau_x = 0, dich_bat_dau_y = 0;
  int chieu_queo     = 1;
  bool dich_bi_chan  = false;
  int luu_cot = 0, luu_hang = 0;
  vector<DiemLuoi> duong_di; // Đã rút gọn std::
  size_t chi_so = 0;
  bool dang_phuc_hoi = false;
};

// ============================================================
// HÀM TIỆN ÍCH
// ============================================================

static inline double chuan_hoa_goc(double a) {
  a = fmod(a + M_PI, 2.0 * M_PI); // Đã rút gọn std::
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

static void dieu_khien_vi_sai(double van_toc_tien, double van_toc_goc, Motor *dc_trai, Motor *dc_phai) {
  double vT = (van_toc_tien - van_toc_goc * KHOANG_CACH_BANH * 0.5) / BAN_KINH_BANH_XE;
  double vP = (van_toc_tien + van_toc_goc * KHOANG_CACH_BANH * 0.5) / BAN_KINH_BANH_XE;
  double s  = max(abs(vT), abs(vP)); // Đã rút gọn std::
  double ty_le_giam = (s > TOC_DO_GOC_TOI_DA) ? TOC_DO_GOC_TOI_DA / s : 1.0;
  dc_trai->setVelocity(vT * ty_le_giam);
  dc_phai->setVelocity(vP * ty_le_giam);
}

static inline float lidar_tai(const float *anh, int goc) {
  int vi_tri = ((goc % 360) + 360) % 360;
  float d = anh[vi_tri];
  return (d < 0.01f || isinf(d) || isnan(d)) ? 9.9f : d; // Đã rút gọn std::
}

static float lidar_trung_binh(const float *anh, int goc_dau, int goc_cuoi) {
  float tong = 0; int dem = 0;
  for (int g = goc_dau; g <= goc_cuoi; g++) {
    float v = lidar_tai(anh, g);
    if (v < 9.0f) { tong += v; dem++; }
  }
  return (dem > 0) ? tong / dem : 9.9f;
}

static void the_gioi_ra_luoi(double x, double y, int &cot, int &hang) {
  cot  = (int)floor(x / KICH_THUOC_O_LUOI) + DICH_NGUON_COT;
  hang = (int)floor(y / KICH_THUOC_O_LUOI) + DICH_NGUON_HANG;
}

static void luoi_ra_the_gioi(int cot, int hang, double &x, double &y) {
  x = (cot  - DICH_NGUON_COT)  * KICH_THUOC_O_LUOI + (KICH_THUOC_O_LUOI * 0.5);
  y = (hang - DICH_NGUON_HANG) * KICH_THUOC_O_LUOI + (KICH_THUOC_O_LUOI * 0.5);
}

static void cap_nhat_ban_do(const float *anh, double rx, double ry, double goc_robot, double van_toc_goc) {
  if (!anh || abs(van_toc_goc) > 1.0) return;
  for (int i = 0; i < 360; i += 5) {
    float kc = anh[i];
    if (kc <= 0.05f || kc >= 2.0f) continue;
    double goc = chuan_hoa_goc(goc_robot - i * DO_SANG_RAD);
    int cot, hang;
    the_gioi_ra_luoi(rx + kc * cos(goc), ry + kc * sin(goc), cot, hang);
    if ((unsigned)cot >= (unsigned)SO_COT_LUOI || (unsigned)hang >= (unsigned)SO_HANG_LUOI) continue;
    OLuoi &o = ban_do[hang][cot];
    if (o.trang_thai == 2) continue;
    o.diem_tin = (int8_t)min((int)o.diem_tin + ((kc < 1.0f) ? 3 : 1), 100);
    if (o.diem_tin > 5) o.trang_thai = 2;
  }
}

vector<DiemLuoi> tim_o_chua_tham(int cot_dau, int hang_dau) {
  if ((unsigned)cot_dau >= (unsigned)SO_COT_LUOI || (unsigned)hang_dau >= (unsigned)SO_HANG_LUOI) return {};

  static int cha[SO_HANG_LUOI * SO_COT_LUOI];
  fill(cha, cha + SO_HANG_LUOI * SO_COT_LUOI, -1);

  auto chi_so_1d = [](int c, int h) { return h * SO_COT_LUOI + c; };
  int bat_dau = chi_so_1d(cot_dau, hang_dau);
  cha[bat_dau] = bat_dau; 

  queue<int> hang_doi;
  hang_doi.push(bat_dau);

  const int dc[] = {0, 0, -1, 1}, dh[] = {-1, 1, 0, 0};
  int muc_tieu = -1;

  while (!hang_doi.empty() && muc_tieu < 0) {
    int hien = hang_doi.front(); hang_doi.pop();
    int cc = hien % SO_COT_LUOI, ch = hien / SO_COT_LUOI;
    if (ban_do[ch][cc].trang_thai == 0) { muc_tieu = hien; break; }
    
    for (int i = 0; i < 4; i++) {
      int nc = cc + dc[i], nh = ch + dh[i];
      if ((unsigned)nc >= (unsigned)SO_COT_LUOI || (unsigned)nh >= (unsigned)SO_HANG_LUOI) continue;
      int cs_lan_can = chi_so_1d(nc, nh);
      if (cha[cs_lan_can] >= 0 || ban_do[nh][nc].trang_thai == 2) continue;
      cha[cs_lan_can] = hien;
      hang_doi.push(cs_lan_can);
    }
  }

  if (muc_tieu < 0) return {};

  vector<DiemLuoi> duong;
  for (int hien = muc_tieu; hien != bat_dau; hien = cha[hien])
    duong.push_back({hien % SO_COT_LUOI, hien / SO_COT_LUOI});
  duong.push_back({cot_dau, hang_dau});
  reverse(duong.begin(), duong.end()); 
  return duong;
}

static int dem_o_trong_theo_huong(int cot, int hang, double huong) {
  int dc = (int)round(cos(huong)), dh = (int)round(sin(huong)), so_o = 0;
  int cc = cot, ch = hang;
  while (true) {
    cc += dc; ch += dh;
    if ((unsigned)cc >= (unsigned)SO_COT_LUOI || (unsigned)ch >= (unsigned)SO_HANG_LUOI || ban_do[ch][cc].trang_thai == 2) break;
    if (ban_do[ch][cc].trang_thai == 0) so_o++;
  }
  return so_o;
}

// ============================================================
// MAIN
// ============================================================
int main() {
  Robot *robot = new Robot();
  int buoc_thoi_gian = (int)robot->getBasicTimeStep();

  Motor *dc_trai = robot->getMotor("left wheel motor");
  Motor *dc_phai = robot->getMotor("right wheel motor");
  dc_trai->setPosition(INFINITY); dc_phai->setPosition(INFINITY);

  PositionSensor *enc_trai = robot->getPositionSensor("left wheel sensor");
  PositionSensor *enc_phai = robot->getPositionSensor("right wheel sensor");
  enc_trai->enable(buoc_thoi_gian); enc_phai->enable(buoc_thoi_gian);

  Lidar *lidar = robot->getLidar("LDS-01"); lidar->enable(buoc_thoi_gian);
  InertialUnit *imu = robot->getInertialUnit("inertial unit"); imu->enable(buoc_thoi_gian);
  
  TouchSensor *bumper_trai = robot->getTouchSensor("bumper_left");
  TouchSensor *bumper_phai = robot->getTouchSensor("bumper_right");
  if (bumper_trai) bumper_trai->enable(buoc_thoi_gian);
  if (bumper_phai) bumper_phai->enable(buoc_thoi_gian);

  TrangThai trang_thai = QUET_360;
  TrangThaiRobot rs;

  printf("[Hệ Thống] Bắt đầu khởi động (Phase 1: Tìm góc)...\n");

  while (robot->step(buoc_thoi_gian) != -1) {
    const double yaw = imu->getRollPitchYaw()[2];
    const double dt  = buoc_thoi_gian / 1000.0;

    if (rs.buoc_dau) { rs.goc_truoc = yaw; rs.buoc_dau = false; }
    const double van_toc_goc = chuan_hoa_goc(yaw - rs.goc_truoc) / dt;
    rs.goc_truoc = yaw;

    const float *anh_lidar = lidar->getRangeImage();
    if (!anh_lidar) continue;

    const double enc_L = enc_trai->getValue(), enc_R = enc_phai->getValue();
    const double delta = ((enc_L - rs.enc_trai_cu) + (enc_R - rs.enc_phai_cu)) * 0.5 * BAN_KINH_BANH_XE;
    rs.vi_tri_x += delta * cos(yaw);
    rs.vi_tri_y += delta * sin(yaw);
    rs.enc_trai_cu = enc_L; rs.enc_phai_cu = enc_R;

    int gc, gr;
    the_gioi_ra_luoi(rs.vi_tri_x, rs.vi_tri_y, gc, gr);
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            int hc = gc + i, hr = gr + j;
            if ((unsigned)hc < (unsigned)SO_COT_LUOI && (unsigned)hr < (unsigned)SO_HANG_LUOI && ban_do[hr][hc].trang_thai != 2) {
                ban_do[hr][hc].trang_thai = 1; 
            }
        }
    }

    cap_nhat_ban_do(anh_lidar, rs.vi_tri_x, rs.vi_tri_y, yaw, van_toc_goc);

    double cv = 0, co = 0;
    const bool va_cham = (bumper_trai && bumper_trai->getValue() > 0.5) || (bumper_phai && bumper_phai->getValue() > 0.5);

    switch (trang_thai) {
    case QUET_360: {
      float kc_nho_nhat = 99.0f; int goc_nho_nhat = 0;
      for (int i = 0; i < 360; i++) {
        float d = lidar_tai(anh_lidar, i);
        if (d < kc_nho_nhat) { kc_nho_nhat = d; goc_nho_nhat = i; }
      }
      rs.goc_muc_tieu = rs.goc_tuong_1 = chuan_hoa_goc(yaw - goc_nho_nhat * DO_SANG_RAD);
      trang_thai = QUAY_VE_TUONG_1;
    } break;

    case QUAY_VE_TUONG_1:
      co = HE_SO_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) trang_thai = TIEN_DEN_TUONG_1;
      break;

    case TIEN_DEN_TUONG_1:
      if (va_cham) trang_thai = LUI_1;
      else if (lidar_tai(anh_lidar, 0) < DUNG_CACH_TUONG_1) trang_thai = TIM_TUONG_2;
      else { cv = TOC_DO_TIEN_TUONG_1; co = HE_SO_GIU_THANG * chuan_hoa_goc(rs.goc_muc_tieu - yaw); }
      break;

    case LUI_1:
      cv = TOC_DO_LUI;
      if (!va_cham) trang_thai = TIM_TUONG_2;
      break;

    case TIM_TUONG_2: {
      rs.chieu_tuong_2 = (lidar_trung_binh(anh_lidar, 80, 100) <= lidar_trung_binh(anh_lidar, 260, 280)) ? +1 : -1;
      rs.goc_muc_tieu = chuan_hoa_goc(yaw + rs.chieu_tuong_2 * M_PI * 0.5);
      trang_thai = QUAY_VE_TUONG_2;
    } break;

    case QUAY_VE_TUONG_2:
      co = HE_SO_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) trang_thai = TIEN_DEN_TUONG_2;
      break;

    case TIEN_DEN_TUONG_2:
      if (va_cham) trang_thai = LUI_2;
      else if (lidar_tai(anh_lidar, 0) < DUNG_CACH_TUONG_2) trang_thai = CAN_CHINH_GOC;
      else { cv = TOC_DO_TIEN_TUONG_2; co = HE_SO_GIU_THANG * chuan_hoa_goc(rs.goc_muc_tieu - yaw); }
      break;

    case LUI_2:
      cv = TOC_DO_LUI;
      if (!va_cham) trang_thai = CAN_CHINH_GOC;
      break;

    case CAN_CHINH_GOC: {
      double sai_so = chuan_hoa_goc(round(yaw / (M_PI * 0.5)) * (M_PI * 0.5) - yaw);
      co = HE_SO_QUAY * sai_so;
      if (abs(sai_so) < NGUONG_SAI_SO_GOC) {
        rs.goc_muc_tieu = rs.huong_hang = rs.huong_goc = chuan_hoa_goc(yaw + sai_so + M_PI);
        rs.chieu_queo = -rs.chieu_tuong_2;
        rs.dich_bi_chan = false;
        printf("[Chuyển đổi] Căn góc xong. Quay 180°...\n");
        trang_thai = QUAY_180;
      }
    } break;

    case QUAY_180:
      co = HE_SO_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        printf("[Phase 2] Quay xong. Bắt đầu quét ZigZag!\n");
        trang_thai = TIEN;
      }
      break;

    case TIEN:
      cv = TOC_DO_TIEN_ZIGZAG;
      co = HE_SO_GIU_THANG * chuan_hoa_goc(rs.huong_hang - yaw);
      if (va_cham) trang_thai = LUI;
      else if (lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN || (rs.dang_phuc_hoi && dem_o_trong_theo_huong(gc, gr, rs.huong_hang) == 0)) {
        bool xong_het = true;
        for (int i = 0; i < SO_HANG_LUOI && xong_het; i++)
          for (int j = 0; j < SO_COT_LUOI && xong_het; j++)
            if (ban_do[i][j].trang_thai == 0) xong_het = false;

        if (xong_het) trang_thai = HOAN_THANH;
        else {
          rs.goc_muc_tieu = chuan_hoa_goc(rs.huong_hang + rs.chieu_queo * M_PI * 0.5);
          rs.dich_bi_chan = false;
          trang_thai = QUAY_1;
        }
      }
      break;

    case LUI:
      cv = TOC_DO_LUI; co = HE_SO_GIU_THANG * chuan_hoa_goc(rs.huong_hang - yaw);
      if (!va_cham) { rs.goc_muc_tieu = chuan_hoa_goc(rs.huong_hang + rs.chieu_queo * M_PI * 0.5); rs.dich_bi_chan = false; trang_thai = QUAY_1; }
      break;

    case QUAY_1:
      co = HE_SO_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        if (lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN) {
          rs.dich_bi_chan = true; rs.huong_tiep_tuc = rs.huong_hang; rs.luu_cot = gc; rs.luu_hang = gr; trang_thai = KIEM_TRA_BAN_DO;
        } else {
          rs.dich_bat_dau_x = rs.vi_tri_x; rs.dich_bat_dau_y = rs.vi_tri_y; trang_thai = DICH_HANG;
        }
      }
      break;

    case DICH_HANG: {
      cv = TOC_DO_DICH_HANG; co = HE_SO_GIU_THANG * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      double khoang_da_di = hypot(rs.vi_tri_x - rs.dich_bat_dau_x, rs.vi_tri_y - rs.dich_bat_dau_y);
      bool bi_chan = lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN || va_cham;

      if (bi_chan && khoang_da_di < KHOANG_DICH_HANG * 0.5) {
        rs.dich_bi_chan = true; rs.huong_tiep_tuc = rs.huong_hang; rs.luu_cot = gc; rs.luu_hang = gr; trang_thai = KIEM_TRA_BAN_DO;
      } else if (khoang_da_di >= KHOANG_DICH_HANG || bi_chan) {
        rs.goc_muc_tieu = chuan_hoa_goc(rs.goc_muc_tieu + rs.chieu_queo * M_PI * 0.5); trang_thai = QUAY_2;
      }
    } break;

    case QUAY_2:
      co = HE_SO_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        rs.huong_hang = rs.goc_muc_tieu; rs.chieu_queo *= -1;
        if (lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN) {
          rs.huong_tiep_tuc = rs.huong_hang; rs.luu_cot = gc; rs.luu_hang = gr; trang_thai = KIEM_TRA_BAN_DO;
        } else trang_thai = TIEN;
      }
      break;

    case KIEM_TRA_BAN_DO:
      cv = co = 0; rs.duong_di = tim_o_chua_tham(rs.luu_cot, rs.luu_hang);
      if (rs.duong_di.empty()) trang_thai = HOAN_THANH;
      else { rs.chi_so = 1; trang_thai = (rs.chi_so >= rs.duong_di.size()) ? CAN_LAI_HUONG : DH_QUAY; }
      break;

    case DH_QUAY: {
      double tx, ty; luoi_ra_the_gioi(rs.duong_di[rs.chi_so].cot, rs.duong_di[rs.chi_so].hang, tx, ty);
      double goc_den = atan2(ty - rs.vi_tri_y, tx - rs.vi_tri_x);
      co = HE_SO_QUAY * chuan_hoa_goc(goc_den - yaw);
      if (abs(chuan_hoa_goc(goc_den - yaw)) < NGUONG_SAI_SO_GOC) trang_thai = DH_TIEN;
    } break;

    case DH_TIEN: {
      double tx, ty; luoi_ra_the_gioi(rs.duong_di[rs.chi_so].cot, rs.duong_di[rs.chi_so].hang, tx, ty);
      double goc_den = atan2(ty - rs.vi_tri_y, tx - rs.vi_tri_x);
      cv = TOC_DO_DIEU_HUONG; co = HE_SO_GIU_THANG * chuan_hoa_goc(goc_den - yaw);

      if (va_cham || lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN) trang_thai = DH_LUI;
      else if (hypot(tx - rs.vi_tri_x, ty - rs.vi_tri_y) < NGUONG_DEN_NOI) {
        if (++rs.chi_so >= rs.duong_di.size()) {
          rs.dang_phuc_hoi = true;
          double huong_tot_nhat = rs.huong_goc; int so_o_nhieu_nhat = -1;
          for (int i = 0; i < 4; i++) {
            double h = rs.huong_goc + i * M_PI * 0.5;
            int so_o = dem_o_trong_theo_huong(gc, gr, h);
            if (so_o > so_o_nhieu_nhat) { so_o_nhieu_nhat = so_o; huong_tot_nhat = h; }
          }
          rs.huong_tiep_tuc = chuan_hoa_goc(huong_tot_nhat);
          rs.chieu_queo = (dem_o_trong_theo_huong(gc, gr, chuan_hoa_goc(huong_tot_nhat + M_PI * 0.5)) >=
                           dem_o_trong_theo_huong(gc, gr, chuan_hoa_goc(huong_tot_nhat - M_PI * 0.5))) ? 1 : -1;
          trang_thai = CAN_LAI_HUONG;
        } else trang_thai = DH_QUAY;
      }
    } break;

    case DH_LUI:
      cv = TOC_DO_LUI;
      if (!va_cham) { rs.luu_cot = gc; rs.luu_hang = gr; trang_thai = KIEM_TRA_BAN_DO; }
      break;

    case CAN_LAI_HUONG:
      cv = 0; rs.goc_muc_tieu = rs.huong_tiep_tuc; co = HE_SO_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) { rs.huong_hang = rs.huong_tiep_tuc; trang_thai = TIEN; }
      break;

    case HOAN_THANH:
      cv = co = 0;
      static bool da_in = false;
      if (!da_in) { printf("✓ Hoàn thành nhiệm vụ! Toàn bộ diện tích đã được phủ.\n"); da_in = true; }
      break;
    }

    dieu_khien_vi_sai(cv, co, dc_trai, dc_phai);
  }

  delete robot;
  return 0;
}