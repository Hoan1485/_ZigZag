// ============================================================
//  Robot Hút Bụi Tự Động - Webots Simulation
// ============================================================

#include <webots/InertialUnit.hpp>// Lấy giá trị góc quay (yaw) 
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>//Xác định vòng quay (Encode)
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp> //Cam bien va cham

#include <algorithm> //Các hàm toán phổ biến cho dãy và mảng
#include <cmath>
#include <cstdint> //Kieu DL có kích thước cố định
#include <cstdio> //Hien thi 
#include <queue>  //Hàng đợi (FIFO)
#include <vector>
#include <fstream> 

using namespace webots;
using namespace std;

// --- Thông số ---
static constexpr double BANH_XE_BAN_KINH = 0.031; 
static constexpr double BANH_XE_KHOANG = 0.258;   
static constexpr double MOTOR_TOC_DO_MAX = 6.28;  
static constexpr double V_TIEN_TUONG_1 = 0.20; 
static constexpr double V_TIEN_TUONG_2 = 0.18; 
static constexpr double V_LUI = -0.10;         
static constexpr double V_ZIGZAG = 0.25;       
static constexpr double V_DICH_HANG = 0.18;    
static constexpr double V_DIEU_HUONG = 0.30;   
static constexpr double KHOANG_DICH_HANG = 0.26; 
static constexpr double NGUONG_VAT_CAN = 0.12;   
static constexpr double DUNG_TUONG_1 = 0.15;     
static constexpr double DUNG_TUONG_2 = 0.15;     
static constexpr double NGUONG_DEN_NOI = 0.08;   
static constexpr double KP_GIU_THANG = 4.5; 
static constexpr double KP_QUAY = 5.5;      
static constexpr double NGUONG_SAI_SO_GOC = 0.012; 
static constexpr double DO_SANG_RAD = M_PI / 180.0;
static constexpr double O_LUOI_KICH_THUOC = 0.26; 
static constexpr int LUOI_SO_COT = 100;
static constexpr int LUOI_SO_HANG = 100;
static constexpr int LUOI_DICH_COT = 50;  
static constexpr int LUOI_DICH_HANG = 50; 

// --- Cấu trúc dữ liệu ---
struct OLuoi {
  int8_t trang_thai = 0; 
  int8_t diem_tin = 0;   
  int so_lan_tham = 0;   
};

static OLuoi ban_do[LUOI_SO_HANG][LUOI_SO_COT];

struct DiemLuoi { int cot, hang; };

enum TrangThai {
  QUET_360, QUAY_VE_TUONG_1, TIEN_DEN_TUONG_1, LUI_1,
  TIM_TUONG_2, QUAY_VE_TUONG_2, TIEN_DEN_TUONG_2, LUI_2, CAN_CHINH_GOC,
  QUAY_180, TIEN, LUI, QUAY_1, DICH_HANG, QUAY_2,
  KIEM_TRA_BAN_DO, DH_QUAY, DH_TIEN, DH_LUI, CAN_LAI_HUONG, HOAN_THANH
};

struct TrangThaiRobot {
  double vi_tri_x = 0.0, vi_tri_y = 0.0;
  double enc_trai_cu = 0.0, enc_phai_cu = 0.0;
  double goc_truoc = 0.0;
  bool buoc_dau = true; 
  double goc_muc_tieu = 0.0, huong_hang = 0.0, huong_goc = 0.0, huong_tiep_tuc = 0.0, goc_tuong_1 = 0.0;    
  int chieu_tuong_2 = 1; 
  double dich_bat_dau_x = 0.0, dich_bat_dau_y = 0.0;
  int chieu_queo = 1; 
  bool dich_bi_chan = false;
  int luu_cot = 0, luu_hang = 0;
  vector<DiemLuoi> duong_di; 
  size_t chi_so = 0;
  bool dang_phuc_hoi = false;
  double tong_quang_duong = 0.0;
  int so_o_da_tham = 0;
  int muc_phan_tram_da_in = 0;
};

// --- Hàm Tiện Ích ---
static inline double chuan_hoa_goc(double a) {
  a = fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

static void dieu_khien_vi_sai(double v_tien, double v_goc, Motor *dc_trai, Motor *dc_phai) {
  double w_trai = (v_tien - v_goc * BANH_XE_KHOANG * 0.5) / BANH_XE_BAN_KINH;
  double w_phai = (v_tien + v_goc * BANH_XE_KHOANG * 0.5) / BANH_XE_BAN_KINH;
  double w_max = max(abs(w_trai), abs(w_phai));
  double ty_le = (w_max > MOTOR_TOC_DO_MAX) ? MOTOR_TOC_DO_MAX / w_max : 1.0;
  dc_trai->setVelocity(w_trai * ty_le);
  dc_phai->setVelocity(w_phai * ty_le);
}

static inline float lidar_tai(const float *anh, int goc) {
  int vi_tri = ((goc % 360) + 360) % 360;
  float d = anh[vi_tri];
  return (d < 0.01f || isinf(d) || isnan(d)) ? 9.9f : d;
}

static float lidar_trung_binh(const float *anh, int goc_dau, int goc_cuoi) {
  float tong = 0.0f; int dem = 0;
  for (int g = goc_dau; g <= goc_cuoi; g++) {
    float v = lidar_tai(anh, g);
    if (v < 9.0f) { tong += v; dem++; }
  }
  return (dem > 0) ? tong / dem : 9.9f;
}

static void the_gioi_ra_luoi(double x, double y, int &cot, int &hang) {
  cot = (int)floor(x / O_LUOI_KICH_THUOC) + LUOI_DICH_COT;
  hang = (int)floor(y / O_LUOI_KICH_THUOC) + LUOI_DICH_HANG;
}

static void luoi_ra_the_gioi(int cot, int hang, double &x, double &y) {
  x = (cot - LUOI_DICH_COT) * O_LUOI_KICH_THUOC + (O_LUOI_KICH_THUOC * 0.5);
  y = (hang - LUOI_DICH_HANG) * O_LUOI_KICH_THUOC + (O_LUOI_KICH_THUOC * 0.5);
}

static void cap_nhat_ban_do(const float *anh, double rx, double ry, double goc_robot, double van_toc_goc) {
  if (!anh || abs(van_toc_goc) > 1.0) return;
  for (int i = 0; i < 360; i += 5) {
    float kc = anh[i];
    if (kc <= 0.05f || kc >= 2.0f) continue; 
    double goc_the_gioi = chuan_hoa_goc(goc_robot - i * DO_SANG_RAD);
    int cot, hang;
    the_gioi_ra_luoi(rx + kc * cos(goc_the_gioi), ry + kc * sin(goc_the_gioi), cot, hang);
    if ((unsigned)cot >= (unsigned)LUOI_SO_COT || (unsigned)hang >= (unsigned)LUOI_SO_HANG) continue;
    OLuoi &o = ban_do[hang][cot];
    if (o.trang_thai == 2) continue;
    int them = (kc < 1.0f) ? 3 : 1;
    o.diem_tin = (int8_t)min((int)o.diem_tin + them, 100);
    if (o.diem_tin > 5) o.trang_thai = 2; 
  }
}

vector<DiemLuoi> tim_o_chua_tham(int cot_dau, int hang_dau) {
  if ((unsigned)cot_dau >= (unsigned)LUOI_SO_COT || (unsigned)hang_dau >= (unsigned)LUOI_SO_HANG) return {};
  static int cha[LUOI_SO_HANG * LUOI_SO_COT];
  fill(cha, cha + LUOI_SO_HANG * LUOI_SO_COT, -1);
  auto chi_so_1d = [](int c, int h) { return h * LUOI_SO_COT + c; };
  int bat_dau = chi_so_1d(cot_dau, hang_dau);
  cha[bat_dau] = bat_dau; 
  queue<int> hang_doi;
  hang_doi.push(bat_dau);
  const int dc[] = {0, 0, -1, 1};
  const int dh[] = {-1, 1, 0, 0};
  int muc_tieu = -1;

  while (!hang_doi.empty() && muc_tieu < 0) {
    int hien = hang_doi.front();
    hang_doi.pop();
    int cc = hien % LUOI_SO_COT;
    int ch = hien / LUOI_SO_COT;
    if (ban_do[ch][cc].trang_thai == 0) { muc_tieu = hien; break; }
    for (int i = 0; i < 4; i++) {
      int nc = cc + dc[i]; int nh = ch + dh[i];
      if ((unsigned)nc >= (unsigned)LUOI_SO_COT || (unsigned)nh >= (unsigned)LUOI_SO_HANG) continue;
      int cs = chi_so_1d(nc, nh);
      if (cha[cs] >= 0 || ban_do[nh][nc].trang_thai == 2) continue;
      cha[cs] = hien; hang_doi.push(cs);
    }
  }

  if (muc_tieu < 0) return {};
  vector<DiemLuoi> duong;
  for (int hien = muc_tieu; hien != bat_dau; hien = cha[hien])
    duong.push_back({hien % LUOI_SO_COT, hien / LUOI_SO_COT});
  duong.push_back({cot_dau, hang_dau});
  reverse(duong.begin(), duong.end());
  return duong;
}

static int dem_o_trong_theo_huong(int cot, int hang, double huong) {
  int dc = (int)round(cos(huong)); int dh = (int)round(sin(huong));
  int so_o = 0; int cc = cot, ch = hang;
  while (true) {
    cc += dc; ch += dh;
    if ((unsigned)cc >= (unsigned)LUOI_SO_COT || (unsigned)ch >= (unsigned)LUOI_SO_HANG || ban_do[ch][cc].trang_thai == 2) break;
    if (ban_do[ch][cc].trang_thai == 0) so_o++;
  }
  return so_o;
}

// --- Hàm Khảo sát Dữ Liệu ---
void cap_nhat_du_lieu_khao_sat(TrangThaiRobot &rs, double delta, int gc, int gr, double thoi_gian_hien_tai) {
    rs.tong_quang_duong += std::abs(delta);
    if ((unsigned)gc < (unsigned)LUOI_SO_COT && (unsigned)gr < (unsigned)LUOI_SO_HANG) {
        ban_do[gr][gc].so_lan_tham++; 
        if (ban_do[gr][gc].trang_thai != 2) {
            if (ban_do[gr][gc].trang_thai == 0) rs.so_o_da_tham++;
            ban_do[gr][gc].trang_thai = 1;
        }
    }
    int phan_tram_hien_tai = (rs.so_o_da_tham * 100) / 360; 
    if (phan_tram_hien_tai >= rs.muc_phan_tram_da_in + 10 && phan_tram_hien_tai <= 100) {
        rs.muc_phan_tram_da_in = (phan_tram_hien_tai / 10) * 10;
        printf("[Khảo sát] Hoàn thành: %d%% | Thời gian: %.2f s | Quãng đường: %.2f m\n", 
               rs.muc_phan_tram_da_in, thoi_gian_hien_tai, rs.tong_quang_duong);
    }
}

void xuat_du_lieu_heatmap(double thoi_gian_tong, double quang_duong_tong) {
    printf("==> TỔNG KẾT: Thời gian: %.2f s | Quãng đường: %.2f m\n", thoi_gian_tong, quang_duong_tong);
    std::ofstream file("heatmap_data.csv");
    if (file.is_open()) {
        for (int i = 0; i < LUOI_SO_HANG; i++) {
            for (int j = 0; j < LUOI_SO_COT; j++) {
                file << ban_do[i][j].so_lan_tham;
                if (j < LUOI_SO_COT - 1) file << ",";
            }
            file << "\n";
        }
        file.close();
        printf("Đã lưu dữ liệu Heatmap vào 'heatmap_data.csv'.\n");
    } else {
        printf("Lỗi: Không thể tạo file heatmap_data.csv!\n");
    }
}

// ============================================================
//  HÀM MAIN
// ============================================================
int main() {
  Robot *robot = new Robot();
  int buoc_ms = (int)robot->getBasicTimeStep(); 

  Motor *dc_trai = robot->getMotor("left wheel motor");
  Motor *dc_phai = robot->getMotor("right wheel motor");
  dc_trai->setPosition(INFINITY); dc_phai->setPosition(INFINITY);

  PositionSensor *enc_trai = robot->getPositionSensor("left wheel sensor");
  PositionSensor *enc_phai = robot->getPositionSensor("right wheel sensor");
  enc_trai->enable(buoc_ms); enc_phai->enable(buoc_ms);

  Lidar *lidar = robot->getLidar("LDS-01");
  InertialUnit *imu = robot->getInertialUnit("inertial unit");
  lidar->enable(buoc_ms); imu->enable(buoc_ms);

  TouchSensor *bumper_trai = robot->getTouchSensor("bumper_left");
  TouchSensor *bumper_phai = robot->getTouchSensor("bumper_right");
  if (bumper_trai) bumper_trai->enable(buoc_ms);
  if (bumper_phai) bumper_phai->enable(buoc_ms);

  TrangThai trang_thai = QUET_360;
  TrangThaiRobot rs;
  printf("[Hệ Thống] Khởi động – Phase 1: Tìm góc tường...\n");

  while (robot->step(buoc_ms) != -1) {
    const double yaw = imu->getRollPitchYaw()[2]; 
    const double dt = buoc_ms / 1000.0;           

    if (rs.buoc_dau) { rs.goc_truoc = yaw; rs.buoc_dau = false; }
    const double van_toc_goc = chuan_hoa_goc(yaw - rs.goc_truoc) / dt;
    rs.goc_truoc = yaw;

    const float *anh_lidar = lidar->getRangeImage();
    if (!anh_lidar) continue;

    const double enc_L = enc_trai->getValue();
    const double enc_R = enc_phai->getValue();
    const double delta = ((enc_L - rs.enc_trai_cu) + (enc_R - rs.enc_phai_cu)) * 0.5 * BANH_XE_BAN_KINH;
    
    rs.vi_tri_x += delta * cos(yaw);
    rs.vi_tri_y += delta * sin(yaw);
    rs.enc_trai_cu = enc_L; rs.enc_phai_cu = enc_R;

    int gc, gr;
    the_gioi_ra_luoi(rs.vi_tri_x, rs.vi_tri_y, gc, gr);

    cap_nhat_du_lieu_khao_sat(rs, delta, gc, gr, robot->getTime());
    cap_nhat_ban_do(anh_lidar, rs.vi_tri_x, rs.vi_tri_y, yaw, van_toc_goc);

    const bool va_cham = (bumper_trai && bumper_trai->getValue() > 0.5) ||
                         (bumper_phai && bumper_phai->getValue() > 0.5);

    double cv = 0.0, co = 0.0; 

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
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) trang_thai = TIEN_DEN_TUONG_1;
      break;

    case TIEN_DEN_TUONG_1:
      if (va_cham) { trang_thai = LUI_1; } 
      else if (lidar_tai(anh_lidar, 0) < DUNG_TUONG_1) { trang_thai = TIM_TUONG_2; } 
      else { cv = V_TIEN_TUONG_1; co = KP_GIU_THANG * chuan_hoa_goc(rs.goc_muc_tieu - yaw); }
      break;

    case LUI_1:
      cv = V_LUI; if (!va_cham) trang_thai = TIM_TUONG_2;
      break;

    case TIM_TUONG_2: {
      float kc_phai = lidar_trung_binh(anh_lidar, 80, 100);
      float kc_trai = lidar_trung_binh(anh_lidar, 260, 280);
      rs.chieu_tuong_2 = (kc_phai <= kc_trai) ? +1 : -1;
      rs.goc_muc_tieu = chuan_hoa_goc(yaw + rs.chieu_tuong_2 * M_PI * 0.5);
      trang_thai = QUAY_VE_TUONG_2;
    } break;

    case QUAY_VE_TUONG_2:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) trang_thai = TIEN_DEN_TUONG_2;
      break;

    case TIEN_DEN_TUONG_2:
      if (va_cham) { trang_thai = LUI_2; } 
      else if (lidar_tai(anh_lidar, 0) < DUNG_TUONG_2) { trang_thai = CAN_CHINH_GOC; } 
      else { cv = V_TIEN_TUONG_2; co = KP_GIU_THANG * chuan_hoa_goc(rs.goc_muc_tieu - yaw); }
      break;

    case LUI_2:
      cv = V_LUI; if (!va_cham) trang_thai = CAN_CHINH_GOC;
      break;

    case CAN_CHINH_GOC: {
      double goc_gan_nhat = round(yaw / (M_PI * 0.5)) * (M_PI * 0.5);
      double sai_so = chuan_hoa_goc(goc_gan_nhat - yaw);
      co = KP_QUAY * sai_so;
      if (abs(sai_so) < NGUONG_SAI_SO_GOC) {
        rs.goc_muc_tieu = rs.huong_hang = rs.huong_goc = chuan_hoa_goc(yaw + sai_so + M_PI);
        rs.chieu_queo = -rs.chieu_tuong_2; rs.dich_bi_chan = false;
        printf("[Phase 1 → 2] Căn góc xong. Quay 180°...\n");
        trang_thai = QUAY_180;
      }
    } break;

    case QUAY_180:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        printf("[Phase 2 → 3] Bắt đầu quét ZigZag!\n");
        trang_thai = TIEN;
      }
      break;

    case TIEN: {
      cv = V_ZIGZAG; co = KP_GIU_THANG * chuan_hoa_goc(rs.huong_hang - yaw);
      if (va_cham) { trang_thai = LUI; break; }
      bool gap_vat_can = (lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN);
      bool het_hang = rs.dang_phuc_hoi && dem_o_trong_theo_huong(gc, gr, rs.huong_hang) == 0;

      if (gap_vat_can || het_hang) {
        bool xong_het = true;
        for (int i = 0; i < LUOI_SO_HANG && xong_het; i++)
          for (int j = 0; j < LUOI_SO_COT && xong_het; j++)
            if (ban_do[i][j].trang_thai == 0) xong_het = false;

        if (xong_het) { trang_thai = HOAN_THANH; } 
        else {
          rs.goc_muc_tieu = chuan_hoa_goc(rs.huong_hang + rs.chieu_queo * M_PI * 0.5);
          rs.dich_bi_chan = false; trang_thai = QUAY_1;
        }
      }
    } break;

    case LUI:
      cv = V_LUI; co = KP_GIU_THANG * chuan_hoa_goc(rs.huong_hang - yaw);
      if (!va_cham) {
        rs.goc_muc_tieu = chuan_hoa_goc(rs.huong_hang + rs.chieu_queo * M_PI * 0.5);
        rs.dich_bi_chan = false; trang_thai = QUAY_1;
      }
      break;

    case QUAY_1:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        if (lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN) {
          rs.dich_bi_chan = true; rs.huong_tiep_tuc = rs.huong_hang;
          rs.luu_cot = gc; rs.luu_hang = gr; trang_thai = KIEM_TRA_BAN_DO;
        } else {
          rs.dich_bat_dau_x = rs.vi_tri_x; rs.dich_bat_dau_y = rs.vi_tri_y;
          trang_thai = DICH_HANG;
        }
      }
      break;

    case DICH_HANG: {
      cv = V_DICH_HANG; co = KP_GIU_THANG * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      double khoang_da_di = hypot(rs.vi_tri_x - rs.dich_bat_dau_x, rs.vi_tri_y - rs.dich_bat_dau_y);
      bool bi_chan = lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN || va_cham;
      if (bi_chan && khoang_da_di < KHOANG_DICH_HANG * 0.5) {
        rs.dich_bi_chan = true; rs.huong_tiep_tuc = rs.huong_hang;
        rs.luu_cot = gc; rs.luu_hang = gr; trang_thai = KIEM_TRA_BAN_DO;
      } else if (khoang_da_di >= KHOANG_DICH_HANG || bi_chan) {
        rs.goc_muc_tieu = chuan_hoa_goc(rs.goc_muc_tieu + rs.chieu_queo * M_PI * 0.5);
        trang_thai = QUAY_2;
      }
    } break;

    case QUAY_2:
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        rs.huong_hang = rs.goc_muc_tieu; rs.chieu_queo *= -1; 
        if (lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN) {
          rs.huong_tiep_tuc = rs.huong_hang; rs.luu_cot = gc; rs.luu_hang = gr;
          trang_thai = KIEM_TRA_BAN_DO;
        } else { trang_thai = TIEN; }
      }
      break;

    case KIEM_TRA_BAN_DO:
      cv = co = 0;
      rs.duong_di = tim_o_chua_tham(rs.luu_cot, rs.luu_hang);
      if (rs.duong_di.empty()) { trang_thai = HOAN_THANH; } 
      else { rs.chi_so = 1; trang_thai = (rs.chi_so >= rs.duong_di.size()) ? CAN_LAI_HUONG : DH_QUAY; }
      break;

    case DH_QUAY: {
      double tx, ty;
      luoi_ra_the_gioi(rs.duong_di[rs.chi_so].cot, rs.duong_di[rs.chi_so].hang, tx, ty);
      double goc_den = atan2(ty - rs.vi_tri_y, tx - rs.vi_tri_x);
      co = KP_QUAY * chuan_hoa_goc(goc_den - yaw);
      if (abs(chuan_hoa_goc(goc_den - yaw)) < NGUONG_SAI_SO_GOC) trang_thai = DH_TIEN;
    } break;

    case DH_TIEN: {
      double tx, ty;
      luoi_ra_the_gioi(rs.duong_di[rs.chi_so].cot, rs.duong_di[rs.chi_so].hang, tx, ty);
      double goc_den = atan2(ty - rs.vi_tri_y, tx - rs.vi_tri_x);
      cv = V_DIEU_HUONG; co = KP_GIU_THANG * chuan_hoa_goc(goc_den - yaw);
      if (va_cham || lidar_tai(anh_lidar, 0) < (float)NGUONG_VAT_CAN) { trang_thai = DH_LUI; break; }
      if (hypot(tx - rs.vi_tri_x, ty - rs.vi_tri_y) < NGUONG_DEN_NOI) {
        if (++rs.chi_so >= rs.duong_di.size()) {
          rs.dang_phuc_hoi = true; double huong_tot_nhat = rs.huong_goc; int so_o_nhieu_nhat = -1;
          for (int i = 0; i < 4; i++) {
            double h = rs.huong_goc + i * M_PI * 0.5;
            int so_o = dem_o_trong_theo_huong(gc, gr, h);
            if (so_o > so_o_nhieu_nhat) { so_o_nhieu_nhat = so_o; huong_tot_nhat = h; }
          }
          rs.huong_tiep_tuc = chuan_hoa_goc(huong_tot_nhat);
          int so_trai = dem_o_trong_theo_huong(gc, gr, chuan_hoa_goc(huong_tot_nhat + M_PI * 0.5));
          int so_phai = dem_o_trong_theo_huong(gc, gr, chuan_hoa_goc(huong_tot_nhat - M_PI * 0.5));
          rs.chieu_queo = (so_trai >= so_phai) ? 1 : -1;
          trang_thai = CAN_LAI_HUONG;
        } else { trang_thai = DH_QUAY; }
      }
    } break;

    case DH_LUI:
      cv = V_LUI; if (!va_cham) { rs.luu_cot = gc; rs.luu_hang = gr; trang_thai = KIEM_TRA_BAN_DO; }
      break;

    case CAN_LAI_HUONG:
      cv = 0; rs.goc_muc_tieu = rs.huong_tiep_tuc;
      co = KP_QUAY * chuan_hoa_goc(rs.goc_muc_tieu - yaw);
      if (abs(chuan_hoa_goc(rs.goc_muc_tieu - yaw)) < NGUONG_SAI_SO_GOC) {
        rs.huong_hang = rs.huong_tiep_tuc; trang_thai = TIEN;
      }
      break;

    case HOAN_THANH:
      cv = co = 0; static bool da_in = false;
      if (!da_in) {
        printf("✓ Hoàn thành! Toàn bộ diện tích đã được phủ.\n");
        xuat_du_lieu_heatmap(robot->getTime(), rs.tong_quang_duong);
        da_in = true;
      }
      break;
    } // end switch

    dieu_khien_vi_sai(cv, co, dc_trai, dc_phai);
  } // end while

  delete robot;
  return 0;
}