#ifndef INVERSE_HEADER_H_
#define INVERSE_HEADER_H_


#include <cstdint>
#include <cmath>
#include "dynamixel_sdk/dynamixel_sdk.h"
// Control table address
#define ADDR_AX_TORQUE_ENABLE 24 //  Control table address AX is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION 30
#define ADDR_AX_GOAL_SPEED 32
#define ADDR_AX_PRESENT_POSITION 36

// Data Byte Length
#define LEN_AX 2 // AX
#define LEN_AX_PRESENT_POSITION 2

// Protocol version
#define protocol_version 1.0 // See which protocol version is used in the Dynamixel

// Default setting

#define TORQUE_ENABLE 1                 // Value for enabling the torque
#define TORQUE_DISABLE 0                // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD 10  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE 0x1b

#define DEVICE_NAME "/dev/ttyUSB0"
#define BAUDRATE 1000000


#define DXL_ID_1 1
#define DXL_ID_2 2
#define DXL_ID_3 3
// RTengah
#define DXL_ID_4 4
#define DXL_ID_5 5
#define DXL_ID_6 6
// RBelakang
#define DXL_ID_7 7
#define DXL_ID_8 8
#define DXL_ID_9 9
// LDepan
#define DXL_ID_10 10
#define DXL_ID_11 11
#define DXL_ID_12 12
// LTengah
#define DXL_ID_13 13
#define DXL_ID_14 14
#define DXL_ID_15 15
// LBelakang
#define DXL_ID_16 16
#define DXL_ID_17 17
#define DXL_ID_18 18


#define kL1 0
#define kL2 1
#define kL3 2
#define kR1 3
#define kR2 4
#define kR3 5
#define xx -1 // none

int32_t posisi_servo[18]; int32_t speed_servo[18];

uint8_t dxl_idku[18] = {DXL_ID_1, DXL_ID_2, DXL_ID_3,
                        DXL_ID_4, DXL_ID_5, DXL_ID_6,
                        DXL_ID_7, DXL_ID_8, DXL_ID_9,
                        DXL_ID_10, DXL_ID_11, DXL_ID_12,
                        DXL_ID_13, DXL_ID_14, DXL_ID_15,
                        DXL_ID_16, DXL_ID_17, DXL_ID_18};


dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);


int radToServoax(float rads)
{
    float val = rads * (1023.0 / 5.236);
    return (int)val;
}

int radToServomx(float rads)
{
    float val = rads * (4095 / 5.236);
    return (int)val;
}


    //   c       f       t
// IK 21.00; 60.00; 71.80;
float c =  21.00;       //28.64
float f =  60.00;       //65.00
float t =  71.80;       // 81.00
float pi = 3.14285714; // 3.14285714;

// panjang link kaki

float x, y, z, px, py, pz;
float theta_c, theta_f, theta_t, kalkulasi;     // sudut dari analisa IK
float theta_c_real, theta_f_real, theta_t_real; // sudut hasil normalisasi servo
float theta_f1, theta_f2, a, x0, x1;            // digunakan pd perhitungann
float sudut;                                    // digunakan untuk body statis
float step_cal = 1;                             // perhitungan step gait

// KONFIGURASI TRAJECTORY
float iterasi = 0.1;   // banyak step 0.1 = 10 ATAU 0.05=20
#define memori_data 50 // jumlah memori per koordinat per kaki
float dly_trj = 10000;  //delay trajektori yg dipakai

bool done = false;

// speed robot
int ultra_slow = 128;
int slow = 256;
int med = 512;
int med_fast = 640;
int fast = 756;
int ultra_fast = 0;

// variabel data titik trajectory
int jlh_data;

//BODY
float max_putar = 10; //derajat

//KONFIGURASI GERAK
//gerak langkah
float max_step = 30; //mm

//gerak putar
float e_theta, e_alpha, e_beta; //sudut rotasi
float step_kanan, step_kiri, step_kanan_x, step_kanan_y, step_kiri_x, step_kiri_y ; //langkah kaki
float body_step, body_step_y, body_step_x, jumlah_step, jumlah_step_y, jumlah_step_x, step_gait, step_gait_y, step_gait_x ;
float arah_2;
float arah;

//koordinat IK sebelum (old)
float o_L1_x, o_L1_y, o_L1_z; //kaki L1
float o_L2_x, o_L2_y, o_L2_z; //kaki L2
float o_L3_x, o_L3_y, o_L3_z; //kaki L3

float o_R1_x, o_R1_y, o_R1_z; //kaki R1
float o_R2_x, o_R2_y, o_R2_z; //kaki R2
float o_R3_x, o_R3_y, o_R3_z; //kaki R3

//koordinat IK baru (new)
float n_L1_x, n_L1_y, n_L1_z; //kaki L1
float n_L2_x, n_L2_y, n_L2_z; //kaki L2
float n_L3_x, n_L3_y, n_L3_z; //kaki L3

float n_R1_x, n_R1_y, n_R1_z; //kaki R1
float n_R2_x, n_R2_y, n_R2_z; //kaki R2
float n_R3_x, n_R3_y, n_R3_z; //kaki R3

// R1
float array_px_R1[memori_data];
float array_py_R1[memori_data];
float array_pz_R1[memori_data];
// R2
float array_px_R2[memori_data];
float array_py_R2[memori_data];
float array_pz_R2[memori_data];
// R3
float array_px_R3[memori_data];
float array_py_R3[memori_data];
float array_pz_R3[memori_data];

// L1
float array_px_L1[memori_data];
float array_py_L1[memori_data];
float array_pz_L1[memori_data];
// L2
float array_px_L2[memori_data];
float array_py_L2[memori_data];
float array_pz_L2[memori_data];
// L2
float array_px_L3[memori_data];
float array_py_L3[memori_data];
float array_pz_L3[memori_data];

// variabel koordinat inverse kinematics (current position)
float L1_x, L1_y, L1_z; // kaki L1
float L2_x, L2_y, L2_z; // kaki L2
float L3_x, L3_y, L3_z; // kaki L3
float R1_x, R1_y, R1_z; // kaki R1
float R2_x, R2_y, R2_z; // kaki R2
float R3_x, R3_y, R3_z; // kaki R3

// float sudut; //digunakan untuk body statis
// float step_cal = 1.15; //perhitungan step gait
float titik_puncak_R1 = -30;    float titik_puncak_R2 = -30;    float titik_puncak_R3 = -30;   //titik puncak langkah -50
float titik_puncak_L1 = -30;    float titik_puncak_L2 = -30;    float titik_puncak_L3 = -30;   //titik puncak langkah -50

//float titik_puncak_R1 = -30;    float titik_puncak_R2 = -30;    float titik_puncak_R3 = -30;   //titik puncak langkah -50
//float titik_puncak_L1 = -30;    float titik_puncak_L2 = -30;    float titik_puncak_L3 = -30;   //titik puncak langkah -50
float x_awal, y_awal, z_awal, z_puncak;
float x_akhir, y_akhir, z_akhir;

//offset bodi (mm)
//kaki L1
float offset_L1_x = -48.73;
float offset_L1_y = 97.53;
float offset_L1_z = 0;
//kaki L2
float offset_L2_x = -82.8;
float offset_L2_y = 0;
float offset_L2_z = 0;
//kaki L3
float offset_L3_x = -48.73;
float offset_L3_y = -97.53;
float offset_L3_z = 0;
//kaki R1
float offset_R1_x = 48.73;
float offset_R1_y = 97.53;
float offset_R1_z = 0;
//kaki R2
float offset_R2_x = 82.8;
float offset_R2_y = 0;
float offset_R2_z = 0;
//kaki R3
float offset_R3_x = 48.73;
float offset_R3_y = -97.53;
float offset_R3_z = 0;

//posisi bodi (current position)
float body_x = 0, body_y = 0;

//sudut bodi (current position)
float body_theta = 0, body_alpha = 0, body_beta = 0;

//sudut arah body
float body_arah = 0;

//variabel koordinat kaki thdp bodi (current position)
float body_L1_x, body_L1_y, body_L1_z; //kaki L1
float body_L2_x, body_L2_y, body_L2_z; //kaki L2
float body_L3_x, body_L3_y, body_L3_z; //kaki L3

float body_R1_x, body_R1_y, body_R1_z; //kaki R1
float body_R2_x, body_R2_y, body_R2_z; //kaki R2
float body_R3_x, body_R3_y, body_R3_z; //kaki R2

//variabel koordinat baru, kaki thdp bodi (new)
float n_body_L1_x, n_body_L1_y, n_body_L1_z; //kaki L1
float n_body_L2_x, n_body_L2_y, n_body_L2_z; //kaki L2
float n_body_L3_x, n_body_L3_y, n_body_L3_z; //kaki L3

float n_body_R1_x, n_body_R1_y, n_body_R1_z; //kaki R1
float n_body_R2_x, n_body_R2_y, n_body_R2_z; //kaki R2
float n_body_R3_x, n_body_R3_y, n_body_R3_z; //kaki R3

//sensor hcsr04
float depan_kiri; float belakang_kiri;
float sensor_belakang; float belakang_kanan;
float depan_kanan; float sensor_depan;

int korban = 0;


//PID
unsigned long previousTime = 0;
unsigned long interval = 1; // interval waktu dalam milidetik
//Define Variables we'll be connecting to
float setpoint, error, delta_error, P, I, D, nilai_sensor,
      hasil_PID, error_sebelumnya;
//Specify the links and initial tuning parameters
float Kp = 0.2, Ki = 12, Kd = 0, T = 0.01;

bool robotBergerak = false;

//tof kanan kiri
int Sensor_1, Sensor_2, Sensor_3, Sensor_4;


void inverse_k(uint8_t dxl_id, float x, float y, float z, int kecepatan);
void polinomial_trj(uint32_t dxl_id, float xp1, float yp1, float zp1, float xp2, float yp2, float zp2, float xp3, float yp3, float zp3, float xp4, float yp4, float zp4);
void trj_lurus(uint32_t dxl_id, float x0, float y0, float z0, float x1, float y1, float z1);
void trj_langkah(uint32_t dxl_id, float x0, float y0, float z0, float x1, float y1, float zp);
void trj_start(uint32_t id_kakiL1, uint32_t id_kakiL2, uint32_t id_kakiL3, uint32_t id_kakiR1, uint32_t id_kakiR2, uint32_t id_kakiR3, int kecepatan, int dly_trj);
void siap();
void coba_majuv2();
void coba_mundurv4();
void mundur_robot();
void tangga_korban4();
void coba_maju1();
void tanggav2_korban4();
void kanan_robot();
void kiri_robot();
void putar_kiri();
void putar_kanan();
void coba_putar_kanan();
void coba_putar_kiri();
void coba_mundur();
void coba_maju2();
void siapv2();
void coba_maju();
void coba_putar_kananv2();
void coba_putar_kiriv2();
void siapv3();
void coba_majuv3();
void coba_majuv33();
void coba_majurintangan();
void coba_majurintangan1();

int dxl_comm_result = COMM_TX_FAIL;                                                  // Communication result
bool dxl_addparam_result = false;                                                    // addParam result

uint8_t param_goal_position[2]; uint8_t param_goal_speed[2];


#endif