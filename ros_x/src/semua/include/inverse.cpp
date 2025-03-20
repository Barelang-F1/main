#include "inverse_header.h"
// #include "icecream.hpp"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "cmath"
void setPosisiServo(uint8_t dxl_id, uint16_t posisi)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result;

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_AX_GOAL_POSITION, posisi, &dxl_error);
}
void setPosisiServoSync(uint8_t dxl_id1, uint16_t posisi1, uint8_t dxl_id2, uint16_t posisi2) {
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, 2);

    uint8_t param1[2] = {DXL_LOBYTE(posisi1), DXL_HIBYTE(posisi1)};
    uint8_t param2[2] = {DXL_LOBYTE(posisi2), DXL_HIBYTE(posisi2)};

    groupSyncWrite.addParam(dxl_id1, param1);
    groupSyncWrite.addParam(dxl_id2, param2);

    groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
}

void inverse_k(uint8_t dxl_id, float x, float y, float z, int kecepatan)
{

    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX);
    dynamixel::GroupSyncWrite groupSyncWriteSpeed(portHandler, packetHandler, ADDR_AX_GOAL_SPEED, LEN_AX);
    //  a. HITUNG THETA Coxa
    //  hitung theta c
    theta_c = radToServomx(atan2(y, x));
    x0 = sqrt((y * y) + (x * x));
    // b. HITUNG THETA Femur
    // hitung theta f1
    x1 = x0 - c; // pengurangan panjang x0 dan coxa
    theta_f1 = radToServoax(atan2(z, x1));
    // hitung panjang a
    a = sqrt((z * z) + (x1 * x1));
    // hitung f2
    theta_f2 = radToServoax(acos((pow(f, 2) + pow(a, 2) - pow(t, 2)) / (2 * a * f)));
    // hitung f
    theta_f = theta_f1 + theta_f2;
    // c. HITUNG THETA Tibia
    // hitung theta t
    theta_t = radToServoax(acos((pow(f, 2) + pow(t, 2) - pow(a, 2)) / (2 * f * t)) - 1.57);

    // d. Normalisasi 0 derajat servo
    if (dxl_id == kR1 || dxl_id == kR2 || dxl_id == kR3)
    {
        theta_c_real = 2048 - theta_c;
        theta_f_real = 512 - theta_f;
        theta_t_real = 512 + theta_t;
    }

    if (dxl_id == kL1 || dxl_id == kL2 || dxl_id == kL3)
    {
        theta_c_real = 2048 - theta_c;
        theta_f_real = 512 - theta_f;
        theta_t_real = 512 + theta_t;
    }

    switch (dxl_id)
    {
    case kR1:

        posisi_servo[0] = theta_t_real;
        posisi_servo[1] = theta_f_real;
        posisi_servo[2] = theta_c_real;

        speed_servo[0] = kecepatan;
        speed_servo[1] = kecepatan;
        speed_servo[2] = kecepatan;

        R1_x = x;
        R1_y = y;
        R1_z = z;
        break;
    case kR2:

        posisi_servo[3] = theta_t_real;
        posisi_servo[4] = theta_f_real;
        posisi_servo[5] = theta_c_real;

        speed_servo[3] = kecepatan;
        speed_servo[4] = kecepatan;
        speed_servo[5] = kecepatan;

        R2_x = x;
        R2_y = y;
        R2_z = z;
        break;
    case kR3:

        posisi_servo[6] = theta_t_real;
        posisi_servo[7] = theta_f_real;
        posisi_servo[8] = theta_c_real;

        speed_servo[6] = kecepatan;
        speed_servo[7] = kecepatan;
        speed_servo[8] = kecepatan;

        R3_x = x;
        R3_y = y;
        R3_z = z;
        break;
    case kL1:

        posisi_servo[9] = theta_t_real;
        posisi_servo[10] = theta_f_real;
        posisi_servo[11] = theta_c_real;

        speed_servo[9] = kecepatan;
        speed_servo[10] = kecepatan;
        speed_servo[11] = kecepatan;

        L1_x = x;
        L1_y = y;
        L1_z = z;
        break;
    case kL2:

        posisi_servo[12] = theta_t_real;
        posisi_servo[13] = theta_f_real;
        posisi_servo[14] = theta_c_real;

        speed_servo[12] = kecepatan;
        speed_servo[13] = kecepatan;
        speed_servo[14] = kecepatan;

        L2_x = x;
        L2_y = y;
        L2_z = z;
        break;
    case kL3:

        posisi_servo[15] = theta_t_real;
        posisi_servo[16] = theta_f_real;
        posisi_servo[17] = theta_c_real;

        speed_servo[15] = kecepatan;
        speed_servo[16] = kecepatan;
        speed_servo[17] = kecepatan;

        L3_x = x;
        L3_y = y;
        L3_z = z;
        break;
    }

    for (int i = 0; i < 18; i++)
    {

        // Allocate goal position value into byte array
        param_goal_position[0] = DXL_LOBYTE(posisi_servo[i]);
        param_goal_position[1] = DXL_HIBYTE(posisi_servo[i]);

        groupSyncWrite.addParam(dxl_idku[i], param_goal_position);
        
        groupSyncWrite.txPacket();
        // }
    }

    groupSyncWrite.clearParam();
}

void polinomial_trj(uint32_t dxl_id, float xp1, float yp1, float zp1, float xp2, float yp2, float zp2, float xp3, float yp3, float zp3, float xp4, float yp4, float zp4)
{
    float A, B, C, D; // utk perhitungan polinomial
    float px, py, pz; // hasil polinomial
    int nmr_data = 0; // no data array

    // hitung end point dengan polinomial

    for (float t = 0.0; t <= 1.009; t = t + iterasi)
    {
        // hitung polinomial
        A = pow((1 - t), 3);
        B = 3 * t * pow(1 - t, 2);
        C = 3 * pow(t, 2) * (1 - t);
        D = pow(t, 3);
        px = A * xp1 + B * xp2 + C * xp3 + D * xp4;
        py = A * yp1 + B * yp2 + C * yp3 + D * yp4;
        pz = A * zp1 + B * zp2 + C * zp3 + D * zp4;

        // simpan hasil perhitungan
        switch (dxl_id)
        {

        case kR1:
            array_px_R1[nmr_data] = px;
            array_py_R1[nmr_data] = py;
            array_pz_R1[nmr_data] = pz;
            break;
        case kR2:
            array_px_R2[nmr_data] = px;
            array_py_R2[nmr_data] = py;
            array_pz_R2[nmr_data] = pz;
            break;
        case kR3:
            array_px_R3[nmr_data] = px;
            array_py_R3[nmr_data] = py;
            array_pz_R3[nmr_data] = pz;
            break;
        case kL1:
            array_px_L1[nmr_data] = px;
            array_py_L1[nmr_data] = py;
            array_pz_L1[nmr_data] = pz;
            break;
        case kL2:
            array_px_L2[nmr_data] = px;
            array_py_L2[nmr_data] = py;
            array_pz_L2[nmr_data] = pz;
            break;
        case kL3:
            array_px_L3[nmr_data] = px;
            array_py_L3[nmr_data] = py;
            array_pz_L3[nmr_data] = pz;
            break;
        }

        nmr_data++;
    }

    jlh_data = nmr_data;
}

void trj_lurus(uint32_t dxl_id, float x0, float y0, float z0, float x1, float y1, float z1)
{
    float xp1, yp1, zp1; // titik vektor1
    float xp2, yp2, zp2; // titik vektor2
    float xp3, yp3, zp3; // titik vektor3
    float xp4, yp4, zp4; // titik vektor4

    // tentukan titik vektor polinomial
    xp1 = x0;
    yp1 = y0;
    zp1 = z0; // P1
    xp2 = x0;
    yp2 = y0;
    zp2 = z0; // P2
    xp3 = x1;
    yp3 = y1;
    zp3 = z1; // P3
    xp4 = x1;
    yp4 = y1;
    zp4 = z1; // P4
    polinomial_trj(dxl_id, xp1, yp1, zp1, xp2, yp2, zp2, xp3, yp3, zp3, xp4, yp4, zp4);
}

// trayektori langkah
void trj_langkah(uint32_t dxl_id, float x0, float y0, float z0, float x1, float y1, float zp)
{
    float xp1, yp1, zp1; // titik vektor1
    float xp2, yp2, zp2; // titik vektor2
    float xp3, yp3, zp3; // titik vektor3
    float xp4, yp4, zp4; // titik vektor4

    float z1;
    z1 = (zp - (0.25 * z0)) / 0.75;
    // tentukan titik vektor polinomial
    xp1 = x0;
    yp1 = y0;
    zp1 = z0; // P1
    xp2 = x0;
    yp2 = y0;
    zp2 = z1; // P2
    xp3 = x1;
    yp3 = y1;
    zp3 = z1; // P3
    xp4 = x1;
    yp4 = y1;
    zp4 = z0; // P4
    polinomial_trj(dxl_id, xp1, yp1, zp1, xp2, yp2, zp2, xp3, yp3, zp3, xp4, yp4, zp4);
}

// eksekusi trayektori
void trj_start(uint32_t id_kakiR1, uint32_t id_kakiR2, uint32_t id_kakiR3, uint32_t id_kakiL1, uint32_t id_kakiL2, uint32_t id_kakiL3, int kecepatan, int dly_trj)
{
    // Hitung hasil perhitungan tsb menggunakan IK
    for (int i = 0; i < jlh_data; i++)
    {

        if (id_kakiR1 == kR1)
        {
            inverse_k(id_kakiR1, array_px_R1[i], array_py_R1[i], array_pz_R1[i], kecepatan);
        }

        if (id_kakiR2 == kR2)
        {
            inverse_k(id_kakiR2, array_px_R2[i], array_py_R2[i], array_pz_R2[i], kecepatan);
        }

        if (id_kakiR3 == kR3)
        {
            inverse_k(id_kakiR3, array_px_R3[i], array_py_R3[i], array_pz_R3[i], kecepatan);
        }

        if (id_kakiL1 == kL1)
        {
            inverse_k(id_kakiL1, array_px_L1[i], array_py_L1[i], array_pz_L1[i], kecepatan);
        }

        if (id_kakiL2 == kL2)
        {
            inverse_k(id_kakiL2, array_px_L2[i], array_py_L2[i], array_pz_L2[i], kecepatan);
        }

        if (id_kakiL3 == kL3)
        {
            inverse_k(id_kakiL3, array_px_L3[i], array_py_L3[i], array_pz_L3[i], kecepatan);
        }

        usleep(dly_trj);
    }
}

void translasi_body(float pos_x, float pos_y, float pos_z , int kecepatan, int dly_trj) {

  //hitung posisi relatif thdp bodi
  kaki_to_body();

  //hitung koordinat selanjutnya
//kaki R1
  n_body_R1_x = body_R1_x + pos_x;
  n_body_R1_y = body_R1_y + pos_y;
  n_body_R1_z = body_R1_z + pos_z;
  //kaki R2
  n_body_R2_x = body_R2_x + pos_x;
  n_body_R2_y = body_R2_y + pos_y;
  n_body_R2_z = body_R2_z + pos_z;
  //kaki R3
  n_body_R3_x = body_R3_x + pos_x;
  n_body_R3_y = body_R3_y + pos_y;
  n_body_R3_z = body_R3_z + pos_z;
  //kaki L1
  n_body_L1_x = body_L1_x + pos_x;
  n_body_L1_y = body_L1_y + pos_y;
  n_body_L1_z = body_L1_z + pos_z;
  //kaki L2
  n_body_L2_x = body_L2_x + pos_x;
  n_body_L2_y = body_L2_y + pos_y;
  n_body_L2_z = body_L2_z + pos_z;
  //kaki L3
  n_body_L3_x = body_L3_x + pos_x;
  n_body_L3_y = body_L3_y + pos_y;
  n_body_L3_z = body_L3_z + pos_z;
  
  //kembalikan koordinat ke pusat coxa
  kaki_to_coxa();

  //gerakkan kaki (pos sekarang ke pos baru)
  
  trj_lurus(kR1, R1_x, R1_y, R1_z, n_R1_x, n_R1_y, n_R1_z);
  trj_lurus(kR2, R2_x, R2_y, R2_z, n_R2_x, n_R2_y, n_R2_z);
  trj_lurus(kR3, R3_x, R3_y, R3_z, n_R3_x, n_R3_y, n_R3_z);
  trj_lurus(kL1, L1_x, L1_y, L1_z, n_L1_x, n_L1_y, n_L1_z);
  trj_lurus(kL2, L2_x, L2_y, L2_z, n_L2_x, n_L2_y, n_L2_z);
  trj_lurus(kL3, L3_x, L3_y, L3_z, n_L3_x, n_L3_y, n_L3_z);
  trj_start(kR1, kR2, kR3, kL1, kL2, kL3, kecepatan, dly_trj);

}
//hitung koordinat kaki terhadap pusat body
void kaki_to_body() {
  //kaki R1
  body_R1_x = R1_x + offset_R1_x;
  body_R1_y = R1_y + offset_R1_y;
  body_R1_z = R1_z + offset_R1_z;
  //kaki R2
  body_R2_x = R2_x + offset_R2_x;
  body_R2_y = R2_y + offset_R2_y;
  body_R2_z = R2_z + offset_R2_z;
  //kaki R3
  body_R3_x = R3_x + offset_R3_x;
  body_R3_y = R3_y + offset_R3_y;
  body_R3_z = R3_z + offset_R3_z;
  //kaki L1
  body_L1_x = L1_x + offset_L1_x;
  body_L1_y = L1_y + offset_L1_y;
  body_L1_z = L1_z + offset_L1_z;
  //kaki L2
  body_L2_x = L2_x + offset_L2_x;
  body_L2_y = L2_y + offset_L2_y;
  body_L2_z = L2_z + offset_L2_z;
  //kaki L3
  body_L3_x = L3_x + offset_L3_x;
  body_L3_y = L3_y + offset_L3_y;
  body_L3_z = L3_z + offset_L3_z;
  
}
//hitung koordinat kaki terhadap pusat coxa (Inverse kinematics)
void kaki_to_coxa() {
  //koordinat kaki baru
  //kaki R1
  n_R1_x = n_body_R1_x - offset_R1_x;
  n_R1_y = n_body_R1_y - offset_R1_y;
  n_R1_z = n_body_R1_z - offset_R1_z;
  //kaki R2
  n_R2_x = n_body_R2_x - offset_R2_x;
  n_R2_y = n_body_R2_y - offset_R2_y;
  n_R2_z = n_body_R2_z - offset_R2_z;
  //kaki R3
  n_R3_x = n_body_R3_x - offset_R3_x;
  n_R3_y = n_body_R3_y - offset_R3_y;
  n_R3_z = n_body_R3_z - offset_R3_z;
  //kaki L1
  n_L1_x = n_body_L1_x - offset_L1_x;
  n_L1_y = n_body_L1_y - offset_L1_y;
  n_L1_z = n_body_L1_z - offset_L1_z;
  //kaki L2
  n_L2_x = n_body_L2_x - offset_L2_x;
  n_L2_y = n_body_L2_y - offset_L2_y;
  n_L2_z = n_body_L2_z - offset_L2_z;
  //kaki L3
  n_L3_x = n_body_L3_x - offset_L3_x;
  n_L3_y = n_body_L3_y - offset_L3_y;
  n_L3_z = n_body_L3_z - offset_L3_z;
  
}

void siap()
{
    trj_lurus(kR1, 80, 0, -60, 80, 0, -60);
    trj_lurus(kR2, 80, 0, -60, 80, 0, -60);
    trj_lurus(kR3, 80, 0, -60, 80, 0, -60);

    trj_lurus(kL1, 80, 0, -60, 80, 0, -60);
    trj_lurus(kL2, 80, 0, -60, 80, 0, -60);
    trj_lurus(kL3, 80, 0, -60, 80, 0, -60);

    // trj_lurus(R1, 40, 0, -60, 40, 0, -60);
    // trj_lurus(R2, 40, 0, -60, 40, 0, -60);
    // trj_lurus(R3, 40, 0, -60, 40, 0, -60);

    // trj_lurus(L1, 40, 0, -60, 40, 0, -60);
    // trj_lurus(L2, 40, 0, -60, 40, 0, -60);
    // trj_lurus(L3, 40, 0, -60, 40, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
    // trj_start(xx, xx, xx, xx, xx, xx,  ultra_fast, 15000);
}


void siap_angkat_korban()
{
    trj_lurus(kR1, 80, 15, -60, 80, 15, -60);
    trj_lurus(kR2, 80, -40, -60, 80, -40, -60);
    trj_lurus(kR3, 80, -15, -60, 80, -15, -60);

    trj_lurus(kL1, 80, -15, -60, 80, -15, -60);
    trj_lurus(kL2, 80, 40, -60, 80, 40, -60);
    trj_lurus(kL3, 80, 15, -60, 80, 15, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 12000);
    // trj_start(xx, xx, R3, xx, xx, xx,  ultra_fast, 25000);
}
void siap_tangga()
{
    trj_lurus(kR1, 80, 0, -100, 80, 0, -100);
    trj_lurus(kR2, 80, 0, -100, 80, 0, -100);
    trj_lurus(kR3, 100, 0, -80, 100, 0, -80);

    trj_lurus(kL1, 70, 0, -60, 70, 0, -50);
    trj_lurus(kL2, 70, 0, -60, 70, 0, -50);
    trj_lurus(kL3, 60, 0, -60, 60, 0, -50);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 10000);
}
void coba_putar_kananv2()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, -15, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -15, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, -15, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -80); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -80); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -15, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -15, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, -15, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}
void putar_kanan_korban()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, -10, -30);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -10, -30);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, -10, -30);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -60); // 60
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -60); // 60
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -10, -30);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -10, -30);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, -10, -30);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -60); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -60); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);
}
void putar_kanan_korban34()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, -10, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -10, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, -10, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -80); // 80
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -80); // 80
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -10, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -10, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, -10, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);
}
void coba_putar_kananv3()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, -15, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -15, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, -15, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -90); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -90); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -15, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -15, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, -15, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -90); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -90); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);
}
void coba_putar_kiriv2()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 15, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 15, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 15, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -80); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -80); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 15, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 15, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 15, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}
void putar_kiri_korban()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 10, -30);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 10, -30);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 10, -30);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -60); // 60
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -60); // 60
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 10, -30);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 10, -30);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 10, -30);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -60); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -60); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);
}
void putar_kiri_korban34()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 10, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 10, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 10, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -80); // 80
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -80); // 80
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 10, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 10, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 10, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);
}
void coba_putar_kiriv3()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 25, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 25, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 25, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -90); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -90); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 25, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 25, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 25, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -90); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -90); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);
}
void siapv2()
{
    trj_lurus(kR1, 80, -60, -60, 80, -60, -60);
    trj_lurus(kR2, 80, 0, -60, 80, 0, -60);
    trj_lurus(kR3, 80, 60, -60, 80, 60, -60);

    trj_lurus(kL1, 80, 60, -60, 80, 60, -60);
    trj_lurus(kL2, 80, 0, -60, 80, 0, -60);
    trj_lurus(kL3, 80, -60, -60, 80, -60, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 25000);
}
void siapv3()
{
    trj_lurus(kR1, 80, -10, -80, 80, -10, -80);
    trj_lurus(kR2, 80, 0, -80, 80, 0, -80);
    trj_lurus(kR3, 80, 40, -80, 80, 40, -80);

    trj_lurus(kL1, 80, 10, -80, 80, 10, -80);
    trj_lurus(kL2, 80, 0, -80, 80, 0, -80);
    trj_lurus(kL3, 80, -40, -80, 80, -40, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 25000);
}
void coba_majuv3()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, -20, -30);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -20, -30);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 15, -30);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -60); // 80
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -60); // 80
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 12000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 20, -30);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 20, -30);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, -15, -30);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -60); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -60); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 12000);
}
void coba_majuv32()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 100, -20, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -20, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 10, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -80); // 80
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -80); // 80
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 25000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 100, 20, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 20, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, -10, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 25000);
}
void coba_majuv()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, -20, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -20, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 15, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -80); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -80); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 20, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 20, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, -15, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);
}
void coba_majuv33()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 90, -20, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -40, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 40, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 90, -20, -90); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -80);   // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 40000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 90, 10, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 40, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, -40, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 90, 20, -90); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80);  // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 40000);
}
void coba_mundurv3()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 30, -30);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 0, -30);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, -15, -30);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, -10, -60); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 40, -60);  // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -30, -30);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 0, -30);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 15, -30);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 10, -60);  // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, -40, -60); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}
void mundurrv32()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 20, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 0, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, -5, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, -10, -80); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 30, -80);  // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -20, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 0, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 5, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 10, -80);  // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, -30, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}
void coba_mundurv4()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 30, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 0, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, -15, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, -10, -80); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 40, -80);  // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -30, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 0, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 15, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 10, -80);  // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, -40, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}
void coba_putar_kanan_tanggav2()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, -25, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -25, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, -25, -50);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -100); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -100); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -50);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 30000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -25, -60);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -25, -60);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, -25, -30);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -50); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -50); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 30000);
}
void coba_putar_kiri_tanggav2()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 25, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 25, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 25, -50);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -100); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -100); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -50);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 30000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 25, -60);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 25, -60);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 25, -30);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -50); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -50); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 30000);
}
void coba_majuv2()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 25, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -25, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 15, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, -40, -60); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 10, -60);  // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -25, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 25, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, -15, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 40, -60);  // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, -10, -60); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);
}

void coba_maju()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, -35, -30);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 35, -30);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 120, 0, -30);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 35, -60);  // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, -35, -60); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 120, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 12000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -35, -30);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 35, -30);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 70, 0, -30);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -60); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -60); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 70, 0, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 12000);
}
void coba_maju2()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, -35, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 35, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 120, 0, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 35, -80);  // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, -35, -80); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 120, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 25000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -35, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 35, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 70, 0, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 70, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 25000);
}
void maju_korban5()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 100, -20, -30);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 0, -30);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 0, -30);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -60);  // 60
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, -20, -60); // 60
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 25, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 25000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 100, 20, -30);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 0, -30);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 0, -30);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -60); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 20, -60); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, -25, -60);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 25000);
}
void maju_korban3dan4()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 100, -20, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 0, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 0, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -80);  // 80
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, -20, -80); // 80
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 25, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 25000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 100, 20, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 0, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 0, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 20, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, -25, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 25000);
}
void coba_maju1()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 15, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -15, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 70, 0, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 10, -80); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 10, -80); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 70, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 20, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -20, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 120, 0, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 120, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 18000);
}
void coba_maju3()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 15, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -15, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 70, 0, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 10, -80); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 10, -80); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 70, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 20, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -20, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 100, 0, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 115, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);
    // // kanan
    // trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 15, -40);
    // trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -15, -40);
    // trj_langkah(kR2, R2_x, R2_y, R2_z, 70, 0, -40);

    // trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 10, -80); // 90
    // trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 10, -80); // 90
    // trj_lurus(kL2, L2_x, L2_y, L2_z, 70, 0, -80);
    // trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);

    // trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 20, -40);
    // trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -20, -40);
    // trj_langkah(kL2, L2_x, L2_y, L2_z, 120, 0, -40);

    // trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    // trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    // trj_lurus(kR2, R2_x, R2_y, R2_z, 120, 0, -80);
    // trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);
}
void coba_majurintangan()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, -40, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 40, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 120, 0, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 35, -80);  // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, -35, -80); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 120, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, -40, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 40, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 70, 0, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 70, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);
}
void coba_majurintangan1()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 90, -30, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, 30, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 0, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 90, 35, -80);  // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, -35, -80); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 90, -30, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 30, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 80, 0, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 90, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 20000);
}
void coba_mundur()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 80, 35, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 80, -35, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 60, 0, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -80); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -80); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 35000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 35, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -35, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 120, 0, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 80, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 80, 0, -80); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 80, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 35000);
}

void mundur_robot()
{
    // MUNDUR //belum buat
    trj_langkah(kL1, L1_x, L1_y, L1_z, 40, 10, -60);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 40, 10, -60);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 40, -10, -60);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 40, 0, -95); // 90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 40, 0, -70); // 90
    trj_lurus(kL2, L2_x, L2_y, L2_z, 40, 0, -90);

    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 40, -10, -60);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 40, -10, -60);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 40, 10, -60);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 40, 0, -90); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 40, 0, -90); // 100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 40, 0, -90);

    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}

void kiri_robot()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 30, 0, -60);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 30, 0, -60);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 50, 0, -60);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 40, 0, -80); // 40
    trj_lurus(kR3, R3_x, R3_y, R3_z, 40, 0, -80); // 40
    trj_lurus(kL2, L2_x, L2_y, L2_z, 40, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 50, 0, -60);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 50, 0, -60);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 30, 0, -60);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 40, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 40, 0, -80); // 00
    trj_lurus(kR2, R2_x, R2_y, R2_z, 40, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}

void kanan_robot()
{
    // kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 30, 0, -60);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 30, 0, -60);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 50, 0, -60);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 40, 0, -80); // 40
    trj_lurus(kR3, R3_x, R3_y, R3_z, 40, 0, -80); // 40
    trj_lurus(kL2, L2_x, L2_y, L2_z, 40, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 50, 0, -60);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 50, 0, -60);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 30, 0, -60);

    trj_lurus(kL1, L1_x, L1_y, L1_z, 40, 0, -80); // 95
    trj_lurus(kL3, L3_x, L3_y, L3_z, 40, 0, -80); // 00
    trj_lurus(kR2, R2_x, R2_y, R2_z, 40, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}

void coba_putar_kanan()
{

    // langkah kaki kiri
    trj_langkah(kL1, L1_x, L1_y, L1_z, 40, -15, -75); // x0,y0,x1,y1,z0,zp
    trj_langkah(kL3, L3_x, L3_y, L3_z, 40, -15, -70);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 40, -15, -70);
    // geser kaki kanan
    trj_lurus(kR1, R1_x, R1_y, R1_z, 40, 0, -95);
    trj_lurus(kR3, R3_x, R3_y, R3_z, 40, 0, -70);
    trj_lurus(kL2, L2_x, L2_y, L2_z, 40, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
    // langkah kaki kanan
    trj_langkah(kR1, R1_x, R1_y, R1_z, 40, -15, -70);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 40, -15, -50);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 40, -15, -70);
    // geser kaki kanan
    trj_lurus(kL1, L1_x, L1_y, L1_z, 40, 0, -90);
    trj_lurus(kL3, L3_x, L3_y, L3_z, 40, 0, -90);
    trj_lurus(kR2, R2_x, R2_y, R2_z, 40, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}
void putar_kanan()
{

    // langkah kaki kiri
    trj_langkah(kL1, L1_x, L1_y, L1_z, -40, 15, -75); // x0,y0,x1,y1,z0,zp
    trj_langkah(kL3, L3_x, L3_y, L3_z, -40, 15, -70);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 40, -15, -70);
    // geser kaki kanan
    trj_lurus(kR1, R1_x, R1_y, R1_z, 40, 0, -95);
    trj_lurus(kR3, R3_x, R3_y, R3_z, 40, 0, -70);
    trj_lurus(kL2, L2_x, L2_y, L2_z, -40, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
    // langkah kaki kanan
    trj_langkah(kR1, R1_x, R1_y, R1_z, 40, -15, -70);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 40, -15, -50);
    trj_langkah(kL2, L2_x, L2_y, L2_z, -40, 15, -70);
    // geser kaki kanan
    trj_lurus(kL1, L1_x, L1_y, L1_z, -40, 0, -90);
    trj_lurus(kL3, L3_x, L3_y, L3_z, -40, 0, -90);
    trj_lurus(kR2, R2_x, R2_y, R2_z, 40, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}
void coba_putar_kiri()
{
    // langkah kaki kiri
    trj_langkah(kL1, L1_x, L1_y, L1_z, 40, 15, -70); // x0,y0,x1,y1,z0,zp
    trj_langkah(kL3, L3_x, L3_y, L3_z, 40, 15, -70);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 40, 15, -70);
    // geser kaki kanan
    trj_lurus(kR1, R1_x, R1_y, R1_z, 40, 0, -95);
    trj_lurus(kR3, R3_x, R3_y, R3_z, 40, 0, -70);
    trj_lurus(kL2, L2_x, L2_y, L2_z, 40, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
    // langkah kaki kanan
    trj_langkah(kR1, R1_x, R1_y, R1_z, 40, 15, -75);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 40, 15, -50);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 40, 15, -70);
    // geser kaki kanan
    trj_lurus(kL1, L1_x, L1_y, L1_z, 40, 0, -90);
    trj_lurus(kL3, L3_x, L3_y, L3_z, 40, 0, -90);
    trj_lurus(kR2, R2_x, R2_y, R2_z, 40, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}
void putar_kiri()
{
    // langkah kaki kiri
    trj_langkah(kL1, L1_x, L1_y, L1_z, -40, -15, -70); // x0,y0,x1,y1,z0,zp
    trj_langkah(kL3, L3_x, L3_y, L3_z, -40, -15, -70);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 40, 15, -70);
    // geser kaki kanan
    trj_lurus(kR1, R1_x, R1_y, R1_z, 40, 0, -95);
    trj_lurus(kR3, R3_x, R3_y, R3_z, 40, 0, -70);
    trj_lurus(kL2, L2_x, L2_y, L2_z, -40, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
    // langkah kaki kanan
    trj_langkah(kR1, R1_x, R1_y, R1_z, 40, 15, -75);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 40, 15, -50);
    trj_langkah(kL2, L2_x, L2_y, L2_z, -40, -15, -70);
    // geser kaki kanan
    trj_lurus(kL1, L1_x, L1_y, L1_z, -40, 0, -90);
    trj_lurus(kL3, L3_x, L3_y, L3_z, -40, 0, -90);
    trj_lurus(kR2, R2_x, R2_y, R2_z, 40, 0, -90);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 15000);
}
void tanggav1()
{

    // langkah kaki kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 95, 30, -10);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 95, -30, -10);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 0, -30); // 10
    // geser kaki kanan
    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, -30, -100);
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 30, -100);
    trj_lurus(kL2, L2_x, L2_y, L2_z, 60, 0, -45);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 30000);

    // langkah kaki kiri
    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 10, -40); // x0,y0,x1,y1,z0,zp
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -10, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 100, 0, -10); // 25
    // geser kaki kanan
    trj_lurus(kL1, L1_x, L1_y, L1_z, 70, 30, -45);
    trj_lurus(kL3, L3_x, L3_y, L3_z, 70, -30, -45);
    trj_lurus(kR2, R2_x, R2_y, R2_z, 100, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 30000);
}
void tanggav2()
{

    // langkah kaki kanan
    trj_langkah(kL1, L1_x, L1_y, L1_z, 95, 30, -10);
    trj_langkah(kL3, L3_x, L3_y, L3_z, 95, -30, -10);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 80, 0, -30); // 10
    // geser kaki kanan
    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, -30, -100);
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 30, -100);
    trj_lurus(kL2, L2_x, L2_y, L2_z, 60, 0, -50);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 30000);

    // langkah kaki kiri
    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 10, -40); // x0,y0,x1,y1,z0,zp
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, -10, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, 100, 0, -10); // 25
    // geser kaki kanan
    trj_lurus(kL1, L1_x, L1_y, L1_z, 70, 30, -50);
    trj_lurus(kL3, L3_x, L3_y, L3_z, 70, -30, -50);
    trj_lurus(kR2, R2_x, R2_y, R2_z, 100, 0, -80);
    trj_start(kR1, kR2, kR3, kL1, kL2, kL3, ultra_fast, 30000);
}

void tangga_korban4()
{
    // langkah kaki kiri
    trj_langkah(kL1, L1_x, L1_y, L1_z, -80, 0, -15); // x0,y0,x1,y1,z0,zp
    trj_langkah(kL3, L3_x, L3_y, L3_z, -80, 0, -15);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 15, 0, -25); // 25
    // geser kaki kanan
    trj_lurus(kR1, R1_x, R1_y, R1_z, 80, 0, -105);
    trj_lurus(kR3, R3_x, R3_y, R3_z, 80, 0, -105);
    trj_lurus(kL2, L2_x, L2_y, L2_z, -15, 0, -85);
    trj_start(kL1, kL2, kL3, kR1, kR2, kR3, med, 15);
    // langkah kaki kanan
    trj_langkah(kR1, R1_x, R1_y, R1_z, 25, 0, -35);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 25, 0, -35);
    trj_langkah(kL2, L2_x, L2_y, L2_z, -80, 0, -10); // 10
    // geser kaki kanan
    trj_lurus(kL1, L1_x, L1_y, L1_z, -25, 0, -85);
    trj_lurus(kL3, L3_x, L3_y, L3_z, -25, 0, -85);
    trj_lurus(kR2, R2_x, R2_y, R2_z, 65, 0, -105);
    trj_start(kL1, kL2, kL3, kR1, kR2, kR3, med, 15);
}

void tanggav2_korban4()
{
    // langkah kaki kiri
    trj_langkah(kL1, L1_x, L1_y, L1_z, -25, 0, -15); // x0,y0,x1,y1,z0,zp
    trj_langkah(kL3, L3_x, L3_y, L3_z, -25, 0, -15);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 65, 0, -25); // 25
    // geser kaki kanan
    trj_lurus(kR1, R1_x, R1_y, R1_z, 15, 0, -105);
    trj_lurus(kR3, R3_x, R3_y, R3_z, 15, 0, -105);
    trj_lurus(kL2, L2_x, L2_y, L2_z, -80, 0, -85);
    trj_start(kL1, kL2, kL3, kR1, kR2, kR3, med, 15);
    // langkah kaki kanan
    trj_langkah(kR1, R1_x, R1_y, R1_z, 80, 0, -35);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 80, 0, -35);
    trj_langkah(kL2, L2_x, L2_y, L2_z, -25, 0, -10); // 10
    // geser kaki kanan
    trj_lurus(kL1, L1_x, L1_y, L1_z, -80, 0, -85);
    trj_lurus(kL3, L3_x, L3_y, L3_z, -80, 0, -85);
    trj_lurus(kR2, R2_x, R2_y, R2_z, 25, 0, -105);
    trj_start(kL1, kL2, kL3, kR1, kR2, kR3, med, 15);
}
void imupid_jalanbiasa()
{

    trj_langkah(kL1, L1_x, L1_y, L1_z, -40, L1_y - hasil_PID, -70);
    trj_langkah(kL3, L3_x, L3_y, L3_z, -40, L3_y - hasil_PID, -70);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 40, R2_y + hasil_PID, -70);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 40, 0, -95);  //- hasil_PID //90
    trj_lurus(kR3, R3_x, R3_y, R3_z, 40, 0, -70);  // - hasil_PID //90
    trj_lurus(kL2, L2_x, L2_y, L2_z, -40, 0, -90); //+ hasil_PID
    trj_start(kL1, kL2, kL3, kR1, kR2, kR3, ultra_fast, 20000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 40, R1_y + hasil_PID, -60);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 40, R3_y + hasil_PID, -70);
    trj_langkah(kL2, L2_x, L2_y, L2_z, -40, L2_y - hasil_PID, -70);

    trj_lurus(kL1, L1_x, L1_y, L1_z, -40, 0, -90); //+ hasil_PID //95
    trj_lurus(kL3, L3_x, L3_y, L3_z, -40, 0, -90); //+ hasil_PID //100
    trj_lurus(kR2, R2_x, R2_y, R2_z, 40, 0, -90);  //- hasil_PID
    trj_start(kL1, kL2, kL3, kR1, kR2, kR3, ultra_fast, 20000);
}

void imupid_jalanrintangan()
{

    trj_langkah(kL1, L1_x, L1_y, L1_z, -40, L1_y - hasil_PID, -40);
    trj_langkah(kL3, L3_x, L3_y, L3_z, -40, L3_y - hasil_PID, -40);
    trj_langkah(kR2, R2_x, R2_y, R2_z, 40, R2_y + hasil_PID, -40);

    trj_lurus(kR1, R1_x, R1_y, R1_z, 40, 0, -100);  //- hasil_PID
    trj_lurus(kR3, R3_x, R3_y, R3_z, 40, 0, -100);  //- hasil_PID
    trj_lurus(kL2, L2_x, L2_y, L2_z, -40, 0, -100); //+ hasil_PID
    trj_start(kL1, kL2, kL3, kR1, kR2, kR3, ultra_fast, 20000);

    trj_langkah(kR1, R1_x, R1_y, R1_z, 40, R1_y + hasil_PID, -40);
    trj_langkah(kR3, R3_x, R3_y, R3_z, 40, R3_y + hasil_PID, -40);
    trj_langkah(kL2, L2_x, L2_y, L2_z, -40, L2_y - hasil_PID, -40);

    trj_lurus(kL1, L1_x, L1_y, L1_z, -40, 0, -100); // + hasil_PID
    trj_lurus(kL3, L3_x, L3_y, L3_z, -40, 0, -110); // + hasil_PID
    trj_lurus(kR2, R2_x, R2_y, R2_z, 40, 0, -100);  //- hasil_PID
    trj_start(kL1, kL2, kL3, kR1, kR2, kR3, ultra_fast, 20000);
}