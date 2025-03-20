#include "../include/inverse.cpp"
#include "ros/ros.h"
#include <JetsonGPIO.h>
#include "std_msgs/String.h"
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/UInt16.h"           // Untuk pesan jarak
#include <std_msgs/UInt16MultiArray.h> // Menggunakan UInt16MultiArray
#include <math.h>
#include "std_msgs/Float32.h"
#include <csignal>
#include <thread>
#include <atomic>
#include <chrono>
#define BUT_PIN 6 // Pin 37 adalah GPIO26
// #define STOP_PIN 12               // Pin 32 adalah GPIO12
bool but = false;                 // Variabel untuk status tombol
bool stop_but = false;            // Variabel untuk status tombol stop
int button_press_count = 0;       // Penghitung tekanan tombol
ros::Time last_button_time;       // Waktu terakhir tombol ditekan
ros::Duration debounce_time(0.2); // Debounce selama 300ms
bool program_running = false;     // Status program berjalan
int step = 0;
int itung = 0;
float centerX = 0;
float centerY = 0;
float confidence = 0.0;
bool objek_terdeteksi = false;
bool case_aktif[11] = {true, false, false, false, false, false, false, false, false, false, false}; // Status awal: hanya case 0 aktif
// Variabel global untuk menyimpan perintah aktif dan data IMU
std::string current_command = "";
float imu_yaw = 0.0, imu_roll = 0.0, imu_pitch = 0.0;

ros::Publisher angle_pub[3];
// std::atomic<bool> running(true);
std::atomic<int> angles[3];

int mulai()
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result

    uint8_t dxl_error = 0; // Dynamixel error
    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        return 0;
    }

    // IC();
    for (int i = 0; i < 20; i++)
    {
        // Enable Dynamixel#1 Torque
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_idku[i], ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully connected \n", dxl_idku[i]);
        }
    }

    setPosisiServoSync(19, 794, 20, 193);
    // setPosisiServoSync(19, 794);
    // setPosisiServoSync(20, 193);
    siap();
}

void maju()
{
    printf("Robot bergerak maju\n");
    coba_majurintangan();
}
void maju_kiri()
{
    coba_maju1();
    printf("Robot bergerak maju\n");
}
void berhenti_robot()
{
    // printf("Robot berhenti untuk mempertahankan heading\n");
    siap();
}
void kiri_robot1()
{
    // printf("Robot berhenti untuk mempertahankan heading\n");
    coba_mundur();
}
void siaptangga()
{
    siap_tangga();
}
void kanan_robot1()
{
    // printf("Robot berputar ke kanan untuk mengoreksi heading\n");
    coba_maju();
}
void kanann_robot2()
{
    printf("Robot berputar ke kanan untuk mengoreksi heading\n");
    coba_maju2();
}
void maju_robot2()
{
    printf("Robot berputar ke kanan untuk mengoreksi heading\n");
    coba_majuv33();
}
void putar_kanan_robot1()
{
    printf("Robot berputar ke kanan untuk mengoreksi heading\n");
    coba_putar_kananv2();
}
void putar_kanan_robot()
{
    printf("Robot berputar ke kanan untuk mengoreksi heading\n");
    coba_putar_kananv3();
}
void maju_robot1()
{
    printf("Robot berputar ke kiri untuk mengoreksi heading\n");
    coba_majuv3();
}
void majuu_korban32()
{
    coba_majuv32();
}

void mundur_korban32()
{
    mundurrv32();
}
void maju_korban34()
{
    coba_maju3();
}
void maju_tinggi()
{
    coba_majuv();
}
void putar_kiri_robot1()
{
    printf("Robot berputar ke kanan untuk mengoreksi heading\n");
    coba_putar_kiriv2();
}
void putar_kiri_robot()
{
    printf("Robot berputar ke kanan untuk mengoreksi heading\n");
    coba_putar_kiriv3();
}
void putar_kanankorban()
{
    putar_kanan_korban();
}
void putar_kanankorban3()
{
    putar_kanan_korban34();
}
void putar_kirikorban()
{
    putar_kiri_korban();
}
void putar_kirikorban3()
{
    putar_kiri_korban34();
}
void rintangan_1()
{
    coba_majurintangan1();
}
void mundur_v4()
{
    coba_mundurv4();
}
void naik_tanggav1()
{
    tanggav1();
}
void naik_tangga()
{
    tanggav2();
}
void otw_angkat_korban()
{
    siap_angkat_korban();
}
void korban_5()
{
    maju_korban5();
}
void korbannn_34()
{
    maju_korban3dan4();
}

void publishServoAngle(int id, float angle)
{
    std_msgs::Float32 msg;
    msg.data = angle;
    angle_pub[id].publish(msg);
    // ROS_INFO("Servo %d sudut dipublikasikan: %f", id + 1, msg.data);
}

void initGPIO()
{
    GPIO::setmode(GPIO::BCM);       // Gunakan nomor pin BOARD
    GPIO::setup(BUT_PIN, GPIO::IN); // Konfigurasi pin 6 sebagai input
    // GPIO::setup(STOP_PIN, GPIO::IN); // Konfigurasi pin 12 sebagai input
}

void updateButtonStatus()
{
    ros::Time current_time = ros::Time::now();

    if (GPIO::input(BUT_PIN) == GPIO::LOW) // LOW artinya tombol ditekan
    {
        if ((current_time - last_button_time) > debounce_time) // Cek debounce
        {
            last_button_time = current_time;

            program_running = !program_running; // Toggle status program_running

            if (program_running) // Jika program berjalan, mulai dari awal
            {
                printf("Tombol ditekan. Robot mulai bergerak dari awal\n");
                step = 0;
                setPosisiServoSync(19, 794, 20, 193);
                siap();
                but = true;
                stop_but = false;
            }
            else // Jika program berhenti
            {
                printf("Tombol ditekan. Robot berhenti\n");
                but = false;
                stop_but = true;
                setPosisiServoSync(19, 794, 20, 193);
                siap();
            }
        }
    }
}

void imuCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (msg->data.size() >= 3)
    {
        imu_yaw = msg->data[0];
        imu_roll = msg->data[1];
        imu_pitch = msg->data[2];
    }
}
void distanceCallback(const std_msgs::UInt16MultiArray::ConstPtr &msg)
{
    if (msg->data.size() >= 4)
    {
        Sensor_1 = msg->data[0];
        Sensor_2 = msg->data[1];
        Sensor_3 = msg->data[2];
        Sensor_4 = msg->data[3];
    }
}
void prosesLogika()
{

    if (!program_running)
    {
        berhenti_robot();
        return;
    }
    switch (step)
    {
    case 0:
        if (but)
        {

            step = 1;
            printf("Tombol ditekan, robot mulai case1\n");
            case_aktif[0] = false; // Matikan case 0
            case_aktif[1] = true;  // Aktifkan case 1
        }
        publishServoAngle(0, 0);
        usleep(500000);
        break;

    case 1:
        coba_maju();
        if (Sensor_1 > 390)
        {
            step = 2;
            printf("Sensor_1 > 360, robot bergerak case2\n");
            case_aktif[1] = false; // Matikan case 1
            case_aktif[2] = true;  // Aktifkan case 2
        }
        break;

    case 2:
        coba_mundurv3();
        if (Sensor_3 <= 85)
        {
            step = 3;
            printf("Sensor_4 > 218, robot bergerak case3\n");
            case_aktif[2] = false; // Matikan case 3
            case_aktif[3] = true;  // Aktifkan case 4
        }
        break;
    case 3:
        if (centerX == 0)
        {
            siap();
            printf("Robot diam\n");
        }
        else if (centerX < 320)
        {
            putar_kiri_korban();
            printf("Robot kekiri\n");
        }
        else if (centerX > 480)
        {
            putar_kanan_korban();
            printf("Robot kanan\n");
        }
        else
        {

            coba_majuv3();
            printf("Robot maju\n");
        }
        if (Sensor_3 > 80 && Sensor_3 < 120)
        {
            printf("Sensor_2 dalam rentang 150-170 case6\n");
            step = 4;
        }
        break;
    case 4:
        static ros::Time start_time_5; // Waktu awal untuk delay

        // Set nilai servo ke 45 dan publish
        publishServoAngle(0, 30);
        usleep(500000); // Tunggu sebentar untuk memastikan servo mulai bergerak
        setPosisiServoSync(19, 249, 20, 235);
        // setPosisiServoSync(19, 249);
        // setPosisiServoSync(20, 235);
        // siap_angkat_korban(); // Jalankan fungsi utama

        printf("Robot berada dalam posisi case 5\n");

        // Inisialisasi waktu saat pertama kali masuk case 4
        if (start_time_5.isZero())
            start_time_5 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_5).toSec() >= 1.0)
        {
            step = 5;                    // Pindah ke case 5
            start_time_5 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke case 6\n");
        }
        break;

    case 5:
        for (int i = 0; i < 5; i++)
        {
            coba_majuv3();
            // usleep(500000); // Beri waktu agar gerakan maju selesai sebelum iterasi berikutnya
        }

        usleep(500000);
        publishServoAngle(0, 3);
        usleep(100000);
        for (int i = 0; i < 4; i++)
        {
            coba_mundurv3();
            // usleep(500000); // Beri waktu agar gerakan maju selesai sebelum iterasi berikutnya
        }
        // usleep(500000);
        // coba_mundurv3();
        printf("Robot mundur setelah dapat korban\n");
        step = 42;
        // if (centerY >= 230 && centerY <= 240)
        // {
        //     step = 42;
        // }
        break;
    case 42:
        static ros::Time start_time_11; // Waktu awal untuk delay
        setPosisiServoSync(19, 794, 20, 193);
        // setPosisiServoSync(19, 794);
        // setPosisiServoSync(20, 193);
        printf("Robot berada dalam posisi case10\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_11.isZero())
            start_time_11 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_11).toSec() >= 1.0)
        {
            step = 6;                     // Pindah ke gerakan berikutnya
            start_time_11 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        break;
    case 6:
        // Loop sampai roll berada dalam rentang yang diinginkan
        if (imu_yaw >= 90 && imu_yaw <= 250)
        {
            printf("Koreksi arah: Robot berputar ke kanancase4\n");
            coba_putar_kananv3();
        }
        else if ((imu_yaw >= 289 && imu_yaw <= 360) || (imu_yaw >= 0 && imu_yaw <= 30))
        {
            printf("Koreksi arah: Robot berputar ke kiricase4\n");
            coba_putar_kiriv3();
        }
        else
        {
            printf("Robot maju rintangan dengan arah luruscase4\n");
            coba_majurintangan();
        }
        // Periksa roll dan tetap berada di case 4 sampai nilai roll berada dalam rentang yang ditentukan
        if ((imu_roll >= 235 && imu_roll <= 250) || (imu_roll >= 0 && imu_roll <= 16))
        {
            step = 7;
            break; // Keluar dari loop setelah roll sesuai
        }
        break;
    case 7:
        // Periksa kondisi yaw untuk koreksi arah terlebih dahulu
        if (imu_yaw >= 90 && imu_yaw <= 240)
        {
            printf("Koreksi arah: Robot berputar ke kanancase5\n");
            coba_putar_kananv3();
        }
        else if ((imu_yaw >= 292 && imu_yaw <= 360) || (imu_yaw >= 0 && imu_yaw <= 30))
        {
            printf("Koreksi arah: Robot berputar ke kiricase5\n");
            coba_putar_kiriv3();
        }
        else
        {
            printf("Robot berada dalam heading yang benar, maju rintangancase5\n");
            coba_majurintangan();
            itung++;
            // Periksa nilai Sensor_2
            // if (Sensor_2 > 30 && Sensor_2 < 55)
            // {
            //     printf("Sensor_2 dalam rentang 150-170 case6\n");
            //     step = 8;
            // }
        }
        if (itung >= 20)
        {
            step = 8;
        }
        else
        {
            step = 7;
        }

        break;
    case 8:
        coba_putar_kananv2();
        if (imu_yaw >= 30 && imu_yaw <= 90)
        {
            step = 9;
        }
        break;
    case 9:
        // Koreksi tambahan setelah imu_yaw memenuhi syarat
        if (Sensor_4 > 100 && Sensor_4 < 250)
        {
            coba_mundurv4();
            printf("Robot melakukan koreksi dengan mundur case8\n");
        }
        // Setelah koreksi mundur, cek apakah sensor2 lebih dari 80
        else if (Sensor_4 > 250)
        {
            step = 10;
            printf("Robot kembali ke posisi siap case9\n");
        }
        break;
    case 10:
        static ros::Time start_time_6; // Waktu awal untuk delay
        setPosisiServoSync(19, 249, 20, 235);
        // setPosisiServoSync(19, 249);
        // setPosisiServoSync(20, 235);
        printf("Robot berada dalam posisi case10\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_6.isZero())
            start_time_6 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_6).toSec() >= 1.0)
        {
            step = 43;                   // Pindah ke gerakan berikutnya
            start_time_6 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        break;
    case 43:
        static ros::Time start_time_33; // Waktu awal untuk delay
        publishServoAngle(0, 30);
        usleep(500000);
        // usleep(500000);
        setPosisiServoSync(19, 794, 20, 193);
        // setPosisiServoSync(19, 794);
        // setPosisiServoSync(20, 193);
        printf("Robot berada dalam posisi case10\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_33.isZero())
            start_time_33 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_33).toSec() >= 1.0)
        {
            step = 11;                    // Pindah ke gerakan berikutnya
            start_time_33 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        break;
    case 11:
        coba_putar_kiriv2();
        printf("Robot berada dalam posisi case11\n");
        if (imu_yaw >= 0 && imu_yaw <= 30)
        {
            step = 12;
        }
        break;
    ////////////////////////////////////////////// case korban 2 /////////////////////////////////
    case 12:
        coba_maju3();
        printf("Robot sedang melakukan case12\n");
        if (Sensor_2 >= 420 && Sensor_2 <= 481)
        {
            step = 13;
        }
        break;
    case 13:
        coba_putar_kiriv2();
        printf("Robot sedang melakukan case13\n");
        if (imu_yaw >= 160 && imu_yaw <= 198)
        {
            step = 14;
        }
        break;
    case 14:
        if (centerX == 0)
        {
            putar_kiri_korban34();
            printf("Robot nyari\n");
        }
        else if (centerX < 320)
        {
            putar_kiri_korban34();
            printf("Robot kekiri\n");
        }
        else if (centerX > 480)
        {
            putar_kanan_korban34();
            printf("Robot kanan\n");
        }
        else
        {
            coba_majuv32();
            printf("Robot maju\n");
            // step = 15;
            
             if (centerY >= 250 && centerY <= 300)
            {
                mundurrv32();
                // printf("Sensor_2 dalam rentang 150-170 case6\n");
                // step = 36; // Atur ke langkah berikutnya (atau langkah yang sesuai)
                // break;
            }
            else if (centerY >= 160 && centerY <= 230)
            {
                coba_majuv32();
            }
            else if (centerY > 230 && centerY < 250)
            {
                printf("Sensor_2 dalam rentang 150-170 case6\n");
                step = 15; // Atur ke langkah berikutnya (atau langkah yang sesuai)
                break;
            }
            
        }

        // if (Sensor_3 >= 50 && Sensor_3 <= 135)
        // {
        //     coba_majuv32();
        // }
        // else if (Sensor_3 >= 136 && Sensor_3 <= 235)
        // {
        //     mundurrv32();
        // }
        // else

        break;
    case 15:
        static ros::Time start_time_1; // Waktu awal untuk delay
                                       // Set nilai servo ke 45 dan publish
        publishServoAngle(0, 40);
        usleep(500000); // Tunggu sebentar untuk memastikan servo mulai bergerak
        setPosisiServoSync(19, 260, 20, 245);
        // setPosisiServoSync(19, 420);
        // setPosisiServoSync(20, 450);
        printf("Robot berada dalam posisi siapcase15\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_1.isZero())
            start_time_1 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_1).toSec() >= 1.0)
        {
            step = 44;                   // Pindah ke gerakan berikutnya
            start_time_1 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        break;
    case 44:
        // if (centerX == 0)
        // {
        //     putar_kiri_korban34();
        //     printf("Robot nyari\n");
        // }
        // else if (centerX < 320)
        // {
        //     putar_kiri_korban34();
        //     printf("Robot kekiri\n");
        // }
        // else if (centerX > 480)
        // {
        //     putar_kanan_korban34();
        //     printf("Robot kanan\n");
        // }
        // else
        // {
            //         coba_majuv32();
            // printf("Robot maju\n");
            for (int i = 0; i < 8; i++)
            {
                coba_majuv32();
                // usleep(500000); // Beri waktu agar gerakan maju selesai sebelum iterasi berikutnya
            }
        // }

        usleep(500000);
        publishServoAngle(0, 3);
        usleep(100000);
        for (int i = 0; i < 2; i++)
        {
            mundurrv32();
            // usleep(500000); // Beri waktu agar gerakan maju selesai sebelum iterasi berikutnya
        }
        // usleep(500000);
        // coba_mundurv3();
        printf("Robot mundur setelah dapat korban\n");
        step = 45;
        // if (centerY >= 230 && centerY <= 240)
        // {
        //     step = 42;
        // }
        break;
    case 45:
        static ros::Time start_time_16; // Waktu awal untuk delay
        setPosisiServoSync(19, 794, 20, 193);
        // setPosisiServoSync(19, 794);
        // setPosisiServoSync(20, 193);
        printf("Robot berada dalam posisi case10\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_16.isZero())
            start_time_16 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_16).toSec() >= 2.0)
        {
            step = 16;                    // Pindah ke gerakan berikutnya
            start_time_16 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        break;
    case 16:
        coba_putar_kananv2();
        if (imu_yaw >= 150 && imu_yaw <= 210)
        {
            step = 17;
        }
    case 17:
        coba_mundurv4();
        if (Sensor_3 <= 76)
        {
            step = 18;
        }
        break;
    case 18:
        coba_maju2();
        if (Sensor_2 <= 245)
        {
            step = 19;
        }
        break;
    case 19:
        coba_putar_kananv2();
        printf("Robot berada dalam posisi case17\n");
        if (imu_yaw >= 265 && imu_yaw <= 290)
        {
            step = 46;
        }
        break;
    case 46:
        static ros::Time start_time_23; // Waktu awal untuk delay
                                        // usleep(500000);
        setPosisiServoSync(19, 400, 20, 330);
        // setPosisiServoSync(19, 400);
        // setPosisiServoSync(20, 330);
        printf("Robot berada dalam posisi case10\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_23.isZero())
            start_time_23 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_23).toSec() >= 1.0)
        {
            step = 20;                    // Pindah ke gerakan berikutnya
            start_time_23 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        break;
    case 20:
        static ros::Time start_time_24; // Waktu awal untuk delay
        publishServoAngle(0, 30);
        usleep(500000);
        // usleep(500000);
        setPosisiServoSync(19, 794, 20, 193);
        // setPosisiServoSync(19, 794);
        // setPosisiServoSync(20, 193);
        printf("Robot berada dalam posisi siapcase18\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_24.isZero())
            start_time_24 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_24).toSec() >= 1.0)
        {
            step = 21;                    // Pindah ke gerakan berikutnya
            start_time_24 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        printf("Case 19: Robot sedang maju dengan koreksi\n");
        break;
    case 21:
        coba_maju1();
        if (Sensor_2 >= 360 && Sensor_2 <= 385)
        {
            step = 22;
        }
        break;
    case 22:
        coba_majuv3();
        if (Sensor_4 <= 73)
        {
            step = 23;
        }
        break;
    case 23:
        coba_maju1();
        printf("Robot sedang melakukan case21\n");
        if (Sensor_2 >= 480)
        {
            step = 24;
        }
        break;
    case 24:
        coba_mundurv3();
        if (Sensor_3 <= 79)
        {
            step = 25;
        }
        break;
    case 25:
        printf("Case 18: Robot sedang mundur\n");
        coba_mundurv4();

        if (imu_yaw >= 264 && imu_yaw <= 272)
        {
            printf("Robot stabil, siap maju (ke case 19)\n");
            step = 26;
        }
        break;
        ///////////////////////////////////////////// case korban 3 dan 4 //////////////////////////////////////
    case 26:

        // Menyesuaikan gerakan berdasarkan centerX
        if (centerX == 0)
        {
            coba_maju1();
            printf("Robot diam\n");
        }
        else if (centerX < 320)
        {
            putar_kiri_korban();
            printf("Robot kekiri\n");
        }
        else if (centerX > 480)
        {
            putar_kanan_korban();
            printf("Robot kanan\n");
        }
        else
        {
            maju_korban5();
            printf("Robot maju\n");
            // Deteksi rintangan
            if (Sensor_3 >= 215)
            {
                printf("Obstacle terdeteksi, pindah ke case 20\n");
                step = 51;
            }
        }
        break;
    case 51:
        static ros::Time start_time_51; // Waktu awal untuk delay
                                        // Set nilai servo ke 45 dan publish
        publishServoAngle(0, 30);
        usleep(500000); // Tunggu sebentar untuk memastikan servo mulai bergerak
        setPosisiServoSync(19, 340, 20, 360);
        // setPosisiServoSync(19, 420);
        // setPosisiServoSync(20, 450);
        printf("Robot berada dalam posisi siapcase15\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_51.isZero())
            start_time_51 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_51).toSec() >= 3.0)
        {
            step = 52;                    // Pindah ke gerakan berikutnya
            start_time_51 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        break;
    case 52:
        for (int i = 0; i < 9; i++)
        {
            maju_korban5();
            // usleep(500000); // Beri waktu agar gerakan maju selesai sebelum iterasi berikutnya
        }

        usleep(500000);
        publishServoAngle(0, 3);
        usleep(100000);
        for (int i = 0; i < 7; i++)
        {
            coba_mundurv3();
            // usleep(500000); // Beri waktu agar gerakan maju selesai sebelum iterasi berikutnya
        }
        // usleep(500000);
        // coba_mundurv3();
        printf("Robot mundur setelah dapat korban\n");
        step = 53;
        // if (centerY >= 230 && centerY <= 240)
        // {
        //     step = 42;
        // }
        break;
    case 53:
        static ros::Time start_time_53; // Waktu awal untuk delay
        setPosisiServoSync(19, 794, 20, 193);
        // setPosisiServoSync(19, 794);
        // setPosisiServoSync(20, 193);
        printf("Robot berada dalam posisi case10\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_53.isZero())
            start_time_53 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_53).toSec() >= 2.0)
        {
            step = 27;                    // Pindah ke gerakan berikutnya
            start_time_53 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        break;
    case 27:
        coba_majuv();
        printf("Robot sedang melakukan case21\n");
        if (Sensor_4 <= 82)
        {
            step = 28;
        }
        break;
    case 28:
        static ros::Time start_time_3; // Waktu awal untuk delay
        siap();
        printf("Robot berada dalam posisi siapcase22\n");
        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_3.isZero())
            start_time_3 = ros::Time::now();
        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_3).toSec() >= 3.0)
        {
            step = 29;                   // Pindah ke gerakan berikutnya
            start_time_3 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        printf("Robot sedang melakukan coba_putar_kananv3\n");
        break;
    case 29:
        itung = 0;
        if (imu_yaw >= 90 && imu_yaw <= 255)
        {
            printf("Koreksi arah: Robot berputar ke kanan23\n");
            coba_putar_kanan_tanggav2();
        }
        else if ((imu_yaw >= 281 && imu_yaw <= 360) || (imu_yaw >= 0 && imu_yaw <= 25))
        {
            printf("Koreksi arah: Robot berputar ke kiri23\n");
            coba_putar_kiri_tanggav2();
        }
        else
        {
            printf("Robot sudah dalam heading yang benar, maju rintangan dengan arah tangga23\n");
            tanggav2();
            itung++;
        }
        // ros::Duration(0.1).sleep(); // Tambahkan jeda 100ms agar robot bisa merespon
        if (itung > 15)
        {
            step = 30;
        }
        else
        {
            step = 29;
        }
        break;
    case 30:
        // Periksa kondisi yaw untuk koreksi arah terlebih dahulu
        if (imu_yaw >= 90 && imu_yaw <= 256)
        {
            printf("Koreksi arah: Robot berputar ke kanan24\n");
            coba_putar_kanan_tanggav2();
        }
        else if ((imu_yaw >= 280 && imu_yaw <= 360) || (imu_yaw >= 0 && imu_yaw <= 24))
        {
            printf("Koreksi arah: Robot berputar ke kiri24\n");
            coba_putar_kiri_tanggav2();
        }
        else
        {
            printf("Robot berada dalam heading yang benar, maju rintangan24\n");
            tanggav1();
        }
        if (imu_roll >= 350 && imu_roll <= 360)
        {
            step = 31;
        }
        break;
    case 31:
        coba_maju1();
        printf("Koreksi arah: roll robot menuju 25\n");
        if ((Sensor_1 >= 240 && Sensor_1 <= 260) || (Sensor_1 <= 160))
        {
            step = 32;
        }
        break;
    case 32:
        coba_majuv3();
        printf("Koreksi arah: roll robot menuju 26\n");
        if (Sensor_4 > 57 && Sensor_4 < 118)
        {
            step = 34;
        }
        break;
        //////////////////////////////////////// case korban 5 /////////////////////////////////////////////////
    // case 33:
    //     coba_maju3();
    //     if ( Sensor_1 >= 200 && Sensor_1 <= 255)
    //     {
    //         step = 34;
    //     }
    //     break;
    case 34:
        putar_kiri_korban();
        if (imu_yaw >= 179 && imu_yaw <= 239)
        {
            step = 35;
        }
        break;
    case 35:
        if (centerX == 0)
        {
            putar_kiri_korban();
            printf("Robot diam\n");
        }
        else if (centerX < 350)
        {
            putar_kiri_korban();
            printf("Robot kekiri\n");
        }
        else if (centerX > 510)
        {
            putar_kanan_korban();
            printf("Robot kanan\n");
        }
        else
        {
            maju_korban5();
            printf("Robot maju\n");
            if (centerY >= 220 && centerY <= 270)
            {
                printf("Sensor_2 dalam rentang 150-170 case6\n");
                step = 36; // Atur ke langkah berikutnya (atau langkah yang sesuai)
                // break;
            }
        }
        // step = 37;
        break;
    case 36:
        static ros::Time start_time_4; // Waktu awal untuk delay
        publishServoAngle(0, 30);
        usleep(500000); // Tunggu sebentar untuk memastikan servo mulai bergerak
        setPosisiServoSync(19, 249, 20, 235);
        printf("Robot berada dalam posisi siapcase22\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_4.isZero())
            start_time_4 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_4).toSec() >= 3.0)
        {
            step = 47;                   // Pindah ke gerakan berikutnya
            start_time_4 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        printf("Robot sedang melakukan coba_putar_kananv3\n");
        break;
    case 47:
        for (int i = 0; i < 9; i++)
        {
            maju_korban5();
            // usleep(500000); // Beri waktu agar gerakan maju selesai sebelum iterasi berikutnya
        }
        siap_angkat_korban();
        usleep(500000);
        publishServoAngle(0, 3);
        usleep(100000);
        for (int i = 0; i < 3; i++)
        {
            coba_mundurv3();
            // usleep(500000); // Beri waktu agar gerakan maju selesai sebelum iterasi berikutnya
        }
        // usleep(500000);
        // coba_mundurv3();
        printf("Robot mundur setelah dapat korban\n");
        step = 48;
        // if (centerY >= 230 && centerY <= 240)
        // {
        //     step = 42;
        // }
        break;
    case 48:
        static ros::Time start_time_21; // Waktu awal untuk delay
        setPosisiServoSync(19, 794, 20, 193);
        // setPosisiServoSync(19, 794);
        // setPosisiServoSync(20, 193);
        printf("Robot berada dalam posisi case10\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_21.isZero())
            start_time_21 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_21).toSec() >= 2.0)
        {
            step = 37;                    // Pindah ke gerakan berikutnya
            start_time_21 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        break;
    case 37:
        coba_putar_kiriv2();
        if (imu_yaw >= 150 && imu_yaw <= 210)
        {
            step = 38;
        }
        break;
    case 38:
        coba_mundurv3();
        if (Sensor_4 >= 275 && Sensor_4 <= 385)
        {
            step = 39;
        }
        break;
    case 39:
        coba_maju3();
        if (Sensor_1 <= 140)
        {
            step = 40;
        }
        break;
    case 40:
        if (Sensor_3 >= 60 && Sensor_3 <= 240)
        {
            coba_majuv3();
        }
        else if (Sensor_3 >= 262)
        {
            coba_mundurv3();
        }
        else
        {
            step = 41;
        }
        break;
    case 41:
        static ros::Time start_time_35; // Waktu awal untuk delay
        setPosisiServoSync(19, 400, 20, 330);
        // siap();
        printf("Robot berada dalam posisi siapcase22\n");
        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_35.isZero())
            start_time_35 = ros::Time::now();
        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_35).toSec() >= 3.0)
        {
            step = 49;                    // Pindah ke gerakan berikutnya
            start_time_35 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        printf("Robot sedang melakukan coba_putar_kananv3\n");
        break;
    case 49:
        static ros::Time start_time_41; // Waktu awal untuk delay
                                        // Set nilai servo ke 45 dan publish
        publishServoAngle(0, 30);
        usleep(500000); // Tunggu sebentar untuk memastikan servo mulai bergerak
        // setPosisiServoSync(19, 420);
        // setPosisiServoSync(20, 450);
        printf("Robot berada dalam posisi siapcase15\n");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_41.isZero())
            start_time_41 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_41).toSec() >= 3.0)
        {
            step = 50;                    // Pindah ke gerakan berikutnya
            start_time_41 = ros::Time(0); // Reset waktu untuk step berikutnya
            printf("Delay 3 detik selesai, pindah ke coba_putar_kiriv3\n");
        }
        break;
    case 50:
        setPosisiServoSync(19, 794, 20, 193);
        break;
    default:
        berhenti_robot();
        printf("Robot berhenti\n");
        break;
    }
}
void detectionCallback(const std_msgs::String::ConstPtr &msg)
{
    std::string detection_data = msg->data;
    // Mencari posisi "Korban" dan "Confidence:"
    std::size_t korban_pos = detection_data.find("korban");
    std::size_t at_pos = detection_data.find("(");
    std::size_t comma_pos = detection_data.find(",", at_pos); // Menemukan posisi koma setelah centerX
    std::size_t confidence_pos = detection_data.find("Confidence:");

    if (korban_pos != std::string::npos && at_pos != std::string::npos && comma_pos != std::string::npos && confidence_pos != std::string::npos)
    {
        // Ekstrak centerX
        std::string centerX_str = detection_data.substr(at_pos + 1, confidence_pos - at_pos - 3);        // Mengambil nilai antara "(" dan ","
        std::size_t close_paren_pos = detection_data.find(")", comma_pos);                               // Menemukan posisi tanda ")"
        std::string centerY_str = detection_data.substr(comma_pos + 1, close_paren_pos - comma_pos - 1); // Mengambil nilai antara "," dan ")"
        // Menghapus karakter yang tidak diinginkan seperti tanda kurung atau spasi
        centerY_str.erase(0, centerY_str.find_first_not_of(" \t\n\r()"));        // Hapus karakter yang tidak diinginkan di awal (termasuk tanda kurung)
        centerY_str.erase(centerY_str.find_last_not_of(" \t\n\r()") + 1);        // Hapus karakter yang tidak diinginkan di akhir (termasuk tanda kurung)
        std::string confidence_str = detection_data.substr(confidence_pos + 12); // Mengambil nilai setelah "Confidence: "
        try
        {
            centerX = std::stof(centerX_str); // Mengonversi string ke float
            centerY = std::stof(centerY_str);
            confidence = std::stof(confidence_str); // Mengonversi string ke float
            objek_terdeteksi = true;
            // printf("Center Korban: %.2f, Confidence: %.2f", centerX, confidence);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("error detection data: %s", e.what());
            objek_terdeteksi = false;
        }
        // printf("Center korban: %.2f, centerY: %.2f, Confidence: %.2f", centerX, centerY, confidence);
    }
    else
    {
        ROS_WARN("Failed to parse detection data.");
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    // Inisialisasi GPIO untuk tombol
    initGPIO();
    if (!mulai())
    {
        return 1; // Gagal inisialisasi, keluar program
    }
    ros::Subscriber euler_sub = nh.subscribe("imu/euler", 10, imuCallback);
    ros::Subscriber distance_sub = nh.subscribe("sensor_distance", 10, distanceCallback);
    ros::Subscriber sub = nh.subscribe("detections", 1000, detectionCallback);

    angle_pub[0] = nh.advertise<std_msgs::Float32>("servo1", 10);
    angle_pub[1] = nh.advertise<std_msgs::Float32>("servo2", 10);
    angle_pub[2] = nh.advertise<std_msgs::Float32>("servo3", 10);

    //  Loop utama untuk menjalankan perintah aktif
    ros::Rate loop_rate(33); // Menentukan frekuensi loop (10 Hz)
    while (ros::ok())
    {
        updateButtonStatus(); // Perbarui status tombol
        prosesLogika();       // Jalankan logika utama
        // publishServoAngles();
        // Memproses callback ROS
        ros::spinOnce();

        loop_rate.sleep();
    }
    GPIO::cleanup();
    return 0;
}