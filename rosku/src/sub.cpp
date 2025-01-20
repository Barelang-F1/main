#include "../include/inverse.cpp"
#include "ros/ros.h"
#include <JetsonGPIO.h>
#include "std_msgs/String.h"
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/UInt16.h"           // Untuk pesan jarak
#include <std_msgs/UInt16MultiArray.h> // Menggunakan UInt16MultiArray
#include <math.h>
#define BUT_PIN 6                 // Pin 37 adalah GPIO26
#define STOP_PIN 12               // Pin 32 adalah GPIO12
bool but = false;                 // Variabel untuk status tombol
bool stop_but = false;            // Variabel untuk status tombol stop
int button_press_count = 0;       // Penghitung tekanan tombol
ros::Time last_button_time;       // Waktu terakhir tombol ditekan
ros::Duration debounce_time(0.2); // Debounce selama 300ms
bool program_running = true;      // Status program berjalan
int step = 0;
bool case_aktif[11] = {true, false, false, false, false, false, false, false, false, false, false}; // Status awal: hanya case 0 aktif
// Variabel global untuk menyimpan perintah aktif dan data IMU
std::string current_command = "";
float imu_yaw = 0.0, imu_roll = 0.0, imu_pitch = 0.0;

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
    for (int i = 0; i < 18; i++)
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

    siap();

    // while (1)
    // {
    //     // maju_robot();
    // }

    // portHandler->closePort();
}

void maju()
{
    ROS_INFO("Robot bergerak maju");
    coba_majurintangan(); // Tambahkan logika untuk perintah maju robot di sini
}
void maju_kiri()
{
    coba_maju1();
    ROS_INFO("Robot bergerak maju");
}
void berhenti_robot()
{
    ROS_INFO("Robot berhenti untuk mempertahankan heading.");
    siap();
    // Tambahkan logika untuk menghentikan semua motor robot
}
void kiri_robot1()
{
    ROS_INFO("Robot berhenti untuk mempertahankan heading.");
    coba_mundur();
    // Tambahkan logika untuk menghentikan semua motor robot
}
// Fungsi untuk memutar robot ke kanan
void kanan_robot1()
{
    ROS_INFO("Robot berputar ke kanan untuk mengoreksi heading.");
    coba_maju();
    // Tambahkan logika untuk memutar robot ke kanan
}
void kanann_robot2()
{
    ROS_INFO("Robot berputar ke kanan untuk mengoreksi heading.");
    coba_maju2();
    // Tambahkan logika untuk memutar robot ke kanan
}
void maju_robot2()
{
    ROS_INFO("Robot berputar ke kanan untuk mengoreksi heading.");
    coba_majuv33();
    // Tambahkan logika untuk memutar robot ke kanan
}
void putar_kanan_robot1()
{
    ROS_INFO("Robot berputar ke kanan untuk mengoreksi heading.");
    coba_putar_kananv2();
    // Tambahkan logika untuk memutar robot ke kanan
}
void putar_kanan_robot()
{
    ROS_INFO("Robot berputar ke kanan untuk mengoreksi heading.");
    coba_putar_kananv3();
    // Tambahkan logika untuk memutar robot ke kanan
}
// Fungsi untuk memutar robot ke kiri
void maju_robot1()
{
    ROS_INFO("Robot berputar ke kiri untuk mengoreksi heading.");
    coba_majuv3();
    // Tambahkan logika untuk memutar robot ke kiri
}
void putar_kiri_robot1()
{
    ROS_INFO("Robot berputar ke kanan untuk mengoreksi heading.");
    coba_putar_kiriv2();
}
void putar_kiri_robot()
{
    ROS_INFO("Robot berputar ke kanan untuk mengoreksi heading.");
    coba_putar_kiriv3();
}
void rintangan_1()
{
    coba_majurintangan1();
}
void mundur_v4()
{
    coba_mundurv4();
}
void initGPIO()
{
    GPIO::setmode(GPIO::BCM);        // Gunakan nomor pin BOARD
    GPIO::setup(BUT_PIN, GPIO::IN);  // Konfigurasi pin 6 sebagai input
    GPIO::setup(STOP_PIN, GPIO::IN); // Konfigurasi pin 12 sebagai input
}

void updateButtonStatus()
{
    ros::Time current_time = ros::Time::now();

    // Cek tombol utama (pin 6)
    if (GPIO::input(BUT_PIN) == GPIO::LOW) // LOW artinya tombol ditekan
    {
        if ((current_time - last_button_time) > debounce_time) // Cek debounce
        {
            but = true;
            last_button_time = current_time;
            button_press_count++;

            if (button_press_count == 2) // Jika tombol ditekan dua kali
            {
                button_press_count = 0; // Reset penghitung
                ROS_INFO("Tombol ditekan dua kali. Program akan dimulai ulang dalam 5 detik.");
                ros::Duration(5).sleep(); // Tunggu selama 5 detik
                ROS_INFO("Program dimulai ulang.");
                step = 0;               // Reset step ke awal
                siap();                 // Kembali ke posisi siap
                program_running = true; // Jalankan program kembali
            }
        }
    }
    else
    {
        but = false;
    }

    // Cek tombol stop (pin 12)
    if (GPIO::input(STOP_PIN) == GPIO::LOW) // LOW artinya tombol ditekan
    {
        if ((current_time - last_button_time) > debounce_time) // Cek debounce
        {
            stop_but = true;
            ROS_INFO("Tombol stop ditekan. Program dihentikan.");
            program_running = false; // Hentikan program
            siap();                  // Kembali ke posisi siap
        }
    }
    else
    {
        stop_but = false;
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
            ROS_INFO("Tombol ditekan, robot mulai kanan.");
            case_aktif[0] = false; // Matikan case 0
            case_aktif[1] = true;  // Aktifkan case 1
        }
        break;

    case 1:
        coba_maju();
        if (Sensor_1 > 360)
        {
            step = 2;
            ROS_INFO("Sensor_1 > 360, robot bergerak maju.");
            case_aktif[1] = false; // Matikan case 1
            case_aktif[2] = true;  // Aktifkan case 2
        }
        break;

    case 2:
        coba_majuv3();
        if (Sensor_4 < 218)
        {
            step = 3;
            ROS_INFO("Sensor_4 < 218, robot bergerak maju.");
            case_aktif[2] = false; // Matikan case 2
            case_aktif[3] = true;  // Aktifkan case 3
        }
        break;

    case 3:
        coba_mundurv3();
        if (Sensor_4 > 218 && Sensor_3 < 80)
        {
            step = 4;
            ROS_INFO("Sensor_4 > 218, robot bergerak mundur.");
            case_aktif[3] = false; // Matikan case 3
            case_aktif[4] = true;  // Aktifkan case 4
        }
        break;

    case 4:
        // Loop sampai roll berada dalam rentang yang diinginkan
        if (imu_yaw >= 90 && imu_yaw <= 250)
        {
            ROS_INFO("Koreksi arah: Robot berputar ke kanan.");
            coba_putar_kananv3();
        }
        else if ((imu_yaw >= 300 && imu_yaw <= 360) || (imu_yaw >= 0 && imu_yaw <= 30))
        {
            ROS_INFO("Koreksi arah: Robot berputar ke kiri.");
            coba_putar_kiriv3();
        }
        else
        {
            ROS_INFO("Robot maju rintangan dengan arah lurus.");
            coba_majurintangan();
        }

        // Update imu_roll untuk memastikan robot bergerak sesuai dengan kondisi roll
        // Periksa roll dan tetap berada di case 4 sampai nilai roll berada dalam rentang yang ditentukan
        if ((imu_roll >= 235 && imu_roll <= 250) || (imu_roll > 0 && imu_roll < 15))
        {
            case_aktif[4] = false; // Matikan case 4
            step = 5;
            case_aktif[5] = true; // Aktifkan case 5
            break;                // Keluar dari loop setelah roll sesuai
        }
        break;

    case 5:
        // Periksa kondisi yaw untuk koreksi arah terlebih dahulu
        if (imu_yaw >= 90 && imu_yaw <= 240)
        {
            ROS_INFO("Koreksi arah: Robot berputar ke kanan.");
            coba_putar_kananv3();
        }
        else if ((imu_yaw >= 302 && imu_yaw <= 360) || (imu_yaw >= 0 && imu_yaw <= 30))
        {
            ROS_INFO("Koreksi arah: Robot berputar ke kiri.");
            coba_putar_kiriv3();
        }
        else
        {
            ROS_INFO("Robot berada dalam heading yang benar, maju rintangan1.");
            coba_majurintangan1();

            // Periksa nilai Sensor_2
            if (Sensor_2 > 20 && Sensor_2 < 50)
            {
                ROS_INFO("Sensor_2 dalam rentang 150-170. Robot berputar ke kanan.");

                // Update step ke langkah berikutnya jika diperlukan
                case_aktif[5] = false; // Matikan case 5
                step = 6;              // Atur ke langkah berikutnya (atau langkah yang sesuai)
                case_aktif[6] = true;  // Aktifkan case 6
            }
        }
        break;
    case 6:
        coba_mundurv3();
        // Cek kondisi imu_yaw untuk pindah ke step 7
        if (Sensor_3 > 115 && Sensor_3 < 150)
        {
            step = 7;
        }
        break;
    case 7:
        coba_putar_kananv2();
        ROS_INFO("Robot berada dalam coba_putar_kananv3.");

        // Cek kondisi imu_yaw untuk pindah ke step 7
        if (imu_yaw >= 35 && imu_yaw <= 48)
        {
            siap();
            step = 8;
        }

        break;
    case 8:
        // Koreksi tambahan setelah imu_yaw memenuhi syarat
        if (Sensor_4 > 100 && Sensor_4 < 250)
        {
            coba_mundurv3();
            ROS_INFO("Robot melakukan koreksi dengan mundur karena sensor4 dalam rentang 20-80.");
        }

        // Setelah koreksi mundur, cek apakah sensor2 lebih dari 80
        else if (Sensor_4 > 250)
        {
            step = 9;
            ROS_INFO("Robot kembali ke posisi siap setelah sensor4 lebih dari 80.");
        }
        break;

    case 9:
        static ros::Time start_time; // Waktu awal untuk delay
        siap();
        ROS_INFO("Robot berada dalam posisi siap.");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time.isZero())
            start_time = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time).toSec() >= 3.0)
        {
            step = 10;                 // Pindah ke gerakan berikutnya
            start_time = ros::Time(0); // Reset waktu untuk step berikutnya
            ROS_INFO("Delay 3 detik selesai, pindah ke coba_putar_kiriv3.");
        }
        break;

    case 10:
        coba_putar_kiriv2();
        if (imu_yaw >= 0 && imu_yaw <= 10)
        {
            step = 11;
        }
        ////////////////////////////////////////case korban 2/////////////////////////////////////////////////
    case 11:
        coba_maju1();
        ROS_INFO("Robot sedang melakukan coba_putar_kiriv3.");
        if (Sensor_1 <= 380)
        {
            step = 12;
        }
        break;

    case 12:
        coba_putar_kiriv2();
        ROS_INFO("Robot sedang melakukan coba_putar_kananv3.");
        if (imu_yaw >= 150 && imu_yaw <= 198)
        {
            siap();
            step = 13;
        }
        break;
    case 13:
        // Koreksi tambahan setelah imu_yaw memenuhi syarat
        if (Sensor_4 > 160 && Sensor_4 < 170)
        {
            coba_mundurv3();
            ROS_INFO("Robot melakukan koreksi dengan mundur karena sensor4 dalam rentang 20-80.");
        }

        // Setelah koreksi mundur, cek apakah sensor2 lebih dari 80
        else if (Sensor_2 <= 165)
        {
            coba_maju1();
        }
        else if (Sensor_4 > 170)
        {
            step = 14;
            ROS_INFO("Robot kembali ke posisi siap setelah sensor4 lebih dari 80.");
        }
        break;

    case 14:
        static ros::Time start_time_1; // Waktu awal untuk delay
        siap();
        ROS_INFO("Robot berada dalam posisi siap.");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_1.isZero())
            start_time_1 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_1).toSec() >= 3.0)
        {
            step = 15;                   // Pindah ke gerakan berikutnya
            start_time_1 = ros::Time(0); // Reset waktu untuk step berikutnya
            ROS_INFO("Delay 3 detik selesai, pindah ke coba_putar_kiriv3.");
        }
        ROS_INFO("Robot sedang melakukan coba_putar_kananv3.");
        break;
    case 15:
        coba_maju2();
        ROS_INFO("Robot sedang melakukan coba_putar_kananv3.");
        if (Sensor_2 <= 40)
        {
            
            step = 16;
        }
        break;
    case 16:
    coba_putar_kananv2();
     if (imu_yaw >= 330 && imu_yaw <= 345)
        {
            step = 17;
        }

        break;
    case 17:
        static ros::Time start_time_2; // Waktu awal untuk delay
        siap();
        ROS_INFO("Robot berada dalam posisi siap.");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_2.isZero())
            start_time_2 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_2).toSec() >= 3.0)
        {
            step = 18;                 // Pindah ke gerakan berikutnya
            start_time_2 = ros::Time(0); // Reset waktu untuk step berikutnya
            ROS_INFO("Delay 3 detik selesai, pindah ke coba_putar_kiriv3.");
        }
        ROS_INFO("Robot sedang melakukan coba_putar_kananv3.");
        break;
    case 18:
    coba_mundurv4();
            if(imu_yaw  >= 264 && imu_yaw <= 272)
    {
        step = 19;
    }
    
    ROS_INFO("Robot sedang melakukan coba_putar_kiriv3.");

        break;
    case 19:
coba_maju1();
    ROS_INFO("Robot sedang melakukan maju1.");
    if(Sensor_1 <= 35){
        step = 20;
    }
        break;
    case 20:
    coba_majuv3();
    if(Sensor_4 <= 25){
        step = 21;
    }
    break;
    case 21:
        static ros::Time start_time_3; // Waktu awal untuk delay
        siap();
        ROS_INFO("Robot berada dalam posisi siap.");

        // Inisialisasi waktu saat pertama kali masuk case 7
        if (start_time_3.isZero())
            start_time_3 = ros::Time::now();

        // Cek apakah sudah 3 detik
        if ((ros::Time::now() - start_time_3).toSec() >= 3.0)
        {
            // step = 22;                 // Pindah ke gerakan berikutnya
            start_time_3 = ros::Time(0); // Reset waktu untuk step berikutnya
            ROS_INFO("Delay 3 detik selesai, pindah ke coba_putar_kiriv3.");
        }
        ROS_INFO("Robot sedang melakukan coba_putar_kananv3.");
    break;
    // case 22:

    default:
        berhenti_robot();
        ROS_INFO("Robot berhenti.");
        break;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    // Inisialisasi GPIO untuk tombol
    initGPIO();
    if (!mulai())
    {
        return 1; // Gagal inisialisasi, keluar program
    }

    // Subscribing ke topik chatter
    // ros::Subscriber sub = n.subscribe("chatter", 1000, commandCallback);
    // Subscriber untuk menerima data IMU
    ros::Subscriber euler_sub = n.subscribe("imu/euler", 10, imuCallback);
    ros::Subscriber distance_sub = n.subscribe("sensor_distance", 10, distanceCallback);
    // Loop utama untuk menjalankan perintah aktif
    ros::Rate loop_rate(33); // Menentukan frekuensi loop (10 Hz)
    while (ros::ok())
    {
        updateButtonStatus(); // Perbarui status tombol
        prosesLogika();       // Jalankan logika utama
        // Memproses callback ROS
        ros::spinOnce();

        loop_rate.sleep();
    }
    GPIO::cleanup();
    return 0;
}
