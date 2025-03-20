#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <fcntl.h>  // Untuk open()
#include <unistd.h> // Untuk read(), write(), close()
#include <sys/ioctl.h> // Untuk ioctl()
#include <linux/i2c-dev.h> // Untuk konstanta I2C_SLAVE
#include <cmath> // Untuk perhitungan map fungsi

#define PCA9685_I2C_ADDR 0x40 // Alamat I2C PCA9685
#define MODE1 0x00  // Register MODE1
#define PRESCALE 0xFE  // Register untuk prescale
#define LED0_ON_L 0x06  // Register awal untuk PWM

int file;

// Fungsi untuk inisialisasi I2C
int i2c_init(const char* i2c_device) {
    int file = open(i2c_device, O_RDWR);
    if (file < 0) {
        ROS_ERROR("Gagal membuka I2C device!");
        return -1;
    }
    if (ioctl(file, I2C_SLAVE, PCA9685_I2C_ADDR) < 0) {
        ROS_ERROR("Gagal mengakses driver servo!");
        close(file);
        return -1;
    }
    return file;
}

// Fungsi untuk menulis ke register
void write_register(int file, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(file, buffer, 2) != 2) {
        ROS_ERROR("Gagal menulis ke register!");
    }
}

// Fungsi untuk mengatur frekuensi PWM
void set_pwm_freq(int file, float freq) {
    uint8_t oldmode = 0x10; // Masuk mode sleep sementara (untuk konfigurasi)
    write_register(file, MODE1, oldmode);

    uint8_t prescale = static_cast<uint8_t>(std::round(25000000.0 / (4096.0 * freq)) - 1);
    write_register(file, PRESCALE, prescale);

    write_register(file, MODE1, 0x00); // Keluar dari mode sleep
    usleep(500);
    write_register(file, MODE1, 0xA1); // Aktifkan autoincrement
}

// Fungsi untuk mengatur posisi servo
void set_servo_pwm(int file, uint8_t channel, uint16_t on, uint16_t off) {
    write_register(file, LED0_ON_L + 4 * channel, on & 0xFF);
    write_register(file, LED0_ON_L + 4 * channel + 1, on >> 8);
    write_register(file, LED0_ON_L + 4 * channel + 2, off & 0xFF);
    write_register(file, LED0_ON_L + 4 * channel + 3, off >> 8);
}

// Fungsi untuk menghitung nilai PWM berdasarkan derajat
uint16_t map_angle_to_pwm(float angle) {
    float min_pulse = 180;
    float max_pulse = 630;
    return static_cast<uint16_t>((angle / 180.0) * (max_pulse - min_pulse) + min_pulse);
}

// Callback untuk menerima sudut servo 1 dan menggerakkan servo
void servo1Callback(const std_msgs::Float32::ConstPtr& msg) {
    float angle = msg->data;
    ROS_INFO("Servo 1 menerima sudut: %f", angle);

    if (angle < 0 || angle > 180) {
        ROS_ERROR("Servo 1: Sudut di luar rentang!");
        return;
    }

    set_servo_pwm(file, 0, 0, map_angle_to_pwm(angle));
    ROS_INFO("Servo 1 diatur ke sudut: %f derajat", angle);
}

// Callback untuk menerima sudut servo 2 dan menggerakkan servo
void servo2Callback(const std_msgs::Float32::ConstPtr& msg) {
    float angle = msg->data;
    ROS_INFO("Servo 2 menerima sudut: %f", angle);

    if (angle < 0 || angle > 180) {
        ROS_ERROR("Servo 2: Sudut di luar rentang!");
        return;
    }

    set_servo_pwm(file, 1, 0, map_angle_to_pwm(angle));
    ROS_INFO("Servo 2 diatur ke sudut: %f derajat", angle);
}

int main(int argc, char **argv) {
    // Inisialisasi ROS
    ros::init(argc, argv, "dual_servo_angle_subscriber");
    ros::NodeHandle nh;

    // Inisialisasi I2C
    file = i2c_init("/dev/i2c-1");
    if (file < 0) {
        return 1;
    }

    set_pwm_freq(file, 50.0); // Atur frekuensi PWM

    // Inisialisasi subscriber untuk kedua servo
    ros::Subscriber servo1_sub = nh.subscribe("servo1_angle", 10, servo1Callback);
    ros::Subscriber servo2_sub = nh.subscribe("servo2_angle", 10, servo2Callback);

    ros::spin(); // Tunggu pesan dari publisher

    // Menutup file I2C
    close(file);

    return 0;
}
