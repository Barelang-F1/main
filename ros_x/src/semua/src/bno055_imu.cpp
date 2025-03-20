#include <iostream>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <JetsonGPIO.h>
#include <signal.h>
#include "std_msgs/Float32.h"
#include <cmath>

// VL53L0X
#define VL53L0X_DEFAULT_ADDRESS 0x29
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x32
#define LOX3_ADDRESS 0x34
#define LOX4_ADDRESS 0x36 // Alamat sensor 4
#define SHT_LOX1 21 // GPIO untuk sensor 1
#define SHT_LOX2 20 // GPIO untuk sensor 2
#define SHT_LOX3 19 // GPIO untuk sensor 3
#define SHT_LOX4 16 // GPIO untuk sensor 4

// BNO055
#define BNO055_ADDRESS 0x28
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_EULER_H_ADDR 0x1A
#define BNO055_EULER_R_ADDR 0x1C
#define BNO055_EULER_P_ADDR 0x1E

// Servo
#define PCA9685_I2C_ADDR 0x40
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06

int file;

static volatile bool keep_running = true;
std::mutex i2c_mutex;

void signalHandler(int signal)
{
    keep_running = false;
    std::cout << "Signal " << signal << " received. Cleaning up GPIO..." << std::endl;
    GPIO::cleanup();
}

int i2c_init(const char *i2c_device, uint8_t address)
{
    int file = open(i2c_device, O_RDWR);
    if (file < 0)
    {
        std::cerr << "Gagal membuka I2C device: " << i2c_device << std::endl;
        return -1;
    }
    if (ioctl(file, I2C_SLAVE, address) < 0)
    {
        std::cerr << "Gagal mengakses sensor pada alamat: " << static_cast<int>(address) << std::endl;
        close(file);
        return -1;
    }
    return file;
}

void write_register(int file, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    if (write(file, buffer, 2) != 2)
    {
        std::cerr << "Gagal menulis " << static_cast<int>(value) << " ke register " << static_cast<int>(reg) << std::endl;
    }
}

uint8_t read_register(int file, uint8_t reg)
{
    uint8_t value;
    if (write(file, &reg, 1) != 1)
    {
        std::cerr << "Gagal menulis ke register!" << std::endl;
        return -1;
    }
    if (read(file, &value, 1) != 1)
    {
        std::cerr << "Gagal membaca register!" << std::endl;
        return -1;
    }
    return value;
}

uint16_t read_distance(int file)
{
    uint8_t high_byte = read_register(file, 0x1E);
    uint8_t low_byte = read_register(file, 0x1F);

    uint16_t distance = (high_byte << 8) | low_byte;

    // Periksa apakah jarak valid
    if (distance == 20 || distance > 8190) // Nilai tidak valid
    {
        return 8190; // Out of range
    }

    return distance;
}

void reset_sensor(int gpio_pin, uint8_t new_address)
{
    GPIO::output(gpio_pin, GPIO::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    GPIO::output(gpio_pin, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    int file = i2c_init("/dev/i2c-1", VL53L0X_DEFAULT_ADDRESS);
    if (file >= 0)
    {
        write_register(file, 0x8A, new_address);                    // Atur alamat baru
        std::this_thread::sleep_for(std::chrono::milliseconds(25)); // Tambah delay agar stabil
        close(file);
    }
}

uint16_t read_distance_retry(int file, int retries = 3)
{
    uint16_t distance = 0;
    while (retries-- > 0)
    {
        distance = read_distance(file);
        if (distance > 0 && distance <= 8190) // Rentang valid
            break;
        std::cerr << "Retry membaca jarak..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return distance;
}

// Fungsi setup untuk VL53L0X
void setup_sensors()
{
    GPIO::setmode(GPIO::BCM);
    GPIO::setup(SHT_LOX1, GPIO::OUT, GPIO::LOW);
    GPIO::setup(SHT_LOX2, GPIO::OUT, GPIO::LOW);
    GPIO::setup(SHT_LOX3, GPIO::OUT, GPIO::LOW);
    GPIO::setup(SHT_LOX4, GPIO::OUT, GPIO::LOW);

    // Reset semua sensor
    GPIO::output(SHT_LOX1, GPIO::LOW);
    GPIO::output(SHT_LOX2, GPIO::LOW);
    GPIO::output(SHT_LOX3, GPIO::LOW);
    GPIO::output(SHT_LOX4, GPIO::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Unreset semua sensor
    GPIO::output(SHT_LOX1, GPIO::HIGH);
    GPIO::output(SHT_LOX2, GPIO::HIGH);
    GPIO::output(SHT_LOX3, GPIO::HIGH);
    GPIO::output(SHT_LOX4, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Atur alamat untuk masing-masing sensor

    reset_sensor(SHT_LOX1, LOX1_ADDRESS);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Tunggu stabilisasi Kalibrasi sensor 1
    int file1 = i2c_init("/dev/i2c-1", LOX1_ADDRESS);

    if (file1 >= 0)
    {
        write_register(file1, 0x002E, 0x01); // Memulai kalibrasi
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        close(file1);
    }

    reset_sensor(SHT_LOX2, LOX2_ADDRESS);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Tunggu stabilisasi Kalibrasi sensor 2

    int file2 = i2c_init("/dev/i2c-1", LOX2_ADDRESS);
    if (file2 >= 0)
    {
        write_register(file2, 0x002E, 0x01); // Memulai kalibrasi
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        close(file2);
    }

    reset_sensor(SHT_LOX3, LOX3_ADDRESS);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Tunggu stabilisasi

    // Kalibrasi sensor 3
    int file3 = i2c_init("/dev/i2c-1", LOX3_ADDRESS);
    if (file3 >= 0)
    {
        write_register(file3, 0x002E, 0x01); // Memulai kalibrasi
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        close(file3);
    }

    reset_sensor(SHT_LOX4, LOX4_ADDRESS);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Tunggu stabilisasi

    // Kalibrasi sensor 3
    int file4 = i2c_init("/dev/i2c-1", LOX4_ADDRESS);
    if (file4 >= 0)
    {
        write_register(file4, 0x002E, 0x01); // Memulai kalibrasi
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        close(file4);
    }
}

bool is_sensor_ready(int file)
{
    uint8_t ready_status = read_register(file, 0x13); // Contoh register status
    return (ready_status & 0x01);                     // Periksa bit siap
}
void measure_distance(int file1, int file2, int file3, int file4, ros::Publisher &distance_pub)
{
    uint16_t distance1 = 0;
    uint16_t distance2 = 0;
    uint16_t distance3 = 0;
    uint16_t distance4 = 0;

    if (is_sensor_ready(file1))
        distance1 = read_distance_retry(file1);
    if (is_sensor_ready(file2))
        distance2 = read_distance_retry(file2);
    if (is_sensor_ready(file3))
        distance3 = read_distance_retry(file3);
    if (is_sensor_ready(file4))
        distance3 = read_distance_retry(file4);

    {
        std::lock_guard<std::mutex> lock(i2c_mutex);
        write_register(file1, 0x00, 0x01); // Memulai pengukuran untuk sensor 1
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        distance1 = read_distance(file1);
    }

    {
        std::lock_guard<std::mutex> lock(i2c_mutex);
        write_register(file2, 0x00, 0x01); // Memulai pengukuran untuk sensor 2
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        distance2 = read_distance(file2);
    }

    {
        std::lock_guard<std::mutex> lock(i2c_mutex);
        write_register(file3, 0x00, 0x01); // Memulai pengukuran untuk sensor 2
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        distance3 = read_distance(file3);
    }
    {
        std::lock_guard<std::mutex> lock(i2c_mutex);
        write_register(file4, 0x00, 0x01); // Memulai pengukuran untuk sensor 2
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        distance4 = read_distance(file4);
    }

    // Publikasi hasil ke ROS sebagai array uint16
    std_msgs::UInt16MultiArray msg; // Menggunakan UInt16MultiArray
    msg.data.clear();               // Pastikan array kosong
    msg.data.push_back(distance1);  // Masukkan jarak dari sensor 1
    msg.data.push_back(distance2);  // Masukkan jarak dari sensor 2
    msg.data.push_back(distance3);  // Masukkan jarak dari sensor 2
    msg.data.push_back(distance4);  // Masukkan jarak dari sensor 2
    distance_pub.publish(msg);

    // Log ke terminal
    // std::cout << "Sensor_1: " << distance1 << " mm, Sensor_2: " << distance2 << " mm, Sensor_3: " << distance3 << " mm, sensor_4:" << distance4 << "mm" << std::endl;
}

// Fungsi setup BNO055
void setMode(int fd, uint8_t mode)
{
    std::lock_guard<std::mutex> lock(i2c_mutex); // Kunci mutex
    uint8_t buffer[2] = {BNO055_OPR_MODE_ADDR, mode};
    if (write(fd, buffer, 2) != 2)
    {
        ROS_ERROR("Failed to set mode.");
    }
    // usleep(100000); // Tunggu mode diatur
}

void readEulerAngles(int fd, int32_t &yaw, int32_t &roll, int32_t &pitch)
{
    std::lock_guard<std::mutex> lock(i2c_mutex); // Kunci mutex
    uint8_t reg[1] = {BNO055_EULER_H_ADDR};
    if (write(fd, reg, 1) != 1)
    {
        ROS_ERROR("Failed to write to I2C.");
        return;
    }

    uint8_t buffer[6];
    if (read(fd, buffer, 6) != 6)
    {
        ROS_ERROR("Failed to read from I2C.");
        return;
    }

    // Convert raw data to Euler angles in degrees and round to the nearest integer
    yaw = static_cast<int32_t>((buffer[0] | (buffer[1] << 8)) / 16.0 + 0.5);
    roll = static_cast<int32_t>((buffer[2] | (buffer[3] << 8)) / 16.0 + 0.5);
    pitch = static_cast<int32_t>((buffer[4] | (buffer[5] << 8)) / 16.0 + 0.5);

    // Normalize angles to be within the range [0, 360)
    yaw = (yaw + 360) % 360;
    roll = (roll + 360) % 360;
    pitch = (pitch + 360) % 360;
}

// Fungsi pembacaan VL53L0X
void vl53l0x_loop(int file1, int file2, int file3, int file4, ros::Publisher &distance_pub)
{
    while (ros::ok())
    {
        measure_distance(file1, file2, file3, file4, distance_pub);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Fungsi pembacaan BNO055
void bno055_loop(int fd, ros::Publisher &euler_pub, int32_t initial_yaw, int32_t initial_roll, int32_t initial_pitch, ros::Rate &rate)
{
    while (ros::ok())
    {
        int32_t yaw, roll, pitch;
        readEulerAngles(fd, yaw, roll, pitch);

        // Calculate relative angles
        yaw = (yaw - initial_yaw + 360) % 360;
        roll = (roll - initial_roll + 360) % 360;
        pitch = (pitch - initial_pitch + 360) % 360;

        // Create message for Euler angles
        std_msgs::Int32MultiArray euler_array;
        euler_array.data = {yaw, roll, pitch};

        // Publish data
        euler_pub.publish(euler_array);

        rate.sleep();
    }
}

void set_pwm_freq(int file, float freq) {
    uint8_t oldmode = 0x10;
    write_register(file, MODE1, oldmode);
    uint8_t prescale = static_cast<uint8_t>(std::round(25000000.0 / (4096.0 * freq)) - 1);
    write_register(file, PRESCALE, prescale);
    write_register(file, MODE1, 0x00);
    usleep(500);
    write_register(file, MODE1, 0xA1);
}

void set_servo_pwm(int file, uint8_t channel, uint16_t on, uint16_t off) {
    write_register(file, LED0_ON_L + 4 * channel, on & 0xFF);
    write_register(file, LED0_ON_L + 4 * channel + 1, on >> 8);
    write_register(file, LED0_ON_L + 4 * channel + 2, off & 0xFF);
    write_register(file, LED0_ON_L + 4 * channel + 3, off >> 8);
}

uint16_t map_angle_to_pwm(float angle) {
    float min_pulse = 180;
    float max_pulse = 630;
    return static_cast<uint16_t>((angle / 180.0) * (max_pulse - min_pulse) + min_pulse);
}

void servoCallback(const std_msgs::Float32::ConstPtr& msg, int servo_id) {
    float angle = msg->data;
    //ROS_INFO("Servo %d menerima sudut: %f", servo_id + 1, angle);
    if (angle < 0 || angle > 180) {
        //ROS_ERROR("Servo %d: Sudut di luar rentang!", servo_id + 1);
        return;
    }
    set_servo_pwm(file, servo_id, 0, map_angle_to_pwm(angle));
    //ROS_INFO("Servo %d diatur ke sudut: %f derajat", servo_id + 1, angle);
}


int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "sensor_node");
    ros::NodeHandle nh;

    // ROS Publishers
    ros::Publisher distance_pub = nh.advertise<std_msgs::UInt16MultiArray>("sensor_distance", 10);
    ros::Publisher euler_pub = nh.advertise<std_msgs::Int32MultiArray>("imu/euler", 10);
    ros::Subscriber servo1_sub = nh.subscribe<std_msgs::Float32>("servo1", 10, boost::bind(servoCallback, _1, 0));
    ros::Subscriber servo2_sub = nh.subscribe<std_msgs::Float32>("servo2", 10, boost::bind(servoCallback, _1, 1));
    ros::Subscriber servo3_sub = nh.subscribe<std_msgs::Float32>("servo3", 10, boost::bind(servoCallback, _1, 2));

    // Set up BNO055 I2C communication
    int fd_bno055 = i2c_init("/dev/i2c-1", BNO055_ADDRESS);
    if (fd_bno055 < 0)
    {
        std::cerr << "Failed to initialize BNO055 sensor." << std::endl;
        return -1;
    }

    // Set BNO055 to NDOF mode
    setMode(fd_bno055, 0x08); // Set the mode to NDOF (9DoF fusion mode)

    // Set Servo
   file = i2c_init("/dev/i2c-1", PCA9685_I2C_ADDR);
    if (file < 0) {
        return 1;
    }

    set_pwm_freq(file, 50.0);
    // Set up VL53L0X sensors
    setup_sensors();
    int file1 = i2c_init("/dev/i2c-1", LOX1_ADDRESS);
    int file2 = i2c_init("/dev/i2c-1", LOX2_ADDRESS);
    int file3 = i2c_init("/dev/i2c-1", LOX3_ADDRESS);
    int file4 = i2c_init("/dev/i2c-1", LOX4_ADDRESS);
    if (file1 < 0 || file2 < 0 || file3 < 0 || file4 < 0)
    {
        std::cerr << "Failed to initialize one or more VL53L0X sensors." << std::endl;
        return -1;
    }

    // Get initial orientation from BNO055
    int32_t initial_yaw, initial_roll, initial_pitch;
    readEulerAngles(fd_bno055, initial_yaw, initial_roll, initial_pitch);

    // Set ROS rate for sensor loops
    ros::Rate loop_rate(10);  // 10 Hz

    // Launch threads for VL53L0X and BNO055 reading
    std::thread vl53l0x_thread(vl53l0x_loop, file1, file2, file3, file4, std::ref(distance_pub));
    std::thread bno055_thread(bno055_loop, fd_bno055, std::ref(euler_pub), initial_yaw, initial_roll, initial_pitch, std::ref(loop_rate));

    // Wait for ROS shutdown signal
    while (ros::ok() && keep_running)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Clean up threads and GPIO on shutdown
    keep_running = false;
    vl53l0x_thread.join();
    bno055_thread.join();

    // Close file descriptors for sensors
    close(fd_bno055);
    close(file);
    close(file1);
    close(file2);
    close(file3);
    close(file4);

    // Cleanup GPIO
    GPIO::cleanup();
    return 0;
}