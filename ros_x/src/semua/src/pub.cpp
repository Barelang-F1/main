#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <thread>
#include <cstring>
#include <signal.h>
#include <JetsonGPIO.h>
#include <iomanip>

#define BNO055_ADDRESS 0x28
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_EULER_H_ADDR 0x1A
#define VL53L0X_DEFAULT_ADDRESS 0x29
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

#define SHT_LOX1 19 // GPIO untuk sensor 1
#define SHT_LOX2 13 // GPIO untuk sensor 2

static volatile bool keep_running = true;

void signalHandler(int signal)
{
    keep_running = false;
    std::cout << "Signal " << signal << " received. Cleaning up GPIO..." << std::endl;
    GPIO::cleanup();
}

void setMode(int fd, uint8_t mode)
{
    uint8_t buffer[2] = {BNO055_OPR_MODE_ADDR, mode};
    write(fd, buffer, 2);
    // usleep(100000);
}

void readEulerAngles(int fd, int32_t &yaw, int32_t &roll, int32_t &pitch)
{
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


// Fungsi untuk menulis ke register sensor
void write_register(int file, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    if (write(file, buffer, 2) != 2)
    {
        std::cerr << "Gagal menulis " << static_cast<int>(value) << " ke register " << static_cast<int>(reg) << std::endl;
    }
}

// Fungsi untuk membaca register dari sensor
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
void setup_sensors()
{
    GPIO::setmode(GPIO::BCM);
    GPIO::setup(SHT_LOX1, GPIO::OUT, GPIO::LOW);
    GPIO::setup(SHT_LOX2, GPIO::OUT, GPIO::LOW);

    // All reset
    GPIO::output(SHT_LOX1, GPIO::LOW);
    GPIO::output(SHT_LOX2, GPIO::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // All unreset
    GPIO::output(SHT_LOX1, GPIO::HIGH);
    GPIO::output(SHT_LOX2, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Aktifkan sensor 1 dan reset sensor 2
    GPIO::output(SHT_LOX1, GPIO::HIGH);
    GPIO::output(SHT_LOX2, GPIO::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Inisialisasi sensor 1
    int file1 = i2c_init("/dev/i2c-1", VL53L0X_DEFAULT_ADDRESS);
    if (file1 < 0)
    {
        std::cerr << "Gagal membuka sensor LOX1!" << std::endl;
        return;
    }
    write_register(file1, 0x8A, LOX1_ADDRESS); // Set alamat untuk sensor 1
    close(file1);

    // Sekarang hidupkan sensor 2
    GPIO::output(SHT_LOX2, GPIO::HIGH); // Hidupkan sensor 2
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Inisialisasi sensor 2
    int file2 = i2c_init("/dev/i2c-1", VL53L0X_DEFAULT_ADDRESS);
    if (file2 < 0)
    {
        std::cerr << "Gagal membuka sensor LOX2!" << std::endl;
        return;
    }
    write_register(file2, 0x8A, LOX2_ADDRESS); // Set alamat untuk sensor 2
    close(file2);
}

// Fungsi untuk membaca hasil pengukuran jarak
uint16_t read_distance(int file)
{
    uint8_t high_byte = read_register(file, 0x1E);
    uint8_t low_byte = read_register(file, 0x1F);
    return (high_byte << 8) | low_byte;
}
// Fungsi untuk memulai pengukuran jarak
void measure_distance(int file1, int file2, ros::Publisher &distance_pub)
{
    write_register(file1, 0x00, 0x01); // Memulai pengukuran untuk sensor 1
    write_register(file2, 0x00, 0x01); // Memulai pengukuran untuk sensor 2

    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Delay untuk hasil pengukuran

    uint16_t distance1 = read_distance(file1);
    uint16_t distance2 = read_distance(file2);

    // Publikasi data jarak dalam bentuk UInt16MultiArray
    std_msgs::UInt16MultiArray msg;
    msg.data.push_back(distance1);
    msg.data.push_back(distance2);
    distance_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "combined_node");
    ros::NodeHandle nh;

    // Publisher setup
    ros::Publisher euler_pub = nh.advertise<std_msgs::Int32MultiArray>("imu/euler", 10);
    ros::Publisher distance_pub = nh.advertise<std_msgs::UInt16MultiArray>("sensor_distance", 10);

    // Initialize I2C for BNO055
    int fd_bno = open("/dev/i2c-1", O_RDWR);
    if (fd_bno < 0 || ioctl(fd_bno, I2C_SLAVE, BNO055_ADDRESS) < 0)
    {
        ROS_ERROR("Failed to open I2C bus BNO055.");
        return -1;
    }
    setMode(fd_bno, 0x08); // Set to IMU mode

    signal(SIGINT, signalHandler);
    setup_sensors();

    int file1 = i2c_init("/dev/i2c-1", LOX1_ADDRESS);
    int file2 = i2c_init("/dev/i2c-1", LOX2_ADDRESS);

    if (file1 < 0 || file2 < 0)
    {
        std::cerr << "Gagal membuka sensor VL53L0X!" << std::endl;
        return 1;
    }

    // Read initial angles to set as reference
    int32_t initial_yaw, initial_roll, initial_pitch;
    readEulerAngles(fd_bno, initial_yaw, initial_roll, initial_pitch);

    ros::Rate loop_rate(33); // 10 Hz
    while (ros::ok() && keep_running)
    {
        // Publish example string message
        std_msgs::String msg;
        // msg.data = "Sample command";
        // chatter_pub.publish(msg);

        int32_t yaw, roll, pitch;
        readEulerAngles(fd_bno, yaw, roll, pitch);

        // Calculate relative angles
        yaw = (yaw - initial_yaw + 360) % 360;
        roll = (roll - initial_roll + 360) % 360;
        pitch = (pitch - initial_pitch + 360) % 360;

        // Publish IMU data
        sensor_msgs::Imu imu_msg;
        imu_msg.orientation.x = yaw;
        imu_msg.orientation.y = roll;
        imu_msg.orientation.z = pitch;

        // Create message for Euler angles
        std_msgs::Int32MultiArray euler_array;
        euler_array.data = {yaw, roll, pitch};

        // Create individual angle messages
        std_msgs::Int32 yaw_msg, roll_msg, pitch_msg;
        yaw_msg.data = yaw;
        roll_msg.data = roll;
        pitch_msg.data = pitch;

        // imu_pub.publish(imu_msg);
        euler_pub.publish(euler_array);
        // Read distance from VL53L0X
        measure_distance(file1, file2, distance_pub);
        //std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Delay pilihan untuk pembacaan berturut-turut

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Close I2C files
    close(fd_bno);
    close(file1);
    close(file2);
    GPIO::cleanup();
    return 0;
}