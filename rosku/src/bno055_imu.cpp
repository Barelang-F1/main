#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <mutex> // Tambahkan library mutex

#define BNO055_ADDRESS 0x28 // Alamat I2C BNO055
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_EULER_H_ADDR 0x1A
#define BNO055_EULER_R_ADDR 0x1C
#define BNO055_EULER_P_ADDR 0x1E

std::mutex i2c_mutex; // Mutex untuk melindungi akses I2C

void setMode(int fd, uint8_t mode) {
    std::lock_guard<std::mutex> lock(i2c_mutex); // Kunci mutex
    uint8_t buffer[2] = {BNO055_OPR_MODE_ADDR, mode};
    if (write(fd, buffer, 2) != 2) {
        ROS_ERROR("Failed to set mode.");
    }
    // usleep(100000); // Tunggu mode diatur
}

void readEulerAngles(int fd, int32_t &yaw, int32_t &roll, int32_t &pitch) {
    std::lock_guard<std::mutex> lock(i2c_mutex); // Kunci mutex
    uint8_t reg[1] = {BNO055_EULER_H_ADDR};
    if (write(fd, reg, 1) != 1) {
        ROS_ERROR("Failed to write to I2C.");
        return;
    }

    uint8_t buffer[6];
    if (read(fd, buffer, 6) != 6) {
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "bno055_imu_node");
    ros::NodeHandle nh;

    ros::Publisher euler_pub = nh.advertise<std_msgs::Int32MultiArray>("imu/euler", 10);

    // Open I2C file
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        ROS_ERROR("Failed to open I2C bus.");
        return -1;
    }

    // Set I2C address
    if (ioctl(fd, I2C_SLAVE, BNO055_ADDRESS) < 0) {
        ROS_ERROR("Failed to set I2C address.");
        close(fd);
        return -1;
    }

    setMode(fd, 0x08); // Set to IMU mode

    // Read initial angles to set as reference
    int32_t initial_yaw, initial_roll, initial_pitch;
    readEulerAngles(fd, initial_yaw, initial_roll, initial_pitch);

    ros::Rate rate(30); // 30 Hz

    while (ros::ok()) {
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

    close(fd);
    return 0;
}
