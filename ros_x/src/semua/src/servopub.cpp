#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <csignal>
#include <thread>
#include <atomic>
#include <chrono>

ros::Publisher angle_pub[3];
std::atomic<bool> running(true);
std::atomic<int> angles[3];

void signalHandler(int signum) {
    ROS_INFO("SIGINT diterima. Menghentikan program...");
    running = false;
}

void autoMoveServos() {
    int angle = 90;
    int angle1 = 20;
    while (ros::ok() && running) {
        angles[0] = angle;
        
    }
}

void publishServoAngles() {
    while (ros::ok() && running) {
        for (int i = 0; i < 3; ++i) {
            std_msgs::Float32 msg;
            msg.data = angles[i];
            angle_pub[i].publish(msg);
            ROS_INFO("Servo %d sudut dipublikasikan: %f", i + 1, msg.data);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "servo_publisher");
    ros::NodeHandle nh;
    signal(SIGINT, signalHandler);
    
    angle_pub[0] = nh.advertise<std_msgs::Float32>("servo1", 10);
    angle_pub[1] = nh.advertise<std_msgs::Float32>("servo2", 10);
    angle_pub[2] = nh.advertise<std_msgs::Float32>("servo3", 10);

    std::thread servo_thread(autoMoveServos);
    servo_thread.detach();

    ros::Rate loop_rate(10);
    while (ros::ok() && running) {
        publishServoAngles();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Program dihentikan.");
    return 0;
}