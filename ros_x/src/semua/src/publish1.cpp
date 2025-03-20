#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    std::string input;
    while (ros::ok()) {
        // // Minta input dari pengguna
        // std::cout << "Masukkan perintah (1: maju, 2: mundur, 3: kanan, q: keluar): ";
        // std::getline(std::cin, input);  // Menggunakan std::getline untuk membaca input

        // // Cek apakah pengguna ingin keluar
        // if (input == "q") {
        //     break;
        // }

        std_msgs::String msg;
        msg.data = input;  // Menyimpan input ke dalam pesan

        // ROS_INFO("Mengirim pesan: %s", msg.data.c_str());
        chatter_pub.publish(msg);

        ros::spinOnce();
        // loop_rate.sleep();
    }

    return 0;
}
