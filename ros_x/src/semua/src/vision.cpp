#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>

// Fungsi untuk mengontrol robot berdasarkan centerX
void controlRobot(float centerX) {
    if (centerX > 0 &&  centerX < 300) {
        ROS_INFO("Robot kekiri");
    } else if (centerX >500 && centerX < 800) {
        ROS_INFO("Robot kekanan");
    } else {
        ROS_INFO("Robot berhenti");
    }
}

void detectionCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received Detection: %s", msg->data.c_str());

    std::string detection_data = msg->data;

    // Variabel untuk menyimpan hasil ekstraksi
    float centerX = 0.0;
    float confidence = 0.0;

    // Mencari posisi "Korban" dan "Confidence:"
    std::size_t korban_pos = detection_data.find("Korban");
    std::size_t at_pos = detection_data.find("(");
    std::size_t confidence_pos = detection_data.find("Confidence:");

    if (korban_pos != std::string::npos && at_pos != std::string::npos && confidence_pos != std::string::npos) {
        // Ekstrak centerX
        std::string centerX_str = detection_data.substr(at_pos + 1, confidence_pos - at_pos - 3); // Mengambil nilai antara "(" dan ","
        try {
            centerX = std::stof(centerX_str); // Mengonversi string ke float
        } catch (const std::invalid_argument& e) {
            ROS_ERROR("Invalid centerX value: %s", centerX_str.c_str());
            return; // Keluar dari fungsi jika terjadi kesalahan
        } catch (const std::out_of_range& e) {
            ROS_ERROR("centerX value out of range: %s", centerX_str.c_str());
            return; // Keluar dari fungsi jika terjadi kesalahan
        }

        // Ekstrak confidence
        std::string confidence_str = detection_data.substr(confidence_pos + 12); // Mengambil nilai setelah "Confidence: "
        try {
            confidence = std::stof(confidence_str); // Mengonversi string ke float
        } catch (const std::invalid_argument& e) {
            ROS_ERROR("Invalid confidence value: %s", confidence_str.c_str());
            return; // Keluar dari fungsi jika terjadi kesalahan
        } catch (const std::out_of_range& e) {
            ROS_ERROR("Confidence value out of range: %s", confidence_str.c_str());
            return; // Keluar dari fungsi jika terjadi kesalahan
        }
        // Log hasil ekstraksi
        ROS_INFO("Center Korban: %.2f, Confidence: %.2f", centerX, confidence);

        // Panggil fungsi untuk mengontrol robot
        controlRobot(centerX);
    } else {
        ROS_WARN("Failed to parse detection data.");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "korban_subscriber");
    ros::NodeHandle nh;

    // Berlangganan ke topik "detections"
    ros::Subscriber sub = nh.subscribe("detections", 1000, detectionCallback);
    
    // Spin untuk menjaga node tetap hidup
    ros::spin();

    return 0;
}
