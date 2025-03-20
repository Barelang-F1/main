#include <ros/ros.h>
#include <std_msgs/String.h>

void detectionCallback(const std_msgs::String::ConstPtr& msg)
{
    // Process the received message
    std::string detection = msg->data;
    ROS_INFO("Received detection message: %s", detection.c_str());

    // You can further process or manipulate the data, like extracting the center position
    // and using it in further logic.
    // Example: Extract center position from the message (assuming it follows a certain format)
    // This assumes that the message contains a position in the format "Detected: Korban (x), Confidence: y"
    
    size_t pos = detection.find("Korban");
    if (pos != std::string::npos) {
        // Extract center X value, assuming it's a number in the format (x)
        size_t start = detection.find('(') + 1;
        size_t end = detection.find(')');
        std::string centerX = detection.substr(start, end - start);
        ROS_INFO("Center of Korban: %s", centerX.c_str());

        // Add further processing logic as needed (e.g., controlling a robot, sending commands, etc.)
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "detection_subscriber");
    ros::NodeHandle nh;

    // Create a subscriber to the "detections" topic
    ros::Subscriber detection_sub = nh.subscribe("detections", 1000, detectionCallback);

    // Spin to keep the node running and listening to the topic
    ros::spin();

    return 0;
}
