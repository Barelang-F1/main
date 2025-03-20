#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOutput.h>
#include <jetson-inference/detectNet.h>
#include <jetson-inference/objectTracker.h>
#include <vector>
#include <algorithm>
#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <JetsonGPIO.h>

bool signal_received = false;
const float nmsThreshold = 0.5f;

// Function to compute IoU (Intersection over Union)
float IoU(const detectNet::Detection &boxA, const detectNet::Detection &boxB) {
    float xA = std::max(boxA.Left, boxB.Left);
    float yA = std::max(boxA.Top, boxB.Top);
    float xB = std::min(boxA.Right, boxB.Right);
    float yB = std::min(boxA.Bottom, boxB.Bottom);

    float interArea = std::max(0.0f, xB - xA) * std::max(0.0f, yB - yA);
    float boxAArea = (boxA.Right - boxA.Left) * (boxA.Bottom - boxA.Top);
    float boxBArea = (boxB.Right - boxB.Left) * (boxB.Bottom - boxB.Top);

    return interArea / (boxAArea + boxBArea - interArea);
}

// Function for NMS
std::vector<int> nonMaxSuppression(const detectNet::Detection *detections, int numDetections, float threshold) {
    std::vector<int> keepIndices;
    std::vector<bool> suppressed(numDetections, false);

    // Sort detections by confidence
    std::vector<std::pair<float, int>> confidenceIndex;
    for (int i = 0; i < numDetections; i++) {
        confidenceIndex.push_back({detections[i].Confidence, i});
    }
    std::sort(confidenceIndex.rbegin(), confidenceIndex.rend());

    for (const auto &pair : confidenceIndex) {
        int i = pair.second;
        if (suppressed[i]) continue;

        keepIndices.push_back(i);
        for (int j = i + 1; j < numDetections; j++) {
            if (IoU(detections[i], detections[j]) > threshold) {
                suppressed[j] = true; // Mark for suppression
            }
        }
    }
    return keepIndices;
}

void sig_handler(int signo) {
    if (signo == SIGINT) {
        signal_received = true;
    }
}

int usage() {
    printf("usage: detectnet [--help] [--network=NETWORK] [--threshold=THRESHOLD] ...\n");
    printf("                 input [output]\n\n");
    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle nh;
    ros::Publisher detection_pub = nh.advertise<std_msgs::String>("detections", 1000);
    
    int output_pin = 13;

    commandLine cmdLine(argc, argv);
    if (cmdLine.GetFlag("help"))
        return usage();

    if (signal(SIGINT, sig_handler) == SIG_ERR)
        LogError("can't catch SIGINT\n");

    videoOptions options;
    options.width = 800;
    options.height = 400;
    options.frameRate = 30;

    // Create input source (camera or video)
    videoSource *input = videoSource::Create("csi://0", options);
    if (!input) {
        LogError("detectnet:  failed to create input stream\n");
        return 1;
    }

    // Create output (display or save)
    videoOutput *output = videoOutput::Create(cmdLine, ARG_POSITION(1));
    if (!output) {
        LogError("detectnet:  failed to create output stream\n");
        return 1;
    }

    // Load detectNet model (only once at the start)
    detectNet *net = detectNet::Create(NULL, "/home/bf1/jetson-inference/python/training/detection/ssd/models/korban/ssd-mobilenet.onnx", NULL, "/home/bf1/jetson-inference/python/training/detection/ssd/models/korban/labels.txt", 0.5f, "input_0", "scores", "boxes");
    if (!net) {
        LogError("detectnet:  failed to load detectNet model\n");
        return 1;
    }

    // Set overlay options
    const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr(cmdLine.GetString("overlay", "box,labels,conf"));

    // Main inference loop
    while (!signal_received) {
        uchar3 *image = NULL;
        int status = 0;

        // Capture a frame from the input source
        if (!input->Capture(&image, &status)) {
        if (status == videoSource::TIMEOUT) {
            continue; // Coba lagi jika timeout
        } else if (status == videoSource::ERROR) {
            LogError("Failed to capture frame, retrying...\n");
            continue; // Coba lagi jika ada error
        }
        break;  // Jika bukan timeout atau error, keluar dari loop
    }

        GPIO::setmode(GPIO::BCM);
        GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);

        // Perform detection on the captured frame
        detectNet::Detection *detections = NULL;
        const int numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections, overlayFlags);
        
        // Apply Non-Maximum Suppression (NMS) to filter detections
        std::vector<int> keepIndices = nonMaxSuppression(detections, numDetections, nmsThreshold);

        std::ostringstream oss;
        if (!keepIndices.empty()) {
            // LogVerbose("%i objects detected after NMS\n", keepIndices.size());

            // Initialize variable to track the highest confidence for "Korban"
            float highestConfidence = 0.0f;
            int bestIndex = -1;

            // Find the highest confidence detection for "korban"
            for (int index : keepIndices) {
                const std::string class_name = net->GetClassDesc(detections[index].ClassID);
                if (class_name == "Korban" && detections[index].Confidence > highestConfidence) {
                    highestConfidence = detections[index].Confidence;
                    bestIndex = index;
                }
            }

            // If a valid "korban" detection with the highest confidence is found
            if (bestIndex >= 0) {
                float centerX = (detections[bestIndex].Left + detections[bestIndex].Right) / 2;
                float centerY = (detections[bestIndex].Top + detections[bestIndex].Bottom) / 2;
                //  LogVerbose("CenterX %i: (%.2f),CenterY : (%.2f), Confidence: %.2f \n", bestIndex, centerX, centerY, highestConfidence);
                oss << "Detected: korban (" << centerX << "), ("<< centerY <<") Confidence: " << highestConfidence << "\n";
                //LogVerbose("bounding box %i  (%.2f, %.2f) \n ",bestIndex, detections[bestIndex].Left,  detections[bestIndex].Right); 
                if (detections[bestIndex].TrackID >= 0) // Check if this is a tracked object
                    LogVerbose("tracking ID %i status=%i frames=%i lost=%i\n", detections[bestIndex].TrackID, detections[bestIndex].TrackStatus, detections[bestIndex].TrackFrames, detections[bestIndex].TrackLost);

                // Publish the result as a ROS message
                std_msgs::String msg;
                msg.data = oss.str();
                detection_pub.publish(msg);
            }
        }

        //Render output to display
        // if (output != NULL) {
        //     output->Render(image, input->GetWidth(), input->GetHeight());
        //     char str[256];
        //     sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS());
        //     output->SetStatus(str);

        //     if (!output->IsStreaming())
        //         break;
        // }

        // Optionally, print profiler times for performance analysis
        // net->PrintProfilerTimes();
    }

    LogVerbose("detectnet:  shutting down...\n");
    GPIO::output(output_pin, GPIO::LOW);
    // Cleanup
    SAFE_DELETE(input);
    SAFE_DELETE(output);
    SAFE_DELETE(net);

    LogVerbose("detectnet:  shutdown complete.\n");
    return 0;
}
