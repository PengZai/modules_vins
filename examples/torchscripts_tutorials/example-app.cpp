#include <torch/script.h> // One-stop header for TorchScript
#include <opencv2/opencv.hpp> // OpenCV
#include <iostream>
#include <memory>
#include "YoloDetect.h"




int main(int argc, const char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: example-app <path-to-exported-script-module> <path-to-image>\n";
        return -1;
    }


    YoloDetection yolo(argv[1]);
    std::cout << "Model loaded successfully!\n";

    // Load image using OpenCV
    cv::Mat image = cv::imread(argv[2], cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Error loading image: " << argv[2] << "\n";
        return -1;
    }
    std::cout << "Image loaded successfully!\n";

    yolo.Detect(image);




    return 0;
}
