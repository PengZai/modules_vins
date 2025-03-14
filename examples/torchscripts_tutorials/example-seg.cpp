#include <torch/script.h> // One-stop header for TorchScript
#include <opencv2/opencv.hpp> // OpenCV
#include <iostream>
#include <memory>

int main(int argc, const char* argv[]) {


  torch::jit::script::Module module;
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(argv[1]);
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    return -1;
  }

  std::cout << "ok\n";


  // Load image using OpenCV
  cv::Mat image = cv::imread(argv[2], cv::IMREAD_COLOR);
  if (image.empty()) {
      std::cerr << "Error loading image: " << argv[2] << "\n";
      return -1;
  }
  std::cout << "Image loaded successfully!\n";

  // Resize image to match the model input size
  cv::Mat resized_image;
  cv::resize(image, resized_image, cv::Size(640, 640));

  // Convert image to float tensor and normalize to [0,1]
  cv::Mat image_float;
  resized_image.convertTo(image_float, CV_32F, 1.0 / 255.0);

  // Convert OpenCV Mat to Torch Tensor
  torch::Tensor img_tensor = torch::from_blob(image_float.data, {1, 640, 640, 3}, torch::kFloat32);
  img_tensor = img_tensor.permute({0, 3, 1, 2}); // Change from HWC to CHW
  img_tensor = img_tensor.clone();  // Ensure contiguous memory

  auto output = module.forward({img_tensor}).toTuple();

  // Unpack tuple
  auto detections = output->elements()[0].toTensor();  // Bounding boxes
  auto masks = output->elements()[1].toTensor();       // Segmentation masks

  std::cout << "Detections: " << detections.sizes() << std::endl;
  std::cout << "Masks: " << masks.sizes() << std::endl;

  
}