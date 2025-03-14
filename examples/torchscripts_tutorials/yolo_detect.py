import torch
from ultralytics import YOLO

# Create a new YOLO model from scratch
# model = YOLO("yolov5n.yaml")

device = "cuda" if torch.cuda.is_available() else "cpu"

# Load a pretrained YOLO model (recommended for training)
model = YOLO("yolov8n.pt")  # build from YAML and transfer weights
model.to(device)


# Train the model using the 'coco8.yaml' dataset for 3 epochs
# results = model.train(data="coco8.yaml", epochs=3, batch=4, workers=2)

# Evaluate the model's performance on the validation set
# results = model.val()


# Perform object detection on an image using the model
results = model("https://ultralytics.com/images/bus.jpg")
# results = model(example)

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    result.show()  # display to screen
    result.save(filename="result-detect.jpg")  # save to disk

# Export the model to ONNX format
success = model.export(device=device)
print('end')

