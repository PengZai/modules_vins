
from ultralytics import YOLO
import torch


device = "cuda" if torch.cuda.is_available() else "cpu"

model = YOLO("yolov8n-seg.pt")  # build from YAML and transfer weights
model.cuda()
model.eval()

# Export the model
model.export(device=device)