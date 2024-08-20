from ultralytics import YOLO

# Build a YOLOv9c model from scratch
#model = YOLO('yolov9c.yaml')
# Build a YOLOv9c model from pretrained weight
model = YOLO("yolov8m-obb.pt")
# Train the model on the COCO8 example dataset for 100 epochs
results = model.train(data='data.yaml', epochs=100, imgsz=640, name='UR5_DATAmV1_100', device='0', batch=10, workers=20, patience=10)