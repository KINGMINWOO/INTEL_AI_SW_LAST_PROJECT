# train_yolov8.py
from ultralytics import YOLO

# Load a pretrained YOLOv8x model
model = YOLO('yolov8x.pt')

# Train the model using the 'tomato_yolo_prepared.yaml' dataset
if __name__ == '__main__':
    results = model.train(
        data='tomato_yolo_prepared.yaml', 
        epochs=500, 
        imgsz=640, 
        batch=4,
        name='yolov8x_tomato',
        patience=10 # Early stopping patience
    )
