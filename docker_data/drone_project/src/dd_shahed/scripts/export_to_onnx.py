from ultralytics import YOLO

model = YOLO("yolov11-shashed-model-weights-v1-640.pt")

model.export(
    format="onnx",
    imgsz=640,  # training-size
    opset=12,  # совместимый opset
    dynamic=False,
)
