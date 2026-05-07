# weight_to_onnx.py
# YOLO 모델을 ONNX 형식으로 변환하는 스크립트입니다.('.pt' -> '.onnx')

from ultralytics import YOLO

model = YOLO("weights/gy_best.pt")
# format='engine'으로 하면 TensorRT까지 한 번에 가지만,
# 정밀한 제어를 위해 onnx로 먼저 뽑는 것을 추천합니다.
model.export(format="onnx", opset=12, dynamic=True)