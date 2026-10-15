"""CPU detection backend. Receives RGB frames from Hagie's inference worker."""
import argparse
import base64
import contextlib
import json
import sys


def emit(message):
    print(json.dumps(message, allow_nan=False), flush=True)


def detect(model, request, np, cv2):
    width, height = int(request['width']), int(request['height'])
    if not 0 < width <= 16384 or not 0 < height <= 16384:
        raise ValueError('Dimensiones RGB inválidas.')
    confidence, iou = float(request['confidence']), float(request['iou'])
    if not 0 <= confidence <= 1 or not 0 <= iou <= 1:
        raise ValueError('Confianza o NMS inválidos.')
    data = base64.b64decode(request['rgb'], validate=True)
    if len(data) != width * height * 3:
        raise ValueError('Tamaño del RGB inválido.')
    rgb = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    with contextlib.redirect_stdout(sys.stderr):
        result = model.predict(source=bgr, imgsz=640, rect=False, conf=confidence,
                               iou=iou, device='cpu', verbose=False)[0]
    return {'type': 'result', 'timestamp_ms': str(request['timestamp_ms']),
            'boxes': [list(xyxy) + [float(score)]
                      for xyxy, score in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist())],
            'inference_ms': float(result.speed.get('inference', 0))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    args = parser.parse_args()
    try:
        with contextlib.redirect_stdout(sys.stderr):
            import cv2
            import numpy as np
            from ultralytics import YOLO
            model = YOLO(args.model, task='detect')
        emit({'type': 'ready'})
        for line in sys.stdin:
            emit(detect(model, json.loads(line), np, cv2))
    except Exception as error:
        emit({'type': 'error', 'message': str(error)})
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
