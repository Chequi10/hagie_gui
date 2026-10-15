"""Video acquisition only: decoded RGB replaces a camera frame source."""
import argparse
import base64
import contextlib
import json
import math
from pathlib import Path
import sys
import time


def emit(message):
    print(json.dumps(message), flush=True)


def rotate_frame(frame, degrees, cv2):
    if degrees == 0:
        return frame
    codes = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
             270: cv2.ROTATE_90_COUNTERCLOCKWISE}
    return cv2.rotate(frame, codes[degrees])


def run(args):
    if not Path(args.video).is_file():
        raise ValueError('No existe el video seleccionado.')
    with contextlib.redirect_stdout(sys.stderr):
        import cv2
    capture = cv2.VideoCapture(args.video)
    try:
        if not capture.isOpened():
            raise ValueError('No se pudo abrir el video.')
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(fps) or fps <= 0:
            fps = 30.0
        index = 0
        while True:
            started = time.monotonic()
            ok, frame = capture.read()
            if not ok:
                if index == 0:
                    raise ValueError('El video no contiene fotogramas legibles.')
                break
            frame = rotate_frame(frame, args.rotation, cv2)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width = rgb.shape[:2]
            index += 1
            emit({'type': 'frame', 'frame': index, 'time_s': (index - 1) / fps,
                  'width': width, 'height': height,
                  'rgb': base64.b64encode(rgb.tobytes()).decode('ascii')})
            # Worker ACK includes inference and configured frame skipping.
            if sys.stdin.readline().strip() != 'next':
                return
            time.sleep(max(0.0, 1.0 / fps - (time.monotonic() - started)))
        emit({'type': 'finished', 'frames': index})
    finally:
        capture.release()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--video', required=True)
    parser.add_argument('--rotation', type=int, choices=[0, 90, 180, 270], default=0)
    args = parser.parse_args()
    try:
        run(args)
    except Exception as error:
        emit({'type': 'error', 'message': str(error)})
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
