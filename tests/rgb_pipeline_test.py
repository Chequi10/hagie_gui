"""Video acquisition and ONNX protocol tests without weights or cameras."""
import base64
import contextlib
import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch


def module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1] / 'scripts' / (name + '.py'))
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded

source = module('video_rgb_source')
backend = module('onnx_inference')


class PipelineTest(unittest.TestCase):
    def test_source_emits_rgb_without_inference(self):
        with tempfile.TemporaryDirectory() as root:
            video = Path(root) / 'video with spaces.mp4'
            video.touch()
            class Frame:
                shape = (1, 2, 3)
                def tobytes(self): return bytes([10, 20, 30, 40, 50, 60])
            class Capture:
                remaining = 2
                released = False
                def isOpened(self): return True
                def get(self, key): return 25
                def read(self):
                    self.remaining -= 1
                    return (True, Frame()) if self.remaining >= 0 else (False, None)
                def release(self): self.released = True
            capture = Capture()
            cv = SimpleNamespace(VideoCapture=lambda _: capture, CAP_PROP_FPS=5,
                                 COLOR_BGR2RGB=1, cvtColor=lambda frame, _: frame)
            output = io.StringIO()
            with patch.dict('sys.modules', {'cv2': cv, 'ultralytics': None}), \
                    patch('sys.stdin', io.StringIO('next\nnext\n')), \
                    patch.object(source.time, 'sleep'), contextlib.redirect_stdout(output):
                source.run(SimpleNamespace(video=str(video), rotation=0))
            packets = [json.loads(line) for line in output.getvalue().splitlines()]
            self.assertEqual([p['type'] for p in packets], ['frame', 'frame', 'finished'])
            self.assertEqual(base64.b64decode(packets[0]['rgb']), Frame().tobytes())
            self.assertNotIn('boxes', packets[0])
            self.assertTrue(capture.released)

    def test_rotation_directions(self):
        calls = []
        cv = SimpleNamespace(ROTATE_90_CLOCKWISE='cw', ROTATE_180='half',
            ROTATE_90_COUNTERCLOCKWISE='ccw', rotate=lambda frame, code: calls.append(code))
        frame = object()
        self.assertIs(source.rotate_frame(frame, 0, cv), frame)
        for degrees in (90, 180, 270): source.rotate_frame(frame, degrees, cv)
        self.assertEqual(calls, ['cw', 'half', 'ccw'])

    def detector(self, bad_size=False, confidence=0.1, iou=0.45):
        raw = bytes([1, 2, 3, 4, 5, 6])
        request = {'width': 2, 'height': 1, 'rgb': base64.b64encode(raw if not bad_size else b'').decode(),
                   'timestamp_ms': '123', 'confidence': confidence, 'iou': iou}
        seen = {}
        class Array:
            def reshape(self, *shape):
                seen['shape'] = shape
                return 'rgb frame'
        def buffer(data, dtype):
            seen['bytes'] = data
            return Array()
        def color(rgb, code):
            self.assertEqual(rgb, 'rgb frame')
            self.assertEqual(code, 'RGB2BGR')
            return 'bgr frame'
        class Model:
            def predict(self, **kwargs):
                seen.update(kwargs)
                print('library log')
                return [SimpleNamespace(boxes=SimpleNamespace(
                    xyxy=SimpleNamespace(tolist=lambda: [[0, 0, 2, 1]]),
                    conf=SimpleNamespace(tolist=lambda: [0.8])), speed={'inference': 10})]
        np = SimpleNamespace(frombuffer=buffer, uint8='uint8')
        cv = SimpleNamespace(cvtColor=color, COLOR_RGB2BGR='RGB2BGR')
        logs = io.StringIO()
        with contextlib.redirect_stderr(logs):
            result = backend.detect(Model(), request, np, cv)
        return result, seen, logs.getvalue()

    def test_worker_parameters_and_camera_rgb_are_used(self):
        result, seen, logs = self.detector()
        self.assertEqual(seen['bytes'], bytes([1, 2, 3, 4, 5, 6]))
        self.assertEqual(seen['shape'], (1, 2, 3))
        self.assertEqual(seen['source'], 'bgr frame')
        self.assertEqual((seen['conf'], seen['iou'], seen['imgsz'], seen['rect']), (0.1, 0.45, 640, False))
        self.assertEqual(result['timestamp_ms'], '123')
        self.assertEqual(result['boxes'], [[0, 0, 2, 1, 0.8]])
        self.assertIn('library log', logs)

    def test_malformed_rgb_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Tamaño'): self.detector(bad_size=True)

    def test_invalid_threshold_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Confianza'): self.detector(confidence=-0.1)


if __name__ == '__main__': unittest.main()
