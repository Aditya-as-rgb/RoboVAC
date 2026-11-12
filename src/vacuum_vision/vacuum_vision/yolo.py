"""YOLO11 ONNX inference for the dirt detector. numpy + onnxruntime + Pillow only: no ROS, no OpenCV.

The exported model takes a 640x640 RGB tensor (1, 3, 640, 640), values 0..1, and returns
(1, 4 + classes, 8400): for every anchor the box centre x, y and size w, h (in letterboxed
pixels) followed by one score per class. There is no objectness value and no built-in NMS,
so both the letterbox undo and the NMS are done here.
"""

import ast
from dataclasses import dataclass
import os

import numpy as np

DEFAULT_NAMES = ('crumbs', 'dust', 'spill', 'hair')
PAD_VALUE = 114                    # grey used by Ultralytics when it letterboxes
MODEL_RELATIVE = os.path.join('models', 'dirt_detector', 'dirt_yolo11n.onnx')


@dataclass
class Detection:
    cls_id: int
    name: str
    conf: float
    x0: float                      # box in pixels of the ORIGINAL camera image
    y0: float
    x1: float
    y1: float

    @property
    def cx(self):
        return 0.5 * (self.x0 + self.x1)

    @property
    def cy(self):
        return 0.5 * (self.y0 + self.y1)

    @property
    def w(self):
        return self.x1 - self.x0

    @property
    def h(self):
        return self.y1 - self.y0


def letterbox(img, size=640):
    """Resize to fit ``size`` x ``size`` keeping the aspect ratio, pad with grey.

    Returns (canvas uint8 HxWx3, scale, left, top): a point (x, y) in ``img`` lands at
    (x * scale + left, y * scale + top) in the canvas.
    """
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    if (nw, nh) != (w, h):
        from PIL import Image
        img = np.asarray(Image.fromarray(img).resize((nw, nh), Image.BILINEAR))
    top = max(0, int(round((size - nh) / 2 - 0.1)))
    left = max(0, int(round((size - nw) / 2 - 0.1)))
    canvas = np.full((size, size, 3), PAD_VALUE, np.uint8)
    canvas[top:top + nh, left:left + nw] = img
    return canvas, scale, left, top


def preprocess(img_rgb, size=640):
    """uint8 RGB image -> (float32 tensor (1, 3, size, size), scale, left, top)."""
    canvas, scale, left, top = letterbox(img_rgb, size)
    x = canvas.astype(np.float32) / 255.0
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None]), scale, left, top


def nms(boxes, scores, iou_thr):
    """Indices kept by greedy non-maximum suppression. boxes (N, 4) as x0 y0 x1 y1."""
    if len(boxes) == 0:
        return []
    x0, y0, x1, y1 = boxes.T
    areas = np.clip(x1 - x0, 0, None) * np.clip(y1 - y0, 0, None)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        iw = np.clip(np.minimum(x1[i], x1[rest]) - np.maximum(x0[i], x0[rest]), 0, None)
        ih = np.clip(np.minimum(y1[i], y1[rest]) - np.maximum(y0[i], y0[rest]), 0, None)
        inter = iw * ih
        iou = inter / (areas[i] + areas[rest] - inter + 1e-9)
        order = rest[iou <= iou_thr]
    return keep


def decode(output, scale, left, top, width, height, conf_thr=0.25, iou_thr=0.5,
           names=DEFAULT_NAMES, max_det=50):
    """Raw model output -> list of Detection in original-image pixels, best first."""
    out = np.asarray(output, np.float32)
    if out.ndim == 3:
        out = out[0]
    if out.shape[0] < out.shape[1]:
        out = out.T                                    # (anchors, 4 + classes)
    scores = out[:, 4:]
    cls = scores.argmax(axis=1)
    conf = scores[np.arange(len(out)), cls]
    sel = conf >= conf_thr
    out, cls, conf = out[sel], cls[sel], conf[sel]
    if not len(out):
        return []
    cx, cy, w, h = out[:, 0], out[:, 1], out[:, 2], out[:, 3]
    boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)
    boxes[:, [0, 2]] = np.clip((boxes[:, [0, 2]] - left) / scale, 0, width)
    boxes[:, [1, 3]] = np.clip((boxes[:, [1, 3]] - top) / scale, 0, height)
    keep = []
    for c in np.unique(cls):                           # NMS inside each class only
        idx = np.nonzero(cls == c)[0]
        keep += [int(idx[k]) for k in nms(boxes[idx], conf[idx], iou_thr)]
    keep.sort(key=lambda i: -conf[i])
    dets = []
    for i in keep[:max_det]:
        c = int(cls[i])
        name = names[c] if c < len(names) else str(c)
        dets.append(Detection(c, name, float(conf[i]), *[float(v) for v in boxes[i]]))
    return dets


def find_model(path=''):
    """Resolve the ONNX file. ``path`` wins; else $VACUUM_DIRT_MODEL; else look for
    models/dirt_detector/dirt_yolo11n.onnx upwards from this file (works with
    ``colcon build --symlink-install``) and from the current directory."""
    if path:
        p = os.path.expanduser(path)
        if os.path.isfile(p):
            return p
        raise FileNotFoundError('dirt model not found: %s' % p)
    env = os.environ.get('VACUUM_DIRT_MODEL', '')
    if env and os.path.isfile(os.path.expanduser(env)):
        return os.path.expanduser(env)
    for start in (os.path.dirname(os.path.realpath(__file__)), os.getcwd()):
        d = start
        for _ in range(8):
            cand = os.path.join(d, MODEL_RELATIVE)
            if os.path.isfile(cand):
                return cand
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
    raise FileNotFoundError(
        'could not find %s. Pass model:=/full/path/dirt_yolo11n.onnx or set VACUUM_DIRT_MODEL.'
        % MODEL_RELATIVE)


class DirtDetector:
    def __init__(self, model_path, conf=0.25, iou=0.5, max_det=50, threads=2):
        try:
            import onnxruntime as ort
        except ImportError as e:                       # pragma: no cover
            raise ImportError(
                'onnxruntime is not installed for this Python. Run: '
                'pip install --user --break-system-packages onnxruntime "numpy<2"') from e
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = int(threads)
        self.session = ort.InferenceSession(model_path, opts, providers=['CPUExecutionProvider'])
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        self.size = inp.shape[2] if isinstance(inp.shape[2], int) else 640
        self.conf, self.iou, self.max_det = conf, iou, max_det
        self.names = DEFAULT_NAMES
        raw = self.session.get_modelmeta().custom_metadata_map.get('names')
        if raw:
            try:
                d = ast.literal_eval(raw)
                self.names = tuple(d[k] for k in sorted(d))
            except (ValueError, SyntaxError, KeyError):
                pass

    def detect(self, img_rgb):
        h, w = img_rgb.shape[:2]
        x, scale, left, top = preprocess(img_rgb, self.size)
        out = self.session.run(None, {self.input_name: x})[0]
        return decode(out, scale, left, top, w, h, self.conf, self.iou, self.names, self.max_det)
