"""
Face-landmark back-ends.

* ``TasksBackend``  - MediaPipe Tasks FaceLandmarker (mediapipe >= 0.10). This is
  the only API left in current MediaPipe wheels: ``mp.solutions`` was removed
  in 0.10.3x, which is why revision 1 silently fell back to *constant fake values*.
* ``SolutionsBackend`` - legacy ``mp.solutions.face_mesh`` for old installs.

Both return pixel landmarks + optional 4x4 facial transformation matrix.
"""
from __future__ import annotations

import time
import urllib.request
from pathlib import Path
from typing import Optional

import numpy as np

from ..config import DriverMonitorConfig, resolve_path


def ensure_model(cfg: DriverMonitorConfig) -> Path:
    path = resolve_path(cfg.model_path)
    if not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        print(f"[node1] downloading face landmarker model -> {path}")
        urllib.request.urlretrieve(cfg.model_url, path)
    return path


class TasksBackend:
    name = "mediapipe-tasks"

    def __init__(self, cfg: DriverMonitorConfig):
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision

        self.mp = mp
        model = ensure_model(cfg)
        opts = vision.FaceLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(model)),
            running_mode=vision.RunningMode.VIDEO,
            num_faces=1,
            min_face_detection_confidence=0.5,
            min_face_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            output_face_blendshapes=False,
            output_facial_transformation_matrixes=True,
        )
        self.lm = vision.FaceLandmarker.create_from_options(opts)
        self._last_ts = 0

    def detect(self, frame_rgb: np.ndarray):
        h, w = frame_rgb.shape[:2]
        ts = int(time.monotonic() * 1000)
        if ts <= self._last_ts:
            ts = self._last_ts + 1
        self._last_ts = ts
        image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=np.ascontiguousarray(frame_rgb))
        res = self.lm.detect_for_video(image, ts)
        if not res.face_landmarks:
            return None, None
        pts = [(p.x * w, p.y * h) for p in res.face_landmarks[0]]
        mat = res.facial_transformation_matrixes[0] if res.facial_transformation_matrixes else None
        return pts, mat

    def close(self):
        self.lm.close()


class SolutionsBackend:
    name = "mediapipe-solutions"

    def __init__(self, cfg: DriverMonitorConfig):
        import mediapipe as mp
        self.fm = mp.solutions.face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True,
                                                  min_detection_confidence=0.5, min_tracking_confidence=0.5)

    def detect(self, frame_rgb):
        h, w = frame_rgb.shape[:2]
        res = self.fm.process(frame_rgb)
        if not res.multi_face_landmarks:
            return None, None
        return [(p.x * w, p.y * h) for p in res.multi_face_landmarks[0].landmark], None

    def close(self):
        self.fm.close()


def create_backend(cfg: DriverMonitorConfig):
    errors = []
    for cls in (TasksBackend, SolutionsBackend):
        try:
            return cls(cfg)
        except Exception as exc:  # pragma: no cover - depends on installed wheel
            errors.append(f"{cls.__name__}: {exc}")
    raise RuntimeError("No MediaPipe face-landmark backend available:\n  " + "\n  ".join(errors))
