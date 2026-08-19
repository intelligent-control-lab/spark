"""Process-isolated RGB-D display windows for simulator camera feedback."""

from __future__ import annotations

import importlib.util
import multiprocessing as mp
from queue import Empty, Full

import numpy as np


def _uint8_rgb(image) -> np.ndarray:
    array = np.asarray(image)
    if array.dtype == np.uint8:
        return array[..., :3]
    scale = 255.0 if array.size and float(np.nanmax(array)) <= 1.0 else 1.0
    return np.clip(array[..., :3] * scale, 0.0, 255.0).astype(np.uint8)


def _uint8_depth(image) -> np.ndarray | None:
    if image is None:
        return None
    array = np.asarray(image, dtype=np.float32)
    if array.ndim == 3 and array.shape[-1] == 1:
        array = array[..., 0]
    finite = np.isfinite(array)
    result = np.zeros(array.shape, dtype=np.uint8)
    if np.any(finite):
        near = float(np.min(array[finite]))
        far = float(np.max(array[finite]))
        if far > near:
            result[finite] = np.clip(
                255.0 * (array[finite] - near) / (far - near),
                0.0,
                255.0,
            ).astype(np.uint8)
    return result


def _camera_display_worker(frame_queue) -> None:
    """Own OpenCV/Qt completely outside an Isaac Kit process."""

    import cv2

    try:
        while True:
            try:
                feedback = frame_queue.get(timeout=0.1)
            except Empty:
                cv2.waitKey(1)
                continue
            if feedback is None:
                break
            for name, data in feedback.items():
                rgb = _uint8_rgb(data["rgb"])
                rgb_display = cv2.resize(
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                    (320, 240),
                    interpolation=cv2.INTER_AREA,
                )
                cv2.imshow(f"SPARK RGB Camera - {name}", rgb_display)
                depth = _uint8_depth(data.get("depth"))
                if depth is not None:
                    depth_display = cv2.resize(
                        depth,
                        (320, 240),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    cv2.imshow(f"SPARK Depth Camera - {name}", depth_display)
            cv2.waitKey(1)
    finally:
        cv2.destroyAllWindows()


class CameraDisplayProcess:
    """Keep only the newest RGB-D frame in a spawned OpenCV display process."""

    def __init__(self) -> None:
        if importlib.util.find_spec("cv2") is None:
            raise RuntimeError("OpenCV is unavailable")
        context = mp.get_context("spawn")
        self._queue = context.Queue(maxsize=1)
        self._process = context.Process(
            target=_camera_display_worker,
            args=(self._queue,),
            name="spark-camera-display",
        )
        self._process.start()

    @property
    def is_alive(self) -> bool:
        return self._process.is_alive()

    def submit(self, feedback) -> None:
        if not self.is_alive:
            raise RuntimeError("camera display process stopped")
        payload = {
            str(name): {
                "rgb": np.asarray(data["rgb"]).copy(),
                "depth": (None if data.get("depth") is None else np.asarray(data["depth"]).copy()),
            }
            for name, data in feedback.items()
            if data.get("rgb") is not None
        }
        if not payload:
            return
        try:
            self._queue.put_nowait(payload)
            return
        except Full:
            pass
        try:
            self._queue.get_nowait()
        except Empty:
            return
        try:
            self._queue.put_nowait(payload)
        except Full:
            pass

    def close(self) -> None:
        if self._process.is_alive():
            try:
                self._queue.put_nowait(None)
            except Full:
                try:
                    self._queue.get_nowait()
                except Empty:
                    pass
                try:
                    self._queue.put_nowait(None)
                except Full:
                    pass
            self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=2.0)
        self._queue.close()
        self._queue.cancel_join_thread()


def submit_camera_display(owner, feedback) -> bool:
    """Submit frames through an owner-held display process."""

    display = getattr(owner, "_camera_display_process", None)
    if display is None:
        try:
            display = CameraDisplayProcess()
        except RuntimeError:
            return False
        owner._camera_display_process = display
    try:
        display.submit(feedback)
    except RuntimeError:
        display.close()
        owner._camera_display_process = None
        return False
    return True


def close_camera_display(owner) -> None:
    """Stop an owner-held display process, if present."""

    display = getattr(owner, "_camera_display_process", None)
    if display is not None:
        display.close()
        owner._camera_display_process = None
