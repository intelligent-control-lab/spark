"""Regression tests for GIF timing with ImageIO's Pillow backend."""

import numpy as np
import pytest

from spark_agent.simulation.isaac.viewport_recorder import IsaacViewportRecorder
from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent


class _ImageIORecorder:
    def __init__(self):
        self.calls = []

    def mimsave(self, *args, **kwargs):
        self.calls.append((args, kwargs))


class _ConcreteMujocoAgent(MujocoAgent):
    def get_feedback(self):
        return None


def test_isaac_gif_duration_is_supplied_in_milliseconds(tmp_path):
    imageio = _ImageIORecorder()
    recorder = IsaacViewportRecorder.__new__(IsaacViewportRecorder)
    recorder._closed = False
    recorder.pending = False
    recorder._writer = None
    recorder.gif_path = tmp_path / "isaac.gif"
    recorder._gif_frames = [np.zeros((2, 2, 3), dtype=np.uint8)]
    recorder._imageio = imageio
    recorder.frame_period = 0.1

    recorder.close()

    assert imageio.calls[0][1]["duration"] == 100.0


def test_mujoco_gif_duration_is_supplied_in_milliseconds(tmp_path, monkeypatch):
    imageio_v2 = pytest.importorskip("imageio.v2")
    imageio = _ImageIORecorder()
    monkeypatch.setattr(imageio_v2, "mimsave", imageio.mimsave)
    agent = _ConcreteMujocoAgent.__new__(_ConcreteMujocoAgent)
    agent._recording_closed = False
    agent._record_writer = None
    agent.record_gif_path = str(tmp_path / "mujoco.gif")
    agent._record_gif_frames = [np.zeros((2, 2, 3), dtype=np.uint8)]
    agent.record_fps = 10.0
    agent.recording_enabled = True
    agent._record_frame_count = 1

    agent._close_recording()

    assert imageio.calls[0][1]["duration"] == 100.0
