from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent


class _RendererSpy:
    def __init__(self):
        self.calls = []

    def update_scene(self, *args, **kwargs):
        self.calls.append((args, kwargs))


class _ConcreteMujocoAgent(MujocoAgent):
    def get_feedback(self):
        return {}


def _agent(*, recording_enabled, camera=None):
    agent = object.__new__(_ConcreteMujocoAgent)
    agent.renderer = _RendererSpy()
    agent.data = object()
    agent.recording_enabled = recording_enabled
    agent._record_camera = camera
    agent._render_scene_prepared = False
    agent._camera_overlay_geoms = []
    return agent


def test_interactive_render_omits_none_camera_argument():
    agent = _agent(recording_enabled=False)

    agent.begin_render_frame()

    assert agent.renderer.calls == [((agent.data,), {})]
    assert agent._render_scene_prepared


def test_recording_render_passes_configured_camera():
    camera = object()
    agent = _agent(recording_enabled=True, camera=camera)

    agent.begin_render_frame()

    assert agent.renderer.calls == [((agent.data,), {"camera": camera})]
    assert agent._render_scene_prepared


def test_begin_render_frame_retires_camera_overlay_after_capture():
    agent = _agent(recording_enabled=False)
    agent._camera_overlay_geoms.append({"type": "old-frame"})

    agent.begin_render_frame()

    assert agent._camera_overlay_geoms == []
