"""Optional active-viewport recording for Isaac simulation runners."""

from __future__ import annotations

from pathlib import Path


class IsaacViewportRecorder:
    """Capture scheduled Kit viewport frames to MP4 and/or GIF."""

    def __init__(
        self,
        *,
        video_path: str | None = None,
        gif_path: str | None = None,
        width: int = 960,
        height: int = 540,
        fps: float = 20.0,
    ) -> None:
        if not video_path and not gif_path:
            raise ValueError("At least one Isaac recording path is required")
        if width < 1 or height < 1 or fps <= 0.0:
            raise ValueError("Isaac recording width, height, and fps must be positive")
        try:
            import cv2
            import imageio.v2 as imageio
            from omni.kit.viewport.utility import get_active_viewport
        except ImportError as exc:
            raise RuntimeError(
                "Isaac recording requires the optional SPARK recording dependencies"
            ) from exc

        self._cv2 = cv2
        self._imageio = imageio
        self.video_path = Path(video_path).expanduser().resolve() if video_path else None
        self.gif_path = Path(gif_path).expanduser().resolve() if gif_path else None
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.frame_period = 1.0 / self.fps
        self.next_time = 0.0
        self.pending = False
        self.frames = 0
        self._gif_frames = []
        self._gif_frame_times = []
        self._closed = False
        self.viewport = get_active_viewport()
        if self.viewport is None:
            raise RuntimeError("Isaac recording requires an active Kit viewport")
        self.viewport.resolution = (self.width, self.height)
        self._writer = None
        if self.video_path is not None:
            self.video_path.parent.mkdir(parents=True, exist_ok=True)
            self._writer = cv2.VideoWriter(
                str(self.video_path),
                cv2.VideoWriter_fourcc(*"mp4v"),
                self.fps,
                (self.width, self.height),
            )
            if not self._writer.isOpened():
                raise RuntimeError(f"Could not open Isaac video output {self.video_path}")
        if self.gif_path is not None:
            self.gif_path.parent.mkdir(parents=True, exist_ok=True)

    @property
    def closed(self) -> bool:
        return self._closed

    def schedule(self, simulated_time: float) -> None:
        """Request a frame when the simulated recording clock reaches its deadline."""
        if self._closed or self.pending or simulated_time + 1.0e-12 < self.next_time:
            return
        from omni.kit.viewport.utility import capture_viewport_to_buffer

        self.pending = True
        capture_time = float(simulated_time)

        def on_capture(buffer, buffer_size, width, height, pixel_format):
            import numpy as np
            import omni.kit.renderer_capture

            pixels = omni.kit.renderer_capture.convert_raw_bytes_to_list(
                buffer, buffer_size, width, height, pixel_format
            )
            rgba = np.asarray(pixels, dtype=np.uint8).reshape(height, width, -1)
            rgb = rgba[..., :3]
            if (width, height) != (self.width, self.height):
                rgb = self._cv2.resize(rgb, (self.width, self.height))
            if self._writer is not None:
                self._writer.write(self._cv2.cvtColor(rgb, self._cv2.COLOR_RGB2BGR))
            if self.gif_path is not None:
                self._gif_frames.append(rgb.copy())
                self._gif_frame_times.append(capture_time)
            self.frames += 1
            self.next_time += self.frame_period
            self.pending = False

        capture_viewport_to_buffer(self.viewport, on_capture, is_hdr=False)

    def drain(self, render_callback, max_updates: int = 60) -> None:
        """Process Kit updates until the final asynchronous capture completes."""
        for _ in range(max(1, int(max_updates))):
            if not self.pending:
                return
            render_callback()
        if self.pending:
            raise RuntimeError("Isaac viewport capture did not complete before shutdown")

    def close(self) -> None:
        if self._closed:
            return
        if self.pending:
            raise RuntimeError("Drain the pending Isaac viewport capture before closing")
        self._closed = True
        if self._writer is not None:
            self._writer.release()
        if self.gif_path is not None:
            if not self._gif_frames:
                raise RuntimeError("Isaac viewport recording produced no GIF frames")
            # Kit viewport capture is asynchronous.  A busy renderer can miss
            # one or more nominal FPS deadlines, so fixed per-frame delays
            # would make the resulting GIF play faster than simulated time.
            # Preserve the timestamp of every completed request instead.
            frame_times = getattr(self, "_gif_frame_times", [])
            if len(self._gif_frames) > 1 and len(frame_times) == len(self._gif_frames):
                duration_ms = [
                    max(1.0, 1000.0 * (end - start))
                    for start, end in zip(frame_times, frame_times[1:])
                ]
                duration_ms.append(1000.0 * self.frame_period)
            else:
                # A single-frame capture and legacy callers that construct a
                # recorder without timestamp state retain ImageIO's scalar
                # duration contract.
                duration_ms = 1000.0 * self.frame_period
            self._imageio.mimsave(
                self.gif_path,
                self._gif_frames,
                format="GIF",
                duration=duration_ms,
                loop=0,
            )
