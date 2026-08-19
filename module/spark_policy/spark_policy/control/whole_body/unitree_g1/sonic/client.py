from __future__ import annotations

from typing import Any


class SonicClient:
    """Small JSON-over-ZMQ request client for a SONIC policy server."""

    def __init__(
        self,
        endpoint: str = "tcp://127.0.0.1:5560",
        timeout_ms: int = 50,
        linger_ms: int = 0,
    ) -> None:
        try:
            import zmq
        except Exception as exc:  # pragma: no cover - environment dependent.
            raise RuntimeError(
                "UnitreeG1SonicPolicy requires pyzmq in the SPARK environment. "
                "Install it with `conda install pyzmq` or `pip install pyzmq`."
            ) from exc

        self._zmq = zmq
        self.endpoint = str(endpoint)
        self.timeout_ms = int(timeout_ms)
        self.linger_ms = int(linger_ms)
        self._context = zmq.Context.instance()
        self._socket = None
        self._poller = None
        self._connect()

    def _connect(self) -> None:
        self.close_socket()
        self._socket = self._context.socket(self._zmq.REQ)
        self._socket.setsockopt(self._zmq.LINGER, self.linger_ms)
        self._socket.connect(self.endpoint)
        self._poller = self._zmq.Poller()
        self._poller.register(self._socket, self._zmq.POLLIN)

    def close_socket(self) -> None:
        if self._poller is not None and self._socket is not None:
            try:
                self._poller.unregister(self._socket)
            except Exception:
                pass
        if self._socket is not None:
            self._socket.close(self.linger_ms)
        self._socket = None
        self._poller = None

    def close(self) -> None:
        self.close_socket()

    def reconnect(self) -> None:
        """Discard any outstanding REQ state and open a fresh connection."""
        self._connect()

    def request(self, payload: dict[str, Any]) -> dict[str, Any] | None:
        if self._socket is None or self._poller is None:
            self._connect()

        try:
            self._socket.send_json(payload)
            events = dict(self._poller.poll(self.timeout_ms))
            if events.get(self._socket) != self._zmq.POLLIN:
                # REQ sockets cannot send again after a receive timeout.
                self._connect()
                return None
            response = self._socket.recv_json()
        except Exception:
            self._connect()
            return None

        if not isinstance(response, dict):
            return None
        return response
