import logging
import time
import traceback
from typing import Any, Dict

import zmq
import zmq.asyncio
from openpi_client import msgpack_numpy
from openpi_client.base_policy import BasePolicy

logger = logging.getLogger(__name__)


class ZMQPolicyServer:
    """
    ZeroMQ PUB-SUB pattern policy server.
    Server publishes actions to subscribers (clients).
    """

    def __init__(
        self,
        policy: BasePolicy,
        host: str = "0.0.0.0",
        port: int | None = None,
        metadata: dict | None = None,
    ) -> None:
        self._policy = policy
        self._host = host
        self._port = port
        self._metadata = metadata or {}
        logging.getLogger("zmq.server").setLevel(logging.INFO)

        # ZeroMQ context
        self._ctx = zmq.asyncio.Context()
        self._pub_socket = None  # Publisher socket
        self._sub_socket = None  # Subscriber socket for receiving observations

    def serve_forever(self) -> None:
        """Synchronous entry point: run the event loop"""
        import asyncio
        asyncio.run(self.run())

    async def run(self) -> None:
        """Main async loop using PUB-SUB pattern"""
        # Create PUB socket for sending actions
        self._pub_socket = self._ctx.socket(zmq.PUB)
        pub_addr = f"tcp://{self._host}:{self._port}"
        self._pub_socket.bind(pub_addr)
        
        # Create SUB socket for receiving observations
        self._sub_socket = self._ctx.socket(zmq.SUB)
        sub_addr = f"tcp://{self._host}:{self._port + 1}"  # Different port for receiving
        self._sub_socket.bind(sub_addr)
        self._sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

        logger.info(f"ZeroMQ policy server PUBLISHING actions on {pub_addr}")
        logger.info(f"ZeroMQ policy server SUBSCRIBING to observations on {sub_addr}")

        prev_total_time = None
        while True:
            try:
                start_time = time.monotonic()

                # Wait for observation from client
                raw = await self._sub_socket.recv()
                obs = msgpack_numpy.unpackb(raw)

                # Process observation and get action
                infer_time = time.monotonic()
                action = self._policy.infer(obs)
                infer_time = time.monotonic() - infer_time

                # Add timing information
                action["server_timing"] = {
                    "infer_ms": infer_time * 1000,
                    "prev_total_ms": prev_total_time * 1000 if prev_total_time else None
                }

                # Publish action to all subscribers
                await self._pub_socket.send(msgpack_numpy.packb(action))
                prev_total_time = time.monotonic() - start_time

            except Exception:
                logger.exception("Inference error")
                # Publish error to subscribers
                await self._pub_socket.send(traceback.format_exc().encode())