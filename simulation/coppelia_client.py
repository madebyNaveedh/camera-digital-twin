"""
CoppeliaSim Remote API Client Wrapper.
Provides robust connection management and simulation controls over ZeroMQ.
"""

import time
import logging
from typing import Optional

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    RemoteAPIClient = None

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("CoppeliaClient")


class CoppeliaClient:
    def __init__(self, host: str = "localhost", port: int = 23000, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.client: Optional[RemoteAPIClient] = None
        self.sim = None
        self._is_connected: bool = False

    def connect(self) -> bool:
        """Connect to CoppeliaSim ZeroMQ Remote API server."""
        if RemoteAPIClient is None:
            logger.error("coppeliasim_zmqremoteapi_client is not installed.")
            return False

        try:
            logger.info(f"Connecting to CoppeliaSim ZeroMQ server at {self.host}:{self.port}...")
            self.client = RemoteAPIClient(host=self.host, port=self.port)
            self.client.setStepping(False)  # asynchronous non-blocking simulation by default
            self.sim = self.client.require("sim")
            
            # Verify communication
            version = self.sim.getInt32Param(self.sim.intparam_program_version)
            logger.info(f"Connected to CoppeliaSim successfully! Version code: {version}")
            self._is_connected = True
            return True
        except Exception as e:
            logger.warning(f"Could not connect to CoppeliaSim: {e}")
            self._is_connected = False
            self.client = None
            self.sim = None
            return False

    @property
    def is_connected(self) -> bool:
        return self._is_connected and self.sim is not None

    def start_simulation(self) -> bool:
        """Start the CoppeliaSim simulation."""
        if not self.is_connected:
            if not self.connect():
                return False
        try:
            state = self.sim.getSimulationState()
            if state == self.sim.simulation_stopped:
                self.sim.startSimulation()
                logger.info("Simulation started.")
            return True
        except Exception as e:
            logger.error(f"Error starting simulation: {e}")
            return False

    def stop_simulation(self) -> bool:
        """Stop the CoppeliaSim simulation."""
        if not self.is_connected:
            return False
        try:
            self.sim.stopSimulation()
            logger.info("Simulation stopped.")
            return True
        except Exception as e:
            logger.error(f"Error stopping simulation: {e}")
            return False

    def get_simulation_time(self) -> float:
        """Get current simulation time in seconds."""
        if not self.is_connected:
            return 0.0
        try:
            return float(self.sim.getSimulationTime())
        except Exception:
            return 0.0
