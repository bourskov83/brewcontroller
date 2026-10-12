"""
"""
import asyncio
from pathlib import Path
from typing import Callable, Awaitable

from logger_config import log
from plc.scaling import reg_to_float, float_to_reg

PID_PARAMS_PATH = Path("pid_params.yaml")


class BridgeTask:
    """Owns PID config loading + the command-coil polling loop."""

    def __init__(self, tags, scan_task, poll_interval: float = 0.1):
        self.tags = tags
        self.scan_task = scan_task  # to push mode flags through
        self.sensors = None  # set via attach_sensors(); avoids a circular import at module load
        self.poll_interval = poll_interval

        # Each entry: flag tag name -> async handler taking no args.
        # Handler is responsible for the actual work; this class resets
        # the flag tag back to False afterwards.
        self._commands: dict[str, Callable[[], Awaitable[None]]] = {
        }

    def attach_sensors(self, sensors) -> None:
        """Wire in the SensorIO instance (kept separate to avoid a circular import)."""
        self.sensors = sensors

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------

    async def initialize(self, tagmap: dict) -> None:
        try:
            """One-time setup: zero coils, pull sensor config, load PID params."""
            log.debug("init")
            for name, tag in tagmap.items():
                if tag.table == 1:  # coils table
                    self.tags.write(name, False)


        except Exception as e:
            log.error(e)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    async def run(self, tagmap: dict):
        log.info("modbus_bridge_task started")
        try:
            await self.initialize(tagmap)
            

            while True:
                for flag_tag, handler in self._commands.items():
                    try:
                        if self.tags.read(flag_tag)[0]:
                            print(f"{flag_tag} received...")
                            await handler()
                            self.tags.write(flag_tag, False)
                    except Exception as e:
                        log.error(e)


                await asyncio.sleep(self.poll_interval)
        except Exception as e:
            log.error(e)



