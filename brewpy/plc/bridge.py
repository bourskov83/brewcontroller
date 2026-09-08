"""
The slower (100ms-poll) "bridge" loop: watches command coils written by
an operator/HMI (read_roms, write_roms, mlt_pid_save, ...) and carries
out the corresponding action, then resets the coil.

Originally one long if-chain on SoftPLC. Restructured as a small command
table (`_COMMANDS`) of (flag tag -> handler) so adding a new command coil
later is "add one line to the table" instead of "add another if block".
"""
import asyncio
from pathlib import Path
from typing import Callable, Awaitable

from logger_config import log
from pid_store import PidParamsStore, PidParams
from plc.scaling import reg_to_float, float_to_reg

PID_PARAMS_PATH = Path("pid_params.yaml")


class BridgeTask:
    """Owns PID config loading + the command-coil polling loop."""

    def __init__(self, tags, pid, scan_task, poll_interval: float = 0.1):
        self.tags = tags
        self.pid = pid
        self.scan_task = scan_task  # to push mode flags through
        self.sensors = None  # set via attach_sensors(); avoids a circular import at module load
        self.poll_interval = poll_interval
        self.pid_store: PidParamsStore | None = None

        # Each entry: flag tag name -> async handler taking no args.
        # Handler is responsible for the actual work; this class resets
        # the flag tag back to False afterwards.
        self._commands: dict[str, Callable[[], Awaitable[None]]] = {
            "read_roms": self._handle_read_roms,
            "write_roms": self._handle_write_roms,
            "read_offsets": self._handle_read_offsets,
  #          "write_offsets": self._handle_write_offsets,
            "mlt_pid_save": self._handle_mlt_pid_save,
            "hlt_pid_save": self._handle_hlt_pid_save,
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
            for name, tag in tagmap.items():
                if tag.table == 1:  # coils table
                    self.tags.write(name, False)

            await self.sensors.read_sensor_roms()
            log.debug(f"Read sensor roms...")
#            await self.sensors.read_offsets()

            self.pid_store = PidParamsStore(PID_PARAMS_PATH)
            self.pid_store.load_from_disk(create_if_missing={
                "mlt": PidParams(p=0.0, i=0.0, d=0.0),
                "hlt": PidParams(p=0.0, i=0.0, d=0.0),
            })
            self._publish_pid_to_holding_regs("mlt", "mlt_pid")
            self._publish_pid_to_holding_regs("hlt", "hlt_pid")

            mlt_params = self.pid_store.get("mlt")
            hlt_params = self.pid_store.get("hlt")
            self.pid.set_outer_gains(mlt_params.p, mlt_params.i, mlt_params.d)
            self.pid.set_inner_gains(hlt_params.p, hlt_params.i, hlt_params.d)
            self.pid.set_inner_i_limits(-50, 50)
            self.pid.set_inner_i_clamp(True)
        except Exception as e:
            log.error(e)

    def _publish_pid_to_holding_regs(self, loop_name: str, tag_prefix: str) -> None:
        """Write scaled gains to <prefix>_Kp / _Ki / _Kd holding-register tags."""
        params = self.pid_store.get(loop_name)
        self.tags.write(f"{tag_prefix}_Kp", float_to_reg(params.p))
        self.tags.write(f"{tag_prefix}_Ki", float_to_reg(params.i))
        self.tags.write(f"{tag_prefix}_Kd", float_to_reg(params.d))

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

                self._sync_pid_modes()

                await asyncio.sleep(self.poll_interval)
        except Exception as e:
            log.error(e)
    def _sync_pid_modes(self) -> None:
        """Pick up mlt_pid_auto / hlt_pid_auto coil changes and push into the PID + scan task."""
        mlt_auto = self.tags.read("mlt_pid_auto")[0]
        if mlt_auto != self.scan_task.mlt_pid_auto:
            self.scan_task.mlt_pid_auto = mlt_auto
            self.pid.set_outer_mode(mlt_auto)
            log.debug(f"mlt_pid_auto: {mlt_auto}")

        hlt_auto = self.tags.read("hlt_pid_auto")[0]
        if hlt_auto != self.scan_task.hlt_pid_auto:
            self.scan_task.hlt_pid_auto = hlt_auto
            self.pid.set_inner_mode(hlt_auto)
            log.debug(f"hlt_pid_auto: {hlt_auto}")

    # ------------------------------------------------------------------
    # Command handlers
    # ------------------------------------------------------------------

    async def _handle_read_roms(self) -> None:
        await self.sensors.read_sensor_roms()

    async def _handle_write_roms(self) -> None:
        
        log.debug("write_roms")

        await self.sensors.write_sensor_roms()

    async def _handle_read_offsets(self) -> None:
        log.debug("read_offsets")
        await self.sensors.read_offsets()

    async def _handle_write_offsets(self) -> None:
        log.debug("write_offsets")
        await self.sensors.write_offsets()

    async def _handle_mlt_pid_save(self) -> None:
        p = reg_to_float(self.tags.read("mlt_pid_Kp")[0])
        i = reg_to_float(self.tags.read("mlt_pid_Ki")[0])
        d = reg_to_float(self.tags.read("mlt_pid_Kd")[0])
        log.debug(f"mlt PID params changed to Kp:{p}, Ki:{i}, Kd:{d}")
        self.pid.set_outer_gains(p, i, d)

    async def _handle_hlt_pid_save(self) -> None:
        p = reg_to_float(self.tags.read("hlt_pid_Kp")[0])
        i = reg_to_float(self.tags.read("hlt_pid_Ki")[0])
        d = reg_to_float(self.tags.read("hlt_pid_Kd")[0])
        log.debug(f"hlt PID params changed to Kp:{p}, Ki:{i}, Kd:{d}")
        self.pid.set_inner_gains(p, i, d)
