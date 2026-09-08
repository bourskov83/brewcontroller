"""
The fast (100ms) scan loop: read PVs, drive the cascade PID, push
heater/pump coil states out to the RTU slave.

This was previously one ~70-line method on SoftPLC. It's split here into
one method per concern (`_update_process_values`, `_update_setpoints`,
`_run_pid`, `_update_coils`) called in sequence from `run`, so each step
can be read/tested/changed on its own.
"""
import asyncio
from typing import Dict

from logger_config import log
from plc.scaling import reg_to_float, float_to_reg
from plc.sensor_map import load_sensor_map
from plc.rate_of_change import RateOfChangeTracker
from plc.sensor_store import SensorStore, SensorStatus

import traceback

class ScanTask:
    """Owns the state and steps of the fast scan loop."""

    def __init__(self, tags, rtu, pid, ssr, store: SensorStore, scan_time: float = 0.1):
        self.tags = tags
        self.rtu = rtu
        self.pid = pid
        self.ssr = ssr
        self.store = store
        self.scan_time = scan_time

        # mode flags — mirrored from the bridge task, read-only here
        self.mlt_pid_auto = False
        self.hlt_pid_auto = False

        self._last_mlt_sp = 0
        self._last_hlt_sp = 0
        self._last_coil_state: Dict[str, list] = {}
        self._cycles_since_full_resync = 0

        # ------------------------------------------------------------------
        # Rate of Change tracker initalization
        # ------------------------------------------------------------------


        DEFAULT_ROC_WINDOW = 30
        DEFAULT_ROC_MIN_SAMPLES = 10
        SENTINEL = -32768
        self._cycles_since_roc_update = 0
        self._roc_update_every_n_cycles = 8 # 8 scan time cycles is 800ms @ 100ms Scan time
        sensor_config = load_sensor_map() #load sensor config from sensors.yaml
        self._sensor_map = sensor_config  # kept for _update_sensor_readings()

        #create a dict of trackers 
        self._roc_trackers = {}
        for s in sensor_config:
            if s.roc_enabled:
                self._roc_trackers[s.sensor] = RateOfChangeTracker(
                    name=s.sensor,
                    window_seconds=getattr(s, "roc_window_seconds", DEFAULT_ROC_WINDOW),
                    min_samples=DEFAULT_ROC_MIN_SAMPLES,
                    sentinel=SENTINEL,
                )       


    async def run(self):
        log.info("scan_task started")
        loop = asyncio.get_event_loop()

        while True:
            t0 = loop.time()

            await self._update_sensor_readings()
            mlt_pv, hlt_pv = self._update_process_values()
            self._update_setpoints()
            hlt_output, ssr_state = self._run_pid(mlt_pv, hlt_pv)
            await self._update_coils()
            self._periodic_resync()
            self._update_rocs()
            self._update_diff_tags()
            self._calculate_chill_effective_ratio()

            elapsed = loop.time() - t0
            await asyncio.sleep(max(0, self.scan_time - elapsed))
            self._cycles_since_full_resync += 1

    # ------------------------------------------------------------------
    # Steps
    # ------------------------------------------------------------------

    async def _update_sensor_readings(self) -> None:
        """
        Single writer of {sensor}_temp tags and sensor_fault_word.

        Pulls whatever SensorIO.sensor_update_task has most recently
        published to the shared SensorStore. On OK, publishes the value
        to the tag as normal. On FAULT (ESP32 sentinel / sensor missing
        from the bus), the tag is left untouched — freezing the last
        good value — so the historian doesn't show a cliff-edge drop to
        the sentinel; the loss is instead visible via sensor_fault_word.
        """

        readings = await self.store.get_all()
        fault_word = 0
            
        for m in self._sensor_map:
            reading = readings.get(m.sensor)
            if reading is None or reading.status != SensorStatus.OK:
                # Not yet published (startup) or FAULT: flag it, leave the
                # tag frozen at its last good value.
                fault_word |= (1 << m.slot)
                continue
            self.tags.write(f"{m.sensor}_temp", int(reading.value))

        self.tags.write("sensor_fault_word", fault_word)

    def _update_process_values(self) -> tuple[float, float]:
        """
        Refresh mlt_pv / hlt_pv tags from the live sensor readings and
        return them as floats for the PID step.

        NOTE: original code set BOTH mlt_pv and hlt_pv from hlt_temp:
            self.tags.write("mlt_pv", self.tags.read("hlt_temp"))
            self.tags.write("hlt_pv", self.tags.read("hlt_temp"))
        That looks like a copy/paste bug (mlt_pv should likely come from
        mlt_temp) — flagging it here rather than silently "fixing" it,
        since I don't know if this was intentional during testing.
        Behavior is preserved as-is below; flip the commented line if
        the fix is wanted.
        """
        self.tags.write("mlt_pv", self.tags.read("hlt_temp"))  # see NOTE above
        # self.tags.write("mlt_pv", self.tags.read("mlt_temp"))  # <- likely correct
        self.tags.write("hlt_pv", self.tags.read("hlt_temp"))

        mlt_pv = reg_to_float(self.tags.read("mlt_pv")[0])
        hlt_pv = reg_to_float(self.tags.read("hlt_pv")[0])
        return mlt_pv, hlt_pv

    def _update_setpoints(self) -> None:
        """Pick up operator setpoint changes and push them into the PID."""
        mlt_sp = self.tags.read("mlt_sp")
        hlt_sp = self.tags.read("hlt_sp")

        if mlt_sp != self._last_mlt_sp:
            log.debug(f"mlt_sp changed from {self._last_mlt_sp} -> {mlt_sp}")
            self.pid.set_outer_setpoint(reg_to_float(mlt_sp[0]))
            self._last_mlt_sp = mlt_sp

        # Only take the operator's hlt_sp while MLT isn't in cascade/auto
        # mode — once MLT auto is on, the PID drives hlt_sp itself (see
        # _run_pid), so accepting operator writes here would fight it.
        if hlt_sp != self._last_hlt_sp and not self.mlt_pid_auto:
            log.debug(f"hlt_sp changed from {self._last_hlt_sp} -> {hlt_sp}")
            self.pid.set_inner_setpoint(reg_to_float(hlt_sp[0]))
            self._last_hlt_sp = hlt_sp

    def _run_pid(self, mlt_pv: float, hlt_pv: float) -> tuple[float, int]:
        """Run one cascade PID step and push the output + SSR state to tags."""
        hlt_output, hlt_auto_sp = self.pid.compute(outer_pv=mlt_pv, inner_pv=hlt_pv)

        if self.mlt_pid_auto:  # MLT is in AUTO -> PID owns the inner setpoint
            self.tags.write("hlt_sp", float_to_reg(hlt_auto_sp))
            self._last_hlt_sp = hlt_auto_sp

        self.tags.write("hlt_heater_output", int(hlt_output))

        ssr_state = self.ssr.update(int(hlt_output))
        self.tags.write("hlt_heater_state", ssr_state)

        return hlt_output, ssr_state

    async def _update_coils(self) -> None:
        """Push pump/heater coil states to the RTU slave if anything changed."""
        current_coil_state = {
            "mlt_pump": self.tags.read("mlt_pump"),
            "hlt_pump": self.tags.read("hlt_pump"),
            "bk_pump": self.tags.read("bk_pump"),
        }

        if current_coil_state == self._last_coil_state:
            return

        coil_values = []
        coil_values.extend(current_coil_state["mlt_pump"])
        coil_values.extend(current_coil_state["hlt_pump"])
        coil_values.extend(current_coil_state["bk_pump"])

        try:
            await self.rtu.write_coils(0, coil_values)
            log.debug("coils updated via RTU")
            self._last_coil_state = current_coil_state.copy()
        except asyncio.TimeoutError:
            log.warning("write timeout")
        except Exception as e:
            log.debug(e)

    def _periodic_resync(self) -> None:
        """Every N cycles, force a full coil resync and log PID status."""
        if self._cycles_since_full_resync < 50:
            return
        log.debug(self.pid.status()['outer'])
        log.debug(self.pid.status()['inner'])
        log.debug(f"periodic coil resync after {self._cycles_since_full_resync} cycles")
        self._last_coil_state = {}
        self._cycles_since_full_resync = 0

    def _update_rocs(self) -> None:
        """Update the Rate of Change trackers for RoC-enabled sensors"""
        if self._cycles_since_roc_update < self._roc_update_every_n_cycles:
            self._cycles_since_roc_update += 1
            return
        
        self._cycles_since_roc_update = 0
        for name, tracker in self._roc_trackers.items():
            current_value = self.tags.read(f"{name}_temp")[0]
            try:
                roc_value = tracker.update(current_value)
                if roc_value is None:
                    reg = [-32768]
                else:
                    reg = float_to_reg(roc_value, scale=1)
                    
                self.tags.write(f"{name}_temp_roc", reg[0])

            except Exception as e:
                log.debug(f"{e}")
                log.debug(f"{traceback.format_exc()}")

    def _calculate_target_diff(self, target_temp: int, actual_temp: int) -> int: 
        """
        Calculate difference between target and actual temp, returns diff
        """
        if actual_temp is None:
            return None
        return actual_temp - target_temp


    def _calculate_chill_effective_ratio(self) -> None:
        """
        Calculate chiller effectiveness ratio
        """
        try:
            outlet = self.tags.read("chill_outlet_temp")[0]
            inlet = self.tags.read("chill_inlet_temp")[0]
            bk = self.tags.read("bk_temp")[0]

            dt_max = bk - inlet
            MIN_DT_MAX = 50  # raw units = 0.5°C at x100 scale

            if dt_max < MIN_DT_MAX:
               # log.debug(f"dt_max below threshold ({dt_max/100:.2f}°C) — skipping ratio calc")
                return

            ratio = (outlet - inlet) / dt_max
            self.tags.write("chill_effective_ratio", float_to_reg(ratio))

        except Exception as e:
            log.error(e)
            log.debug(f"{traceback.format_exc()}")



    def _update_diff_tags(self) -> None:
        """
        update diff tags for targets
        """
        try:
            self.tags.write("mash_target_diff", self._calculate_target_diff(self.tags.read("mash_target_temp")[0], self.tags.read("mlt_temp")[0]))
            self.tags.write("sparge_target_diff", self._calculate_target_diff(self.tags.read("sparge_target_temp")[0], self.tags.read("hlt_temp")[0]))
            self.tags.write("pitch_target_diff", self._calculate_target_diff(self.tags.read("pitch_target_temp")[0], self.tags.read("bk_temp")[0]))
            self.tags.write("chill_max_wort_delta", self._calculate_target_diff(self.tags.read("chill_inlet_temp")[0], self.tags.read("bk_temp")[0]))
            self.tags.write("chill_delta", self._calculate_target_diff(self.tags.read("chill_inlet_temp")[0], self.tags.read("chill_outlet_temp")[0]))


        except Exception as e:
            log.error(e)
            log.debug(f"{traceback.format_exc()}")

            