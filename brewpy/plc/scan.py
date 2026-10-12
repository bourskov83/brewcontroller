"""
TBD
"""
from logger_config import log
from plc.scaling import reg_to_float, float_to_reg
from plc.sensor_map import load_sensor_map
from plc.rate_of_change import RateOfChangeTracker
from plc.sensor_store import SensorStore, SensorStatus
import asyncio
import traceback


SENTINEL = -32768
DEFAULT_ROC_WINDOW = 30
DEFAULT_ROC_MIN_SAMPLES = 10

ROC_DEFAULT_WINDOW_S = 30
ROC_MIN_SAMPLES = 10
ROC_UPDATE_EVERY_N_CYCLES = 8  # 800 ms @ 100 ms scan time

COIL_RESYNC_EVERY_N_CYCLES = 50  # force a full coil rewrite (5 s @ 100 ms)

def _s16(raw: int) -> int:
    """Tags.write() masks to 16 bit, so reads come back unsigned. Make signed."""
    return raw - 0x10000 if raw & 0x8000 else raw


class ScanTask:
    """Owns the state and steps of the fast scan loop."""

    def __init__(self, tags, rtu, store: SensorStore, scan_time: float = 0.1):
        self.tags = tags
        self.rtu = rtu
        self.store = store
        self.scan_time = scan_time

        # mode flags — mirrored from the bridge task, read-only here
        self._last_coil_state: dict[str, list] = {}
        self._cycles_since_full_resync = 0

        # ------------------------------------------------------------------
        # Rate of Change tracker initalization
        # ------------------------------------------------------------------


        self._cycles_since_roc_update = 0
        self._sensor_map  =load_sensor_map() 
        #create a dict of trackers 
        self._roc_trackers = {
            s.sensor: RateOfChangeTracker(
                    name=s.sensor,
                    window_seconds=getattr(s, "roc_window_seconds", ROC_DEFAULT_WINDOW_S),
                    min_samples=ROC_MIN_SAMPLES,
                    sentinel=SENTINEL,
            )
        for s in self._sensor_map
            if s.roc_enabled
        }

    async def run(self):
        log.info("scan_task started")
        loop = asyncio.get_running_loop()

        # list of syncronous calls
        sync_steps = (
            self._periodic_resync,
            self._update_rocs,
            self._update_diff_tags,
            self._calculate_chill_effective_ratio,
        )



        while True:
            t0 = loop.time()

            await self._update_sensor_readings()
            await self._update_coils()

            # iterate over syncronoush calls            
            for step in sync_steps:
                try:
                    step()
                except Exception:
                    log.exception(f"scan step {step.__name__} failed")

            
            elapsed = loop.time() - t0
            await asyncio.sleep(max(0, self.scan_time - elapsed))


    def _read_s16(self, name: str) -> int:
        return _s16(self.tags.read(name)[0])

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
        try:
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
        except Exception:
            log.exception("sensor reading update failed")


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
        log.debug(f"periodic coil resync after {self._cycles_since_full_resync} cycles")
        self._last_coil_state = {}
        self._cycles_since_full_resync = 0

    def _update_rocs(self) -> None:
        """Update the Rate of Change trackers for RoC-enabled sensors"""
        if self._cycles_since_roc_update < ROC_UPDATE_EVERY_N_CYCLES:
            self._cycles_since_roc_update += 1
            return
        
        self._cycles_since_roc_update = 0
        for name, tracker in self._roc_trackers.items():
            current_value = self.tags.read(f"{name}_temp")[0]
            try:
                roc_value = tracker.update(current_value)
                if roc_value is None:
                    reg = SENTINEL
                else:
                    reg = float_to_reg(roc_value, scale=1)
                    
                self.tags.write(f"{name}_temp_roc", reg)

            except Exception as e:
                log.debug(f"{e}")
                log.debug(f"{traceback.format_exc()}")

    @staticmethod
    def _delta(reference: int, actual: int) -> int:
        """actual - reference, or 0 when no reference is set (reference == 0)."""
        if reference == 0:
            return 0
        return actual - reference
 

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
        update diff tags for targets and chiller deltas
        """
    
        r = self._read_s16
        w = self.tags.write
        w("mash_target_delta",      self._delta(self.tags.read("mash_target_temp")[0],      r("mlt_temp")))
        w("sparge_target_delta",    self._delta(self.tags.read("sparge_target_temp")[0],    r("hlt_temp")))
        w("pitch_target_delta",     self._delta(self.tags.read("pitch_target_temp")[0],     r("bk_temp")))
        w("chill_max_wort_delta",   self._delta(self.tags.read("chill_inlet_temp")[0],      r("bk_temp")))
        w("chill_delta",            self._delta(self.tags.read("chill_inlet_temp")[0],      r("chill_outlet_temp")))


        
            