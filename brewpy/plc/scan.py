"""
TBD
"""
from logger_config import log
from plc.rate_of_change import RateOfChangeTracker
from plc.sensor_store import SensorStore, SensorStatus
import asyncio


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

    def __init__(self, tags, rtu, store: SensorStore, sensor_map, device, scan_time: float = 0.1):
        self.tags = tags
        self.rtu = rtu
        self.store = store
        self.scan_time = scan_time

        # mode flags — mirrored from the bridge task, read-only here
        self._coil_tags = device.coil_block()
        self._last_coils = None
        self._cycles_since_full_resync = 0

        # ------------------------------------------------------------------
        # Rate of Change tracker initalization
        # ------------------------------------------------------------------


        self._cycles_since_roc_update = 0
        self._sensor_map = sensor_map
        #create a dict of trackers 
        self._roc_trackers = {
            s.tag: RateOfChangeTracker(
                    name=s.tag,
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
            readings = await self.store.snapshot()
            fault_word = 0
                
            for m in self._sensor_map:
                reading = readings.get(m.tag)
                if reading is None or reading.status != SensorStatus.OK:
                    # Not yet published (startup) or FAULT: flag it, leave the
                    # tag frozen at its last good value.
                    log.error(f"sensor {reading} status={reading.status}" )
                    fault_word |= (1 << m.slot)
                    continue
                self.tags.set_raw(m.tag, reading.value)

            self.tags.set_raw("sensor_fault_word", fault_word)
        except Exception:
            log.exception("sensor reading update failed")


    async def _update_coils(self) -> None:
        current = [bool(self.tags.get_raw(n)) for n in self._coil_tags]
        if current == self._last_coils:
            return
        try:
            await self.rtu.write_coils(0, current)
            self._last_coils = current
        except Exception as e:
            log.warning(f"coil write failed: {e}")

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
        for tag, tracker in self._roc_trackers.items():
            roc = tracker.update(self.tags.get_eng(tag))
            if roc is None:
                self.tags.set_raw(f"{tag}_roc", SENTINEL)
            else:
                self.tags.set_eng(f"{tag}_roc", roc)

    @staticmethod
    def _delta(reference: int, actual: int) -> int:
        """actual - reference, or 0 when no reference is set (reference == 0)."""
        if reference == 0:
            return 0
        return actual - reference
 

    def _calculate_chill_effective_ratio(self) -> None:
        outlet, inlet, bk = (self.tags.get_raw(n) for n in ("chill_outlet_temp", "chill_inlet_temp", "bk_temp"))
        dt_max = bk - inlet
        if dt_max < 50:
            return
        self.tags.set_eng("chill_effective_ratio", (outlet - inlet) / dt_max)

    def _update_diff_tags(self) -> None:
        get, put = self.tags.get_raw, self.tags.set_raw
        put("mash_target_delta",    self._delta(get("mash_target_temp"),  get("mlt_temp")))
        put("sparge_target_delta",  self._delta(get("sparge_target_temp"), get("hlt_temp")))
        put("pitch_target_delta",   self._delta(get("pitch_target_temp"), get("bk_temp")))
        put("chill_max_wort_delta", self._delta(get("chill_inlet_temp"),  get("bk_temp")))
        put("chill_delta",          self._delta(get("chill_inlet_temp"),  get("chill_outlet_temp")))
