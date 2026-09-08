import time


class SSRController:
    def __init__(self, cycle_time=15.0):
        self.cycle_time = float(cycle_time)        # seconds
        self.monotonic = time.monotonic
        self._cycle_start = self.monotonic()       # cycle window reference
        self.last_output = 0                       # for diagnostics

    def update(self, duty_percent: float) -> bool:
        """
        Returns True = SSR ON
                False = SSR OFF
        duty_percent: 0.0 to 100.0
        """
        now = self.monotonic()
        elapsed = now - self._cycle_start

        # Start new cycle if needed
        if elapsed >= self.cycle_time:
            self._cycle_start = now
            elapsed = 0.0

        # Compute ON window
        on_time = (max(0.0, min(100.0, duty_percent)) / 100.0) * self.cycle_time

        # Decide ON/OFF
        ssr_on = elapsed < on_time

        self.last_output = ssr_on
        return ssr_on
