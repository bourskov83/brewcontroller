from collections import deque
import time


class RateOfChangeTracker:
    """Tracks a rolling rate-of-change (units/min) for a single sensor.
    Sentinel readings (failed/missing sensor) are skipped, not recorded."""

    def __init__(self, name, window_seconds=60, min_samples=5, sentinel=-32768):
        self.name = name
        self.window_seconds = window_seconds
        self.min_samples = min_samples
        self.sentinel = sentinel
        self._history = deque()

    def update(self, value, timestamp=None):
        """Feed a new reading in, get back the current ROC (units/min) or None.
        Sentinel values are ignored entirely — they don't enter the buffer
        and don't count toward min_samples."""
        now = timestamp if timestamp is not None else time.time()

        if value == self.sentinel:
            # Don't record it, don't reset the window — just skip this tick.
            # A single missed sample shouldn't nuke an otherwise-good window.
            self._prune(now)
            return self._compute_slope() if len(self._history) >= self.min_samples else None

        self._history.append((now, value))
        self._prune(now)

        if len(self._history) < self.min_samples:
            return None

        return self._compute_slope()

    def _prune(self, now):
        while self._history and now - self._history[0][0] > self.window_seconds:
            self._history.popleft()

    def _compute_slope(self):
        n = len(self._history)
        mean_t = sum(t for t, _ in self._history) / n
        mean_v = sum(v for _, v in self._history) / n
        num = sum((t - mean_t) * (v - mean_v) for t, v in self._history)
        den = sum((t - mean_t) ** 2 for t, _ in self._history)
        if den == 0:
            return None
        return (num / den) * 60.0

    def reset(self):
        self._history.clear()