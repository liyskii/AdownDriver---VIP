"""Signal-processing helpers shared by both nodes."""
from __future__ import annotations

import collections
import statistics
import time


class EMAFilter:
    """x_f(t) = a * x(t) + (1 - a) * x_f(t-1)."""

    def __init__(self, alpha: float = 0.35, initial: float | None = None):
        self.alpha = alpha
        self.value = initial

    def update(self, x: float) -> float:
        self.value = float(x) if self.value is None else self.alpha * float(x) + (1 - self.alpha) * self.value
        return self.value

    def reset(self, value: float | None = None) -> None:
        self.value = value


class MedianFilter:
    def __init__(self, size: int = 3):
        self.buf: collections.deque = collections.deque(maxlen=size)

    def update(self, x: float) -> float:
        self.buf.append(float(x))
        return statistics.median(self.buf)


class PersistenceTimer:
    """Accumulates how long a condition has been continuously true."""

    def __init__(self):
        self.elapsed = 0.0

    def update(self, condition: bool, dt: float) -> float:
        self.elapsed = self.elapsed + dt if condition else 0.0
        return self.elapsed

    def reset(self) -> None:
        self.elapsed = 0.0


class WindowFraction:
    """Fraction of time a condition was true over a sliding window (e.g. PERCLOS)."""

    def __init__(self, window_s: float):
        self.window_s = window_s
        self.samples: collections.deque = collections.deque()
        self.total = 0.0
        self.true_time = 0.0

    def update(self, condition: bool, dt: float) -> float:
        self.samples.append((dt, condition))
        self.total += dt
        if condition:
            self.true_time += dt
        while self.samples and self.total - self.samples[0][0] >= self.window_s:
            d, c = self.samples.popleft()
            self.total -= d
            if c:
                self.true_time -= d
        return self.value

    @property
    def value(self) -> float:
        # Report 0 until at least a quarter of the window is filled to avoid start-up spikes
        if self.total < self.window_s * 0.25:
            return 0.0
        return max(0.0, self.true_time / self.total)

    def reset(self) -> None:
        self.samples.clear()
        self.total = self.true_time = 0.0


class RateTimer:
    """Fires at most every 1/hz seconds."""

    def __init__(self, hz: float):
        self.period = 1.0 / hz if hz > 0 else 0.0
        self.next_t = 0.0

    def ready(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        if now >= self.next_t:
            self.next_t = max(self.next_t + self.period, now) if self.next_t else now + self.period
            return True
        return False


class FpsMeter:
    def __init__(self, alpha: float = 0.1):
        self.last = None
        self.ema = EMAFilter(alpha)

    def tick(self, now: float | None = None) -> float:
        now = time.monotonic() if now is None else now
        if self.last is not None and now > self.last:
            self.ema.update(1.0 / (now - self.last))
        self.last = now
        return self.ema.value or 0.0
