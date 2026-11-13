"""
Starter-kit peripherals: RGB status LED, active buzzer (through a 2N2222/S8050
NPN transistor), two push buttons and the 0.96" SSD1306 OLED.

Everything degrades gracefully to a no-op ``Mock*`` implementation when the
libraries or the hardware are missing (laptop, CI, simulator), so the same
code runs everywhere. Wiring: docs/HARDWARE_WIRING.md
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from ..config import HardwareConfig

COLORS = {
    "OFF": (0, 0, 0), "GREEN": (0, 1, 0), "YELLOW": (1, 0.6, 0), "ORANGE": (1, 0.25, 0),
    "RED": (1, 0, 0), "BLUE": (0, 0, 1), "WHITE": (1, 1, 1), "CYAN": (0, 1, 1), "MAGENTA": (1, 0, 1),
}

# buzzer pattern -> list of (on_s, off_s); repeated
BUZZER_PATTERNS = {
    "OFF": None,
    "INTERMITTENT": [(0.15, 0.85)],
    "FAST_PULSE": [(0.10, 0.15)],
    "CONTINUOUS": [(1.0, 0.0)],
    "CHIRP": [(0.05, 0.05), (0.05, 1e9)],   # one-shot acknowledgement
}


def _want(flag: str) -> Optional[bool]:
    return {"yes": True, "no": False}.get(str(flag).lower())


class StatusIndicators:
    """Non-blocking LED + buzzer pattern player (runs in its own thread)."""

    def __init__(self, cfg: HardwareConfig, role: str = "node"):
        self.cfg = cfg
        self.role = role
        self.led_name = "OFF"
        self.buzz_name = "OFF"
        self.muted = False
        self._led = self._buzzer = None
        self.backend = "mock"
        want = _want(cfg.gpio_enabled)
        if want is not False:
            try:
                from gpiozero import Buzzer, RGBLED
                p = cfg.pins
                self._led = RGBLED(p.led_r, p.led_g, p.led_b, active_high=not p.common_anode)
                self._buzzer = Buzzer(p.buzzer) if cfg.buzzer_enabled else None
                self.backend = "gpiozero"
            except Exception as exc:
                if want:
                    raise
                print(f"[{role}] GPIO unavailable ({exc.__class__.__name__}); using mock indicators")
        self._stop = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def set(self, led: str, buzzer: str) -> None:
        self.led_name, self.buzz_name = led, buzzer

    def chirp(self) -> None:
        self._chirp_until = time.monotonic() + 0.1

    def _apply_led(self, on: bool) -> None:
        if self._led is None:
            return
        base = self.led_name.replace("_BLINK", "")
        rgb = COLORS.get(base, (1, 0, 1)) if on else (0, 0, 0)
        try:
            self._led.color = rgb
        except Exception:
            pass

    def _apply_buzzer(self, on: bool) -> None:
        if self._buzzer is None:
            return
        try:
            self._buzzer.on() if on else self._buzzer.off()
        except Exception:
            pass

    def _run(self) -> None:
        t0 = time.monotonic()
        self._chirp_until = 0.0
        while not self._stop:
            now = time.monotonic() - t0
            blink_on = (not self.led_name.endswith("_BLINK")) or (now % 0.5 < 0.25)
            self._apply_led(blink_on)
            pat = None if self.muted else BUZZER_PATTERNS.get(self.buzz_name)
            on = False
            if pat:
                on_s, off_s = pat[0]
                on = (now % (on_s + off_s)) < on_s
            if time.monotonic() < self._chirp_until:
                on = True
            self._apply_buzzer(on)
            time.sleep(0.02)

    def close(self) -> None:
        self._stop = True
        self._apply_buzzer(False)
        if self._led is not None:
            try:
                self._led.off()
            except Exception:
                pass


class Buttons:
    """Two active-low push buttons with debounce; callbacks run in gpiozero's thread."""

    def __init__(self, cfg: HardwareConfig, on_a: Callable[[], None], on_b: Callable[[], None],
                 hold_a: Optional[Callable[[], None]] = None):
        self.available = False
        if _want(cfg.gpio_enabled) is False:
            return
        try:
            from gpiozero import Button
            self.a = Button(cfg.pins.button_a, pull_up=True, bounce_time=0.05, hold_time=2.0)
            self.b = Button(cfg.pins.button_b, pull_up=True, bounce_time=0.05)
            self.a.when_pressed = on_a
            if hold_a:
                self.a.when_held = hold_a
            self.b.when_pressed = on_b
            self.available = True
        except Exception:
            pass


class Oled:
    """128x64 SSD1306 status screen (luma.oled). ``show(lines)`` is rate limited."""

    def __init__(self, cfg: HardwareConfig):
        self.dev = None
        self.last = 0.0
        if _want(cfg.oled_enabled) is False:
            return
        try:
            from luma.core.interface.serial import i2c
            from luma.oled.device import ssd1306
            self.dev = ssd1306(i2c(port=cfg.oled_i2c_bus, address=cfg.oled_address), width=128, height=64)
            from luma.core.render import canvas
            self._canvas = canvas
        except Exception:
            self.dev = None

    @property
    def available(self) -> bool:
        return self.dev is not None

    def show(self, title: str, lines: list[str], invert: bool = False) -> None:
        if self.dev is None or time.monotonic() - self.last < 0.2:
            return
        self.last = time.monotonic()
        try:
            with self._canvas(self.dev) as draw:
                if invert:
                    draw.rectangle((0, 0, 127, 15), fill="white")
                    draw.text((2, 2), title[:21], fill="black")
                else:
                    draw.text((2, 2), title[:21], fill="white")
                    draw.line((0, 14, 127, 14), fill="white")
                for i, ln in enumerate(lines[:4]):
                    draw.text((2, 17 + i * 12), ln[:21], fill="white")
        except Exception:
            pass

    def clear(self) -> None:
        if self.dev is not None:
            try:
                self.dev.clear()
            except Exception:
                pass
