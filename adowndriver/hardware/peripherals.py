"""
Starter-kit peripherals: RGB status LED, active buzzer (through a 2N2222/S8050
NPN transistor), two push buttons and the 0.96" SSD1306 OLED.

Everything degrades gracefully to a no-op ``Mock*`` implementation when the
libraries or the hardware are missing (laptop, CI, simulator), so the same
code runs everywhere. Wiring: docs/HARDWARE_WIRING.md
"""
from __future__ import annotations

import shutil
import subprocess
import threading
import time
from typing import Callable, Optional

import numpy as np

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


# custom WAV per buzzer pattern (config hardware.audio_dir); played in a loop while the pattern is active
AUDIO_FILES = {"INTERMITTENT": "warning.wav", "FAST_PULSE": "slowing.wav", "CONTINUOUS": "stop.wav", "CHIRP": "ack.wav"}
HELLO_FILE = "hello.wav"                        # played once when the cockpit starts


def load_wav_mono(path, rate: int) -> Optional[np.ndarray]:
    """Read a PCM WAV (8/16/24/32-bit, any rate, mono/stereo) -> int16 mono at *rate*. None if unreadable."""
    import wave
    try:
        with wave.open(str(path), "rb") as w:
            ch, sw, sr, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
            raw = w.readframes(n)
    except Exception as exc:
        print(f"[audio] cannot read {path}: {exc}")
        return None
    if sw == 1:
        x = (np.frombuffer(raw, np.uint8).astype(np.float32) - 128) / 128
    elif sw == 2:
        x = np.frombuffer(raw, "<i2").astype(np.float32) / 32768
    elif sw == 3:
        b = np.frombuffer(raw, np.uint8).reshape(-1, 3)
        x = ((b[:, 0].astype(np.int32) | (b[:, 1].astype(np.int32) << 8) | (b[:, 2].astype(np.int32) << 16)) << 8 >> 8) / 8388608.0
        x = x.astype(np.float32)
    else:
        x = np.frombuffer(raw, "<i4").astype(np.float32) / 2147483648
    x = x.reshape(-1, ch).mean(axis=1)
    if sr != rate and x.size > 1:
        t_new = np.arange(int(x.size * rate / sr)) * (sr / rate)
        x = np.interp(t_new, np.arange(x.size), x).astype(np.float32)
    return (np.clip(x, -1, 1) * 32767).astype(np.int16)


# tone per buzzer pattern: the higher the urgency, the higher the pitch
AUDIO_TONES_HZ = {"INTERMITTENT": 1400.0, "FAST_PULSE": 2000.0, "CONTINUOUS": 2600.0, "CHIRP": 3000.0}


class AudioBuzzer:
    """Plays the buzzer patterns on a speaker through ``aplay`` (raw PCM fed in 20 ms blocks).

    The pattern clock stays in StatusIndicators; this class only renders "on/off + which tone".
    A long-running aplay process keeps latency at ~60 ms. Any failure -> silently disabled.
    """
    RATE = 22050
    BLOCK_S = 0.02

    def __init__(self, cfg: HardwareConfig, role: str):
        self.available = False
        self.on = False
        self.tone_hz = 2000.0
        self.pattern = "OFF"
        self.clips: dict = {}
        self._clip_name = None
        self._clip_pos = 0
        self._oneshot = None
        self._oneshot_pos = 0
        want = _want(cfg.audio_buzzer)
        if want is False or (want is None and role != "node1") or not shutil.which("aplay"):
            return
        device = cfg.audio_device
        if device == "auto":
            device = "default"
            try:
                cards = subprocess.run(["aplay", "-l"], capture_output=True, text=True, timeout=3).stdout
                if "Headphones" in cards:
                    device = "plughw:CARD=Headphones,DEV=0"
            except Exception:
                pass
        try:
            self.proc = subprocess.Popen(
                ["aplay", "-q", "-D", device, "-t", "raw", "-f", "S16_LE", "-c", "1",
                 "-r", str(self.RATE), "--buffer-time=60000"],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:
            print(f"[{role}] audio buzzer unavailable ({exc.__class__.__name__})")
            return
        self.device = device
        self.volume = max(0.0, min(1.0, float(cfg.audio_volume)))
        self._vol_file = None
        try:                                              # volume set from the dashboard survives restarts
            from ..config import resolve_path as _rp
            self._vol_file = _rp("config") / "volume.txt"
            if self._vol_file.is_file():
                self.volume = max(0.0, min(1.0, float(self._vol_file.read_text().strip())))
        except Exception:
            pass
        self.n = int(self.RATE * self.BLOCK_S)
        self.silence = np.zeros(self.n, dtype=np.int16).tobytes()
        self.phase = 0.0
        from ..config import resolve_path
        folder = resolve_path(cfg.audio_dir)
        gap = np.zeros(int(self.RATE * max(0.0, cfg.audio_gap_s)), np.int16)
        loaded: dict = {}                                  # identical files share one clip (see _block)
        for pat, fname in AUDIO_FILES.items():
            if (folder / fname).is_file():
                key = (folder / fname).read_bytes()
                if key not in loaded:
                    c = load_wav_mono(folder / fname, self.RATE)
                    loaded[key] = None if c is None or not c.size else (c if pat == "CHIRP" else np.concatenate([c, gap]))
                if loaded[key] is not None:
                    self.clips[pat] = loaded[key]
        if (folder / HELLO_FILE).is_file():
            hello = load_wav_mono(folder / HELLO_FILE, self.RATE)
            if hello is not None and hello.size:
                self._oneshot = hello
        if self.clips:
            print(f"[{role}] custom alert sounds: {', '.join(AUDIO_FILES[p] for p in self.clips)}")
        self.available = True
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"[{role}] audio buzzer on {device}")

    def _block(self) -> bytes:
        pat = self.pattern
        if self._oneshot is not None and pat in ("OFF", "CHIRP"):   # greeting; any alert cuts it off
            chunk = self._oneshot[self._oneshot_pos:self._oneshot_pos + self.n]
            self._oneshot_pos += self.n
            if self._oneshot_pos >= self._oneshot.size:
                self._oneshot = None
            if chunk.size < self.n:
                chunk = np.concatenate([chunk, np.zeros(self.n - chunk.size, np.int16)])
            return chunk.tobytes()
        self._oneshot = None
        if pat in self.clips and pat != "CHIRP":
            clip = self.clips[pat]
            if self._clip_name != pat:                    # new alert level -> start the sound from the beginning,
                same = self._clip_name in self.clips and self.clips[self._clip_name] is clip
                self._clip_name = pat                     # unless it is the same track (keep playing it)
                if not same:
                    self._clip_pos = 0
            idx = (self._clip_pos + np.arange(self.n)) % clip.size
            self._clip_pos = int((self._clip_pos + self.n) % clip.size)
            return clip[idx].tobytes()
        self._clip_name = None
        if pat == "CHIRP" and "CHIRP" in self.clips:
            return self.silence
        if not self.on:
            self.phase = 0.0
            return self.silence
        t = (np.arange(self.n) + self.phase) / self.RATE
        self.phase += self.n
        wave = np.sign(np.sin(2 * np.pi * self.tone_hz * t)) * 0.5 + 0.5 * np.sin(2 * np.pi * self.tone_hz * t)
        return (wave * 12000).astype(np.int16).tobytes()

    def set_volume(self, v: float) -> float:
        self.volume = max(0.0, min(1.0, float(v)))
        try:
            if self._vol_file is not None:
                self._vol_file.write_text(f"{self.volume:.2f}\n")
        except Exception:
            pass
        return self.volume

    def _scaled(self) -> bytes:
        raw = self._block()
        if self.volume >= 0.999 or raw is self.silence:
            return raw
        x = np.frombuffer(raw, np.int16).astype(np.float32) * self.volume
        return x.astype(np.int16).tobytes()

    def _loop(self) -> None:
        while self._running:
            try:
                self.proc.stdin.write(self._scaled())      # blocks ~20 ms per block (paced by the sound card)
                self.proc.stdin.flush()
            except Exception:
                self.available = False
                return

    def close(self) -> None:
        self._running = False
        self.on = False
        try:
            self.proc.stdin.close()
            self.proc.terminate()
        except Exception:
            pass


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
        self.audio = AudioBuzzer(cfg, role)
        if self.audio.available and self.backend == "mock":
            self.backend = "audio"
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
        if self.audio.available:
            self.audio.tone_hz = AUDIO_TONES_HZ.get(self.buzz_name, 3000.0)
            self.audio.pattern = "OFF" if self.muted else self.buzz_name
            self.audio.on = on
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
        self.audio.close()
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
