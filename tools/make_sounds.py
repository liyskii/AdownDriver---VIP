"""Generates the project's own (original) alert sounds into sounds/*.wav.  python tools/make_sounds.py"""
import wave, pathlib
import numpy as np

SR = 22050
OUT = pathlib.Path(__file__).resolve().parent.parent / "sounds"

def hz(note):                       # "A4", "C#5"
    names = {"C":-9,"C#":-8,"D":-7,"D#":-6,"E":-5,"F":-4,"F#":-3,"G":-2,"G#":-1,"A":0,"A#":1,"B":2}
    return 440.0 * 2 ** ((names[note[:-1]] + 12 * (int(note[-1]) - 4)) / 12)

def env(n, a=0.005, d=0.25):
    t = np.arange(n) / SR
    return np.minimum(1, t / a) * np.exp(-t / d)

def mallet(f, dur, d=0.18):         # marimba / xylophone-like
    n = int(SR * dur); t = np.arange(n) / SR
    x = np.sin(2*np.pi*f*t) + 0.35*np.sin(2*np.pi*4*f*t)*np.exp(-t/0.03) + 0.2*np.sin(2*np.pi*3*f*t)
    return x * env(n, 0.003, d)

def synth(f, dur, bright=1.0, d=0.6):  # detuned saw pad / lead
    n = int(SR * dur); t = np.arange(n) / SR
    x = sum(((t*f*k) % 1.0)*2-1 for k in (1.0, 1.006, 0.994)) / 3
    lp = np.exp(-t/(0.15*bright))       # crude "filter" by mixing sine as it closes
    x = x*lp + np.sin(2*np.pi*f*t)*(1-lp)
    return x * env(n, 0.01, d)

def seq(notes, voice, step, tail=0.4, **kw):
    total = int(SR * (step*len(notes) + tail)); out = np.zeros(total)
    for i, nt in enumerate(notes):
        if nt is None: continue
        w = voice(hz(nt), step + tail, **kw); s = int(i*step*SR)
        out[s:s+w.size] += w[:total-s]
    return out

def save(name, x, gain=0.8):
    x = x / (np.max(np.abs(x)) + 1e-9) * gain
    with wave.open(str(OUT / name), "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR)
        w.writeframes((x * 32767).astype(np.int16).tobytes())
    print(name, round(x.size / SR, 2), "s")

# 1) warning: cheerful, bouncy xylophone tune - "hey, wake up!"
tune = ["C5","E5","G5","E5","A5",None,"G5",None,"E5","G5","C6",None]
bass = ["C4",None,None,None,"F4",None,None,None,"G4",None,"C4",None]
save("warning.wav", seq(tune, mallet, 0.13, 0.5) + 0.6*seq(bass, mallet, 0.13, 0.5, d=0.3))

# 2) slowing: phone-alarm style marimba pattern, rising and repeating
alarm = ["E5","B5","E5","B5","G#5","E6",None,None]*2
save("slowing.wav", seq(alarm, mallet, 0.11, 0.3, d=0.12))

# 3) stop: urgent two-tone alarm with a pulsing synth
n = int(SR*1.2); t = np.arange(n)/SR
f = np.where((t*4).astype(int) % 2 == 0, 880.0, 660.0)
siren = np.sign(np.sin(2*np.pi*np.cumsum(f)/SR)) * 0.5 * (0.6 + 0.4*np.sin(2*np.pi*16*t)**2)
save("stop.wav", siren + 0.7*seq(["A5",None,"E5",None]*2, mallet, 0.15, 0.0, d=0.1)[:n] if True else siren, 0.75)

# 4) hello: neon "system online" boot greeting - riser, arpeggio, chord
riser_n = int(SR*0.6); rt = np.arange(riser_n)/SR
riser = np.sin(2*np.pi*np.cumsum(200 + 1400*(rt/0.6)**2)/SR) * (rt/0.6) * 0.4
arp = seq(["A3","C4","E4","A4","C5","E5","A5","E6"], synth, 0.07, 0.3, bright=0.6, d=0.25)
chord = sum(synth(hz(nt), 1.6, bright=2.0, d=0.9) for nt in ("A3","E4","A4","C5","G5"))
hello = np.concatenate([riser, arp, chord*0.6])
save("hello.wav", hello, 0.7)
