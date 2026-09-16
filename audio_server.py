#!/usr/bin/env python3
"""
DhruvBrain Audio Sidecar Server
================================
Runs on the Jetson HOST (outside Docker). Handles all audio I/O using
the exact same sounddevice approach proven in utility.py.

The Docker container talks to this via HTTP on localhost:5555.

Setup (run once on Jetson):
    pip install flask edge-tts SpeechRecognition sounddevice numpy

Run (before docker compose up):
    python3 audio_server.py
"""

import asyncio
import io
import json
import os
import subprocess
import threading
import time
import wave
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import sounddevice as sd
import speech_recognition as sr
import edge_tts

# ── Audio config (directly from utility.py) ──────────────────────────
INPUT_DEVICE_INDEX    = int(os.getenv("INPUT_DEVICE_INDEX",    "26"))  # PulseAudio capture
PLAYBACK_DEVICE_INDEX = int(os.getenv("PLAYBACK_DEVICE_INDEX", "24"))  # ReSpeaker hardware
MIC_CHANNELS          = int(os.getenv("MIC_CHANNELS",          "2"))   # PulseAudio → stereo
SAMPLE_RATE           = 16000
READ_CHUNK            = 1024                          # matches utility.py
STT_CHUNK_FRAMES      = 4 * SAMPLE_RATE               # 4-second STT windows
ALSA_DEVICE           = os.getenv("ALSA_DEVICE", "plughw:2,0")
TTS_VOICE             = "en-US-GuyNeural"

# ── Shared mic state ──────────────────────────────────────────────────
_speech_queue: list[str] = []
_speech_lock  = threading.Lock()
_is_muted     = True   # start muted; container calls /unmute when ready
_mic_running  = True
_recognizer   = sr.Recognizer()

# ── Mic background thread ─────────────────────────────────────────────

def mic_loop():
    """Mirrors utility.py's recording loop exactly — runs forever on the host."""
    # Flush device locks (utility.py pattern)
    os.system("fuser -k /dev/snd/pcmC2D0c 2>/dev/null")
    time.sleep(0.5)

    while _mic_running:
        try:
            with sd.InputStream(
                device=INPUT_DEVICE_INDEX,
                channels=MIC_CHANNELS,
                samplerate=SAMPLE_RATE,
                dtype='int16'
            ) as stream:
                print(f"[Audio Server] 🎤 Mic stream opened (device={INPUT_DEVICE_INDEX}, ch={MIC_CHANNELS}).")
                audio_frames  = []
                frames_so_far = 0

                while _mic_running:
                    # Exactly: data_chunk, overflowed = stream.read(1024)
                    data_chunk, overflowed = stream.read(READ_CHUNK)
                    frames_so_far += len(data_chunk)
                    audio_frames.append(data_chunk)

                    if frames_so_far < STT_CHUNK_FRAMES:
                        continue

                    # full_audio = np.concatenate(audio_frames, axis=0)
                    full_audio = np.concatenate(audio_frames, axis=0)

                    # mono_audio = full_audio[:, 0]  (utility.py pattern, with safety)
                    if MIC_CHANNELS > 1:
                        mono_audio = full_audio[:, 0]
                    else:
                        mono_audio = full_audio.flatten()

                    audio_frames  = []
                    frames_so_far = 0

                    if _is_muted:
                        continue  # discard — speaker is talking

                    # STT in background thread so stream never stalls
                    threading.Thread(target=_do_stt, args=(mono_audio,), daemon=True).start()

        except Exception as e:
            print(f"[Audio Server] Mic stream error: {e} — retrying in 2s")
            time.sleep(2)
            os.system("fuser -k /dev/snd/pcmC2D0c 2>/dev/null")
            time.sleep(0.5)


def _do_stt(mono_audio: np.ndarray):
    """Convert mono PCM → WAV → Google STT → push to queue."""
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)          # int16 = 2 bytes
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(mono_audio.tobytes())
    buf.seek(0)

    try:
        with sr.AudioFile(buf) as src:
            audio = _recognizer.record(src)
        text = _recognizer.recognize_google(audio).lower().strip()
        if text:
            print(f"[Audio Server] 🗣️  Heard: '{text}'")
            with _speech_lock:
                _speech_queue.append(text)
    except sr.UnknownValueError:
        pass   # silence / inaudible — normal
    except Exception as e:
        print(f"[Audio Server] STT error: {e}")


# ── TTS ───────────────────────────────────────────────────────────────

def _do_speak(text: str):
    """Stream edge-tts → mpv stdin. Blocks until playback complete."""
    loop = asyncio.new_event_loop()
    loop.run_until_complete(_stream_to_mpv(text))
    loop.close()


async def _stream_to_mpv(text: str):
    alsa_mpv = f"alsa/{ALSA_DEVICE}"
    proc = subprocess.Popen(
        ["mpv", "--no-terminal", f"--audio-device={alsa_mpv}",
         "--demuxer=lavf", "--demuxer-lavf-format=mp3", "-"],
        stdin=subprocess.PIPE, stderr=subprocess.DEVNULL
    )
    communicate = edge_tts.Communicate(text, TTS_VOICE)
    try:
        async for chunk in communicate.stream():
            if chunk["type"] == "audio":
                proc.stdin.write(chunk["data"])
    except BrokenPipeError:
        pass
    finally:
        try:
            proc.stdin.close()
        except Exception:
            pass
    proc.wait()
    # Fallback: retry without explicit device
    if proc.returncode != 0:
        proc2 = subprocess.Popen(
            ["mpv", "--no-terminal", "--demuxer=lavf", "--demuxer-lavf-format=mp3", "-"],
            stdin=subprocess.PIPE, stderr=subprocess.DEVNULL
        )
        communicate2 = edge_tts.Communicate(text, TTS_VOICE)
        try:
            async for chunk in communicate2.stream():
                if chunk["type"] == "audio":
                    proc2.stdin.write(chunk["data"])
        except BrokenPipeError:
            pass
        finally:
            try:
                proc2.stdin.close()
            except Exception:
                pass
        proc2.wait()


# ── HTTP Server ───────────────────────────────────────────────────────

class AudioHandler(BaseHTTPRequestHandler):
    """Minimal HTTP handler — no Flask dependency needed on host."""

    def log_message(self, fmt, *args):
        pass  # suppress default access logs

    def _send_json(self, data: dict, status: int = 200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(length)) if length else {}

    def do_POST(self):
        global _is_muted

        if self.path == "/speak":
            data = self._read_json()
            text = data.get("text", "")
            if text:
                _do_speak(text)          # blocks until mpv done
            self._send_json({"ok": True})

        elif self.path == "/mute":
            _is_muted = True
            with _speech_lock:
                _speech_queue.clear()
            self._send_json({"muted": True})

        elif self.path == "/unmute":
            with _speech_lock:
                _speech_queue.clear()
            _is_muted = False
            self._send_json({"muted": False})

        else:
            self._send_json({"error": "not found"}, 404)

    def do_GET(self):
        if self.path == "/hear":
            with _speech_lock:
                text = _speech_queue.pop(0) if _speech_queue else ""
            self._send_json({"text": text})

        elif self.path == "/health":
            self._send_json({"status": "ok", "muted": _is_muted})

        else:
            self._send_json({"error": "not found"}, 404)


# ── Entry point ───────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 55)
    print("  DhruvBrain Audio Sidecar Server")
    print(f"  Mic  : device={INPUT_DEVICE_INDEX}, ch={MIC_CHANNELS}")
    print(f"  Spkr : {ALSA_DEVICE}  (mpv streaming)")
    print(f"  STT  : Google Speech Recognition (4s chunks)")
    print("=" * 55)

    # Start mic thread
    threading.Thread(target=mic_loop, daemon=True).start()

    # Start HTTP server
    server = HTTPServer(("0.0.0.0", 5555), AudioHandler)
    print("[Audio Server] ✅ Listening on http://0.0.0.0:5555")
    print("[Audio Server]    Container calls: http://localhost:5555")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Audio Server] Shutting down.")
