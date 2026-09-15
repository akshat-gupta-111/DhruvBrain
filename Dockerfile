# Use a lightweight Debian-based Python image compatible with ARM64
FROM python:3.10-slim

# Prevent Python from buffering outputs
ENV PYTHONUNBUFFERED=1
# Tell PulseAudio to use the local unix socket (not session/dbus)
ENV PULSE_SERVER=unix:/run/pulse/native

# 1. Install system utilities and multimedia/audio packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    gnupg \
    ca-certificates \
    build-essential \
    python3-dev \
    # ALSA — low-level Linux audio layer
    libasound2 \
    libasound2-plugins \
    alsa-utils \
    # PortAudio — C library required by 'sounddevice' Python package
    libportaudio2 \
    portaudio19-dev \
    # PulseAudio — Azure Speech SDK uses PA as the audio router inside Docker
    pulseaudio \
    pulseaudio-utils \
    libpulse-dev \
    # TTS fallback (espeak)
    espeak \
    # Camera / USB video
    v4l-utils \
    # GStreamer (optional pipeline support)
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    && rm -rf /var/lib/apt/lists/*

# 2. Install libssl1.1 for ARM64 — REQUIRED by Azure Speech SDK's azure-c-shared layer
#    Error code 2176 = platform init failure without this library on ARM64
RUN wget -q -O /tmp/libssl1.1.deb \
    http://ports.ubuntu.com/ubuntu-ports/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_arm64.deb && \
    dpkg -i /tmp/libssl1.1.deb && \
    rm /tmp/libssl1.1.deb

WORKDIR /app

# 3. Install Python dependencies
COPY requirements.txt .
RUN pip install --default-timeout=1000 --no-cache-dir -r requirements.txt

# 4. Copy codebase
COPY . .

# 5. Make entrypoint executable
RUN chmod +x /app/docker-entrypoint.sh

# Launch: start PulseAudio daemon first, then run the brain
CMD ["/app/docker-entrypoint.sh"]
