# Use a lightweight Debian-based Python image compatible with ARM64
FROM python:3.10-slim

# Prevent Python from buffering outputs
ENV PYTHONUNBUFFERED=1

# 1. Install system utilities and multimedia/audio packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    gnupg \
    ca-certificates \
    build-essential \
    python3-dev \
    # ALSA — low-level Linux audio layer
    libasound2 \
    libasound2-dev \
    libasound2-plugins \
    alsa-utils \
    # PortAudio — C library required by PyAudio Python package
    libportaudio2 \
    portaudio19-dev \
    # PulseAudio — helps route ALSA devices inside Docker
    pulseaudio \
    pulseaudio-utils \
    libpulse-dev \
    # mpv — plays edge-tts MP3 output to hardware speaker
    mpv \
    # TTS fallback
    espeak \
    # Camera / USB video
    v4l-utils \
    # GStreamer (optional pipeline support)
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# 2. Install Python dependencies
COPY requirements.txt .
RUN pip install --default-timeout=1000 --no-cache-dir -r requirements.txt

# 3. Copy codebase
COPY . .

# 4. Make entrypoint executable
RUN chmod +x /app/docker-entrypoint.sh

# Launch: start PulseAudio daemon first, then run the brain
CMD ["/app/docker-entrypoint.sh"]
