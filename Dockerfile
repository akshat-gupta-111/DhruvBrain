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
    libasound2 \
    alsa-utils \
    espeak \
    v4l-utils \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    && rm -rf /var/lib/apt/lists/*

# 2. Download and install libssl1.1 for ARM64
RUN wget -O libssl1.1_1.1.1f-1ubuntu2_arm64.deb \
    http://ports.ubuntu.com/ubuntu-ports/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_arm64.deb && \
    dpkg -i libssl1.1_1.1.1f-1ubuntu2_arm64.deb && \
    rm -f libssl1.1_1.1.1f-1ubuntu2_arm64.deb

WORKDIR /app

# 3. Install Python dependencies
COPY requirements.txt .

RUN pip install --no-cache-dir -r requirements.txt

# 4. Copy codebase
COPY . .

# Launch orchestrator
CMD ["python", "main.py"]
