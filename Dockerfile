# Use a lightweight Debian-based Python image compatible with ARM64 (Jetson Orin)
FROM python:3.10-slim

# Prevent Python from buffering outputs
ENV PYTHONUNBUFFERED=1

# 1. Install system utilities required to fetch packages
RUN apt-get update && apt-get install -y \
    wget \
    gnupg \
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

# 2. Inject OpenSSL 1.1 (Required by Azure Speech SDK on Debian/Ubuntu modern images)
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports focal main universe" >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends libssl1.1 && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app

# 3. Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 4. Copy codebase
COPY . .

# Launch orchestrator
CMD ["python", "main.py"]