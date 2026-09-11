# Use a lightweight Debian-based Python image compatible with ARM64 (Jetson Orin)
FROM python:3.10-slim

# Prevent Python from buffering outputs
ENV PYTHONUNBUFFERED=1

# Install C++ build tools, Linux Audio drivers (ALSA), and Linux Video drivers (V4L)
RUN apt-get update && apt-get install -y \
    build-essential \
    libssl-dev \
    libasound2 \
    alsa-utils \
    espeak \
    v4l-utils \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the DhruvBrain codebase
COPY . .

# Launch the orchestrator
CMD ["python", "main.py"]