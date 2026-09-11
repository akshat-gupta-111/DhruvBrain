# Use a lightweight Debian-based Python image compatible with ARM64 (Jetson Orin)
FROM python:3.10-slim

# Prevent Python from buffering outputs
ENV PYTHONUNBUFFERED=1

# Install C++ build tools, Linux Audio drivers (ALSA), and Linux Video drivers (V4L)
RUN apt-get update && apt-get install -y \
    build-essential \
    libasound2 \
    alsa-utils \
    espeak \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the DhruvBrain codebase
COPY . .

# Launch the orchestrator
CMD ["python", "main.py"]