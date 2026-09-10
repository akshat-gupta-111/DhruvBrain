# Use a lightweight ARM64 compatible base image
FROM python:3.10-slim

# Prevent Python from buffering stdout/stderr to ensure real-time terminal logs
ENV PYTHONUNBUFFERED=1

# Install Linux hardware drivers and build tools needed for OpenCV and PyAudio/SpeechSDK
RUN apt-get update && apt-get install -y \
    build-essential \
    libasound2 \
    espeak \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app

# Copy dependencies first for Docker caching
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the DhruvBrain codebase
COPY . .

# Default command: Start the Master Node (Dashboard + LangGraph)
CMD ["python", "main.py"]