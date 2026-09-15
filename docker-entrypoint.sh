#!/bin/bash
set -e

echo "[Entrypoint] Starting PulseAudio daemon..."
pulseaudio --start \
           --load="module-native-protocol-unix" \
           --log-target=stderr \
           --exit-idle-time=-1 \
           --daemon

# Give PulseAudio a moment to fully initialize before the brain touches audio
sleep 1

echo "[Entrypoint] PulseAudio running. Launching DhruvBrain..."
exec python main.py
