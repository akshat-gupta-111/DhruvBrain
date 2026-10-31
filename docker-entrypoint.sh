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

echo "[Entrypoint] Setting default audio sink to ReSpeaker 4-Mic Array..."
pacmd set-default-sink alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.iec958-stereo || \
    echo "[Entrypoint] WARNING: Could not set default sink (device may not be ready yet)"

echo "[Entrypoint] PulseAudio running. Launching DhruvBrain..."
exec python main.py
