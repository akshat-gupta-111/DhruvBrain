import sounddevice as sd 
import numpy as np 
import wave 
import time 
import os 

# Audio Configuration
SAMPLE_RATE = 16000 
DURATION = 5 
FILENAME = "output.wav" 

# Target indices directly from your working configuration
INPUT_DEVICE_INDEX = 26      # Working pulse capture node
PLAYBACK_DEVICE_INDEX = 24   # Working ReSpeaker hardware speaker node

# CHANGE THIS: PulseAudio nodes usually export 1 (mono) or 2 (stereo) channels
CHANNELS = 2  

def main():
    print("--- Jetson Audio Controller (Pulse Node Mode) ---")
    
    # Flush audio server locks just in case
    os.system("fuser -k /dev/snd/pcmC2D0c 2>/dev/null")
    time.sleep(0.5)
    
    audio_frames = []
    print(f"Recording started for {DURATION} seconds... Speak into the ReSpeaker.")
    start_time = time.time()
    
    try:
        # Directly target the working Pulse node (Index 26)
        with sd.InputStream(device=INPUT_DEVICE_INDEX, channels=CHANNELS, samplerate=SAMPLE_RATE, dtype='int16') as stream:
            while time.time() - start_time < DURATION:
                data_chunk, overflowed = stream.read(1024)
                audio_frames.append(data_chunk)
        print("Recording stopped successfully.")
    except Exception as e:
        print(f"\nRecording Error: {e}")
        return

    # Process and save the captured audio matrix
    full_audio = np.concatenate(audio_frames, axis=0)
    
    # Isolate microphone channel 0 safely depending on channel count
    if CHANNELS > 1:
        mono_audio = full_audio[:, 0] 
    else:
        mono_audio = full_audio.flatten()

    with wave.open(FILENAME, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2) # int16 = 2 bytes
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(mono_audio.tobytes())
    print(f"Saved recording to '{FILENAME}'")

    # Read back the audio track from the local file
    print(f"Loading '{FILENAME}' for hardware playback...")
    with wave.open(FILENAME, 'rb') as wf:
        playback_bytes = wf.readframes(wf.getnframes())
        playback_data = np.frombuffer(playback_bytes, dtype=np.int16)

    # Project the voice file out via the speaker channel (Index 24)
    print("Playing file audio back through the speaker...")
    try:
        sd.play(playback_data, samplerate=SAMPLE_RATE, device=PLAYBACK_DEVICE_INDEX)
        sd.wait()
        print("Playback finished successfully!")
    except Exception as playback_err:
        print(f"Direct hardware playback failed: {playback_err}")
        print("Attempting fallback speaker playback via system default...")
        sd.play(playback_data, samplerate=SAMPLE_RATE, device=None)
        sd.wait()

if __name__ == "__main__":
    main()