import os
import azure.cognitiveservices.speech as speechsdk
from dotenv import load_dotenv

load_dotenv()

def run_test():
    speech_key = os.getenv("AZURE_SPEECH_KEY")
    speech_region = os.getenv("AZURE_SPEECH_REGION")
    
    if not speech_key:
        print("❌ AZURE_SPEECH_KEY is missing from .env")
        return

    print("Initializing Azure Speech...")
    speech_config = speechsdk.SpeechConfig(subscription=speech_key, region=speech_region)
    audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
    
    recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)
    
    print("\n🟢 MIC IS LIVE! Please speak into your Mac right now...")
    
    # recognize_once blocks the script until it hears a full sentence
    result = recognizer.recognize_once_async().get()
    
    if result.reason == speechsdk.ResultReason.RecognizedSpeech:
        print(f"\n✅ SUCCESS! I heard: '{result.text}'")
    elif result.reason == speechsdk.ResultReason.NoMatch:
        print("\n❌ FAILED: I heard silence. macOS is blocking the mic, or it's muted.")
    elif result.reason == speechsdk.ResultReason.Canceled:
        cancellation = result.cancellation_details
        print(f"\n⚠️ CANCELED: {cancellation.reason} - {cancellation.error_details}")

if __name__ == "__main__":
    run_test()