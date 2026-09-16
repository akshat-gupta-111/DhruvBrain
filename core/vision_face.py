import cv2
import face_recognition
import numpy as np
import os
import json

class FaceIdentityEngine:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(FaceIdentityEngine, cls).__new__(cls, *args, **kwargs)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, known_faces_dir="known_faces", description_file="description.json"):
        if self._initialized:
            return
            
        print("\n[VISION] Initializing Face Identity Engine...")
        self.known_face_encodings = []
        self.known_face_names = []
        self.descriptions = {}
        
        self._load_descriptions(description_file)
        self._load_known_faces(known_faces_dir)
        self._initialized = True

    def _load_descriptions(self, filepath):
        if os.path.exists(filepath):
            try:
                with open(filepath, 'r') as f:
                    self.descriptions = json.load(f)
            except Exception as e:
                print(f"[VISION] Error loading descriptions: {e}")
        else:
            print(f"[VISION] Warning: {filepath} not found.")

    def _load_known_faces(self, faces_dir):
        if not os.path.exists(faces_dir):
            os.makedirs(faces_dir, exist_ok=True)
            return

        for filename in os.listdir(faces_dir):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                filepath = os.path.join(faces_dir, filename)
                try:
                    image = face_recognition.load_image_file(filepath)
                    encodings = face_recognition.face_encodings(image)
                    if encodings:
                        self.known_face_encodings.append(encodings[0])
                        name = os.path.splitext(filename)[0].capitalize()
                        self.known_face_names.append(name)
                        print(f"[VISION] Loaded face profile: {name}")
                except Exception as e:
                    print(f"[VISION] Error processing {filename}: {e}")

    def detect_faces(self, frame) -> tuple[list[str], str]:
        """
        Returns a tuple: 
        1) A list of detected names (e.g. ['Akshat', 'Unknown'])
        2) A descriptive string summarizing who is in the frame and their descriptions.
        """
        if frame is None:
            return [], ""

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
        
        detected_names = []
        for face_encoding in face_encodings:
            name = "Unknown"
            if len(self.known_face_encodings) > 0:
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding, tolerance=0.55)
                face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                if len(face_distances) > 0:
                    best_match = np.argmin(face_distances)
                    if matches[best_match]:
                        name = self.known_face_names[best_match]
            detected_names.append(name)

        if not detected_names:
            return [], ""

        summary_parts = []
        for name in set(detected_names):
            desc = self.descriptions.get(name) or self.descriptions.get(name.lower())
            if not desc and name == "Unknown":
                desc = self.descriptions.get("Unknown", "An unidentified person.")
            
            if desc:
                summary_parts.append(f"I see {name}. Context: {desc}")
            else:
                summary_parts.append(f"I see {name}.")

        return detected_names, " ".join(summary_parts)
