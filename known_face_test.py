import cv2
import mediapipe as mp
import face_recognition
import numpy as np
import os

class JarvisVisionEngine:
    def __init__(self, known_faces_dir="known_faces"):
        print("\n[VISION] Initializing Jarvis Vision Engine...")
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Upgraded to handle 4 hands simultaneously
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=4, 
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5
        )

        self.known_face_encodings = []
        self.known_face_names = []
        self._load_known_faces(known_faces_dir)

    def _load_known_faces(self, faces_dir):
        if not os.path.exists(faces_dir):
            os.makedirs(faces_dir, exist_ok=True)
            return

        for filename in os.listdir(faces_dir):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                filepath = os.path.join(faces_dir, filename)
                image = face_recognition.load_image_file(filepath)
                encodings = face_recognition.face_encodings(image)
                if encodings:
                    self.known_face_encodings.append(encodings[0])
                    name = os.path.splitext(filename)[0].capitalize()
                    self.known_face_names.append(name)
                    print(f"[VISION] Loaded face profile: {name}")

    def _count_fingers(self, hand_landmarks):
        """Distance-based finger tracking immune to camera mirroring."""
        landmarks = hand_landmarks.landmark
        fingers = 0

        # Thumb Check (Euclidean distance: tip vs knuckle compared to pinky base)
        thumb_tip_dist = (landmarks[4].x - landmarks[17].x)**2 + (landmarks[4].y - landmarks[17].y)**2
        thumb_base_dist = (landmarks[2].x - landmarks[17].x)**2 + (landmarks[2].y - landmarks[17].y)**2
        if thumb_tip_dist > thumb_base_dist:
            fingers += 1

        # 4 Fingers Check (Vertical Y coordinate)
        tip_ids = [8, 12, 16, 20]
        pip_ids = [6, 10, 14, 18]
        for tip, pip in zip(tip_ids, pip_ids):
            if landmarks[tip].y < landmarks[pip].y:
                fingers += 1

        return fingers

    def process_frame(self, frame):
        if frame is None:
            return 0, [], "No feed", frame

        display_frame = frame.copy()
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # 1. Multi-Face Recognition
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
        detected_names = []

        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding, tolerance=0.55)
            name = "Unknown"
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            
            if len(face_distances) > 0:
                best_match = np.argmin(face_distances)
                if matches[best_match]:
                    name = self.known_face_names[best_match]
            detected_names.append(name)

            box_color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
            cv2.rectangle(display_frame, (left, top), (right, bottom), box_color, 2)
            cv2.putText(display_frame, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)

        # 2. Hand Gesture & Multi-Hand Finger Counting
        results = self.hands.process(rgb_frame)
        total_fingers = 0

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(display_frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                total_fingers += self._count_fingers(hand_landmarks)

        # Context generation
        summary_parts = []
        if detected_names:
            summary_parts.append(f"People in view: {', '.join(detected_names)}.")
        else:
            summary_parts.append("No recognized faces.")
            
        if total_fingers > 0:
            summary_parts.append(f"A total of {total_fingers} fingers are being held up.")
        else:
            summary_parts.append("No fingers are visible.")

        visual_summary = " ".join(summary_parts)
        
        # We now return the full list of detected names, not just the first one
        return total_fingers, detected_names, visual_summary, display_frame

    def release(self):
        self.hands.close()