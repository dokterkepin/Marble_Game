import threading
from camera import detection
from controll import manual

def run_detection():
    detection.run_detection_loop()

def run_manual():
    manual.run_manual_loop()


detection_thread = threading.Thread(target=run_detection)
manual_thread = threading.Thread(target=run_manual)

detection_thread.start()
manual_thread.start()



