import threading
import cv2
import rospy
import time

class VideoCaptureAsync:
    def __init__(self, src=0, width=1280, height=720, b=None):
        self.src = src
        self.cap = cv2.VideoCapture(self.src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.grabbed, self.frame = self.cap.read()
        self.stamp = rospy.Time.from_sec(time.time())
        self.started = False
        self.read_lock = threading.Lock()
        self.b = b

    def set(self, var1, var2):
        self.cap.set(var1, var2)

    def start(self):
        if self.started:
            print('[!] Asynchroneous video capturing has already been started.')
            return None
        self.started = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        if self.b is None:
            pass
        else:
            self.b.wait()
        while self.started:
            grabbed, frame = self.cap.read()
            stamp = rospy.Time.from_sec(time.time())
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame
                self.stamp = stamp

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
            stamp = self.stamp
        return grabbed, frame, stamp

    def stop(self):
        self.started = False
        self.thread.join()

    def __exit__(self, exec_type, exc_value, traceback):
        self.cap.release()
