"""
Puts the USB cameras plugged into this computer on a web page, so the robot's phone app
(http://192.168.4.1, "Cameras" card) and any browser on the same network can watch them.

Built on the team's Camera-SetUp scripts (https://github.com/RutgersMicromouse/Camera-SetUp):
the same threaded CameraStream with MJPEG at 640x480, but sending the pictures to a browser
instead of an OpenCV window.

    pip install opencv-python
    python camera_server.py                                  every camera it can find
    python camera_server.py 2 3 0                            only these camera numbers
    python camera_server.py "2=Left Cam" "3=Middle Cam" "0=Right Cam"     ...with names
    python camera_server.py --port 9000

Camera numbers are the ones Camera-SetUp/Cameras/cameraID.py prints.

To see the feeds in the robot's app, this computer must be on the robot's Wi-Fi
(Antigrav-Mouse). Then type the address this program prints into the app's Cameras card.
Anyone on the same network can open these feeds; there is no password.
"""
import argparse
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

FRAMES_PER_SECOND = 15
JPEG_QUALITY = 70


class CameraStream:
    """One camera, read continuously in the background (from Camera-SetUp/Cameras/Multi-Camera.py)."""

    def __init__(self, index, name):
        self.index = index
        self.name = name
        self.capture = cv2.VideoCapture(index)

        # Force MJPEG (fixes black screens / USB bandwidth problems with several cameras)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # Keep the resolution low to prevent lag
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.status, self.frame = self.capture.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            status, frame = self.capture.read()
            if status:
                self.status, self.frame = status, frame
            else:
                time.sleep(0.05)  # Camera unplugged or busy: keep the last picture and try again
        self.capture.release()

    def jpeg(self):
        """The newest picture with the camera's name written on it, as JPEG bytes (None if there is none)."""
        if self.frame is None:
            return None
        frame = self.frame.copy()
        cv2.putText(frame, self.name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, self.name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return data.tobytes() if ok else None

    def stop(self):
        self.stopped = True


def find_cameras(max_tests=10):
    """Camera numbers that give a picture (same test as Camera-SetUp/Cameras/cameraID.py)."""
    found = []
    for index in range(max_tests):
        capture = cv2.VideoCapture(index)
        if capture.read()[0]:
            found.append(index)
        capture.release()
    return found


def my_addresses():
    """This computer's network addresses, the robot's Wi-Fi (192.168.4.x) first."""
    addresses = set()
    try:
        addresses.update(socket.gethostbyname_ex(socket.gethostname())[2])
    except OSError:
        pass
    addresses.discard('127.0.0.1')
    return sorted(addresses, key=lambda a: (not a.startswith('192.168.4.'), a))


PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Micromouse cameras</title>
<style>body{margin:0;padding:12px;background:#0b0d10;color:#f5f7fa;font-family:sans-serif}
.cams{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:10px}
img{width:100%;border-radius:8px;background:#07090c;display:block}</style></head><body>
<h2>Micromouse cameras</h2><div class="cams">%s</div></body></html>"""


def make_handler(cameras):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # Keep the terminal quiet

        def reply(self, kind, body):
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Access-Control-Allow-Origin', '*')  # Lets the robot's page ask for the camera list
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/':
                images = ''.join('<img src="/cam/%d.mjpg" alt="%s">' % (i, c.name) for i, c in enumerate(cameras))
                self.reply('text/html', (PAGE % (images or 'No cameras found.')).encode())
            elif path == '/cameras.json':
                listing = [{'name': c.name, 'index': c.index, 'stream': '/cam/%d.mjpg' % i} for i, c in enumerate(cameras)]
                self.reply('application/json', json.dumps(listing).encode())
            elif path.startswith('/cam/') and path.endswith('.mjpg'):
                self.stream(path[5:-5])
            else:
                self.send_error(404)

        def stream(self, number):
            if not number.isdigit() or int(number) >= len(cameras):
                self.send_error(404)
                return
            camera = cameras[int(number)]
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                while True:
                    picture = camera.jpeg()
                    if picture:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(picture))
                        self.wfile.write(picture)
                        self.wfile.write(b'\r\n')
                    time.sleep(1.0 / FRAMES_PER_SECOND)
            except (BrokenPipeError, ConnectionError, OSError):
                pass  # The viewer closed the page

    return Handler


def main():
    parser = argparse.ArgumentParser(description='Show this computer\'s USB cameras on a web page.')
    parser.add_argument('cameras', nargs='*', help='camera numbers, optionally named: 2 or "2=Left Cam" (default: all found)')
    parser.add_argument('--port', type=int, default=8090)
    args = parser.parse_args()

    if args.cameras:
        wanted = []
        for item in args.cameras:
            index, _, name = item.partition('=')
            wanted.append((int(index), name or 'Camera %s' % index))
    else:
        print('Looking for cameras (numbers 0 to 9)...')
        wanted = [(index, 'Camera %d' % index) for index in find_cameras()]

    cameras = []
    for index, name in wanted:
        camera = CameraStream(index, name)
        if camera.status:
            print('  %s (number %d) started.' % (name, index))
            cameras.append(camera.start())
        else:
            print('  Could not open %s (number %d).' % (name, index))
            camera.capture.release()
    if not cameras:
        print('No cameras are working. Plug one in, or check the numbers with cameraID.py.')

    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(cameras))
    server.daemon_threads = True
    print('\nCamera feeds are on:')
    print('  http://localhost:%d              (this computer)' % args.port)
    for address in my_addresses():
        note = '   <- put this in the robot app\'s Cameras card' if address.startswith('192.168.4.') else ''
        print('  http://%s:%d%s' % (address, args.port, note))
    print('Press Ctrl+C to stop.')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for camera in cameras:
            camera.stop()


if __name__ == '__main__':
    main()
