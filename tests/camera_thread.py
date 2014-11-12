import threading
import time
import cv2
import picamera
import picamera.array
import curses

def curses_print(string, line, col):
    """
    Function to do a simple curses print.
    """

    #Check for bad inputs
    if col > 1 or col < 0:
        return

    if line > 22 or line < 0:
        return

    #Print to screen using curses
    if col == 0:
        screen.addstr(line, 0, string)
    if col == 1:
        screen.addstr(line, 40, string)

    screen.refresh()

#Setup the screen for curses
screen = curses.initscr()
screen.clear()
screen.refresh()

frame = None

def camera(event):
    global frame
    with picamera.PiCamera() as camera:
        with picamera.array.PiRGBArray(camera) as stream:
            camera.resolution = (320, 240)

            while not event.is_set():
                curses_print("In camera loop",3,0)
                camera.capture(stream, 'bgr', use_video_port=True)
                # stream.array now contains the image data in BGR order
		frame = stream.array
                stream.truncate(0)

event = threading.Event()

d = threading.Thread(name='camera', target=camera, args=(event,))

d.start()

start_time = time.time()

while (time.time()-start_time) < 10:
    curses_print(str(time.time()-start_time),0,0)
    try:
        cv2.imshow('frame', frame)
    except:
        pass
        curses_print("Failed image show",1,0)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    screen.clear()
    screen.refresh()

event.set()
d.join()

cv2.destroyAllWindows()
curses.endwin()
