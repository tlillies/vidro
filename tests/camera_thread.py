import threading
import time
import cv2
import picamera
import picamera.array
import curses
import matplotlib.pyplot as plot

plot_camera_time=[]
plot_camera_cycle=[]
plot_main_time=[]
plot_main_cycle=[]

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
    global plot_camera_time
    global Plot_camera_cycle
    global start_time
    previous_time = time.time()
    with picamera.PiCamera() as camera:
        with picamera.array.PiRGBArray(camera) as stream:
            camera.resolution = (320, 240)

            while not event.is_set():
                current_time = time.time()
                plot_camera_time.append(current_time-start_time)
                plot_camera_cycle.append(current_time-previous_time)
                previous_time = current_time
                curses_print("In camera loop",3,0)
                camera.capture(stream, 'bgr', use_video_port=True)
                # stream.array now contains the image data in BGR order
		frame = stream.array
                stream.truncate(0)

event = threading.Event()

d = threading.Thread(name='camera', target=camera, args=(event,))

d.start()

start_time = time.time()
previous_time = time.time()

while (time.time()-start_time) < 15:
    current_time = time.time()
    curses_print(str(current_time-start_time),0,0)
    plot_main_time.append(current_time-start_time)
    plot_main_cycle.append((current_time-previous_time))
    previous_time = current_time
    """
    try:
        cv2.imshow('frame', frame)
    except:
        pass
        curses_print("Failed image show",1,0)
    """
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    screen.clear()
    screen.refresh()

event.set()
d.join()

cv2.destroyAllWindows()
curses.endwin()

plot.figure(1)
plot.xlabel("Time (sec)")
plot.ylabel("Time/cycle")
plot.title("Thread and main loop cycle times")
plot.plot(plot_camera_time, plot_camera_cycle)
plot.plot(plot_main_time, plot_main_cycle)

plot.show()
