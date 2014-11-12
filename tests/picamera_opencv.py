import cv2
import picamera
import picamera.array
import time

previous_time = time.time()
with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (320, 240)

	while True:
            print time.time() - previous_time
	    previous_time = time.time()
	    camera.capture(stream, 'bgr', use_video_port=True)
	    # stream.array now contains the image data in BGR order
            cv2.imshow('frame', stream.array)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # reset the stream before the next capture
            #stream.seek(0)
            stream.truncate(0)

        cv2.destroyAllWindows()
