"""
Using the NanoCamera with CSI Camera
@author: Ayo Ayibiowu

"""
import cv2

# from nanocamera.NanoCam import Camera
import nanocamera as nano

if __name__ == '__main__':
    # Create the Camera instance
    camera = nano.Camera(flip=0, width=1920, height=1080, fps=30, debug=True)
    # For multiple CSI camera
    camera_2 = nano.Camera(device_id=1, flip=0, width=1280, height=720, fps=60, debug=True)
    print('CSI Camera is now ready')
    while True:
        try:
            status = camera.hasError()
            status2 = camera_2.hasError()
            print (status, status2)
            # read the camera image
            frame = camera.read()
            # display the frame
            cv2.imshow("Video Frame", frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        except KeyboardInterrupt:
            break

    # close the camera instance
    camera.release()

    # remove camera object
    del camera
