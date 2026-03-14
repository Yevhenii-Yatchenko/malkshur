import jetson.utils

def display_csi_camera():
    # Create the camera instance
    camera = jetson.utils.gstCamera(1280, 720, "/dev/video0")  # You may need to adjust resolution and camera index (0) accordingly.

    # Create the display instance
    display = jetson.utils.glDisplay()

    # Main loop to capture and display frames from the camera
    while display.IsOpen():
        # Capture a frame from the camera
        img, width, height = camera.CaptureRGBA(zeroCopy=1)

        # Render the frame
        display.RenderOnce(img, width, height)

        # Update the window title with the current frames per second (FPS)
        display.SetTitle("CSI Camera | {:.1f} FPS".format(display.GetFPS()))

        # Check for user exit (Esc key)
        if display.IsClosed():
            break

# Call the main function to display the camera feed
if __name__ == "__main__":
    display_csi_camera()