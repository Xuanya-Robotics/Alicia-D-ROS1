import cv2

def open_camera(camera_index=0, width=None, height=None):
    """
    Opens the specified camera and displays the feed.

    Args:
        camera_index: The index of the camera to open (e.g., 0 for /dev/video0).
    """
    # Create a VideoCapture object
    cap = cv2.VideoCapture(camera_index)
    if width is not None and height is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        print(f"Error: Could not open camera with index {camera_index}.")
        return

    window_name = 'Camera Feed'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    if width is not None and height is not None:
        cv2.resizeWindow(window_name, width, height)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # Display the resulting frame
        # fix the image show 
        cv2.imshow('Camera Feed', frame)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) == ord('q'):
            # save image:
            cv2.imwrite('captured_image.jpg', frame)
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    open_camera(0, width=1280, height=720)