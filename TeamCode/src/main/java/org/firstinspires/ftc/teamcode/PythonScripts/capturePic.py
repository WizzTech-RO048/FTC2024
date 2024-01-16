import cv2
import argparse

camera = cv2.VideoCapture(1)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 448)

ap = argparse.ArgumentParser()
ap.add_argument("-H", "--height", required=False, type=int, default=0, help="target height for reshape")
ap.add_argument("-W", "--width", required=False, type=int, default=0, help="target width for reshape")
args = vars(ap.parse_args())

while True:
    ret, frame = camera.read()
    if ret:
        cv2.imshow("da", frame)
        if cv2.waitKey(1) == ord("q"):
            if args["height"] != 0 and args["width"] != 0:
                frame = cv2.resize(frame, (args["width"], args["height"]), interpolation=cv2.INTER_AREA)

            cv2.imwrite("image.png", frame)

            break


