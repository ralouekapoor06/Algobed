#detecting object using contrast between background and objects
#this is the same without the motion detect code
#except that the frameDelta(difference between consecutive frames)
#is absent in this code

import cv2
import numpy as np
import matplotlib as plt
import imutils
import argparse
import urllib.request

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the video file")
ap.add_argument("-a", "--min-area", type=int, default=500, help="minimum area size")
args = vars(ap.parse_args())


#reading camera feed
with urllib.request.urlopen("http://192.168.0.100:8080/videocv") as url:
	cam = url.read()
while cap.isOpened():
	cam=np.array(bytearray(cam,'utf8'),dtype=np.uint8)
    cam=cv2.imdecode(cam,-1)
    cap,frame =cam.read()
       
	# resize the frame, convert it to grayscale, and blur it
	frame = imutils.resize(frame, width=500)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (21, 21), 0)
 
	# compute the absolute difference between the current frame and
	# first frame
	thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)[1]
 	
	# dilate the thresholded image to fill in holes, then find contours
	# on thresholded image
	thresh = cv2.dilate(thresh, None, iterations=2)
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
 
	# loop over the contours
	for c in cnts:
		# if the contour is too small, ignore it
		if cv2.contourArea(c) < args["min_area"]:
			continue
 
		# compute the bounding box for the contour, draw it on the frame,
		# and update the text
		
		(x1, y1, w1, h1) = cv2.boundingRect(c) 
		
		cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 2)
		#text = "Bot moving"


		#avoid overlap

		
	# draw the text and timestamp on the frame
	cv2.imshow("Thresh", thresh)
	cv2.imshow("gray", gray)
	cv2.imshow("frame", frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the `q` key is pressed, break from the lop
	if key == ord("q"):
		break
 
# cleanup the camera and close any open windows
vs.stop() if args.get("video", None) is None else vs.release()
cv2.destroyAllWindows()