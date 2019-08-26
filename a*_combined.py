"""
A* grid based planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import matplotlib.pyplot as plt
import math

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_fianl_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gy: goal y position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)#we have the map and all the pos ready
    
    motion = get_motion_model()#This was for the motion of the bot
    

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart #calculates the position of the bot

    #do all the calculations
    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
        current = openset[c_id]

        # show graph
        if show_animation:
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if n_id in closedset:
                continue

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id not in openset:
                openset[n_id] = node  # Discover a new node

            tcost = current.cost + calc_heuristic(current, node)

            if tcost >= node.cost:
                continue  # this is not a better path

            node.cost = tcost
            openset[n_id] = node  # This path is the best unitl now. record it!

    rx, ry = calc_fianl_path(ngoal, closedset, reso)

    return rx, ry


def calc_heuristic(n1, n2):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


#making of the map
def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(xwidth)] for i in range(ywidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")
    #so we get the location of the obstacle
    #we need a different fn for this or we can harcode it
    sx,sy,gx,gy=object_detect()
    

    # start and goal position
    '''
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    '''

    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]
    
    ox , oy = [], []
    ox , oy=object_detect()
    #these are the boundries
    '''
    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        if i>20:
            oy.append(i)
        
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)
    '''
    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


combination of motion_detect , object_detect 

#this code detects movement of the bot

from imutils.video import VideoStream
import argparse
import datetime
import imutils
import time
import cv2


#parsing arguments 
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the video file")
ap.add_argument("-a", "--min-area", type=int, default=500, help="minimum area size")
args = vars(ap.parse_args())


def motion_detect():
        if args.get("video", None) is None:
            vs = VideoStream(src=0).start()
            time.sleep(2.0)
         

        #making sure the initial frame is the still frame which acts as the background
        #to the moving bot 

        firstFrame = None

        # loop over the frames of the video
        while True:
            # grab the current frame and initialize the occupied/unoccupied
            # text
            frame = vs.read()
            frame = frame if args.get("video", None) is None else frame[1]
            text = "No movement"
         
         
            # resize the frame, convert it to grayscale, and blur it
            frame = imutils.resize(frame, width=500)
            
            #purpose of grayscaling and blurring is for a clearer diffrentiation of the moving bot 
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)
         
            # if the first frame is None, initialize it
            if firstFrame is None:
                firstFrame = gray
                continue
            
            # compute the absolute difference between the current frame and
            # first frame
            #frameDelta takes care of the difference between the moving bot and background by seeing 
            #for any change in the frame
            frameDelta = cv2.absdiff(firstFrame, gray)
            
            #thresholding with a max value of 255 here means pixel intensity values less than 
            #25 get mapped to zero(black) and above 25 get mapped to 255(white) 
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
         
            # dilate the thresholded image to fill in holes, then find contours
            # on thresholded image
            thresh = cv2.dilate(thresh, None, iterations=2)
            
            #cnts is a numpy array that stores the values of the contours
            #contours are points with the same intensity or color
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
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                text = "Bot moving"
            
            # draw the text and timestamp on the frame
            cv2.putText(frame, "Bot Status: {}".format(text), (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),
                (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
         
            # show the frame and record if the user presses a key
            cv2.imshow("frame", frame)
            cv2.imshow("Thresh", thresh)
            cv2.imshow("Frame Delta", frameDelta)
            key = cv2.waitKey(1) & 0xFF
         
            # if the `q` key is pressed, break from the lop
            if key == ord("q"):
                break
         
        # cleanup the camera and close any open windows
        vs.stop() if args.get("video", None) is None else vs.release()
        cv2.destroyAllWindows()


def object_detect():
        #reading camera feed
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret,frame = cap.read();
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
            
            #if the coordinates of the oject and bot are same it will skip one iteration of the loop
            if (x1,y1,w1,h1) == (x,y,w,h):
                continue 
            
            #this is the avoiding part or the not colliding with the objects part
            cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 2)
            if(x > (x1 + w1) or x1 > (x+w)):
                flag = 1;
            if(y < (y1 + h1) or (y+h) > y1):
                flag = 2;
            else : pass






            

            
        # draw the text and timestamp on the frame
        cv2.imshow("Thresho", thresh)
        cv2.imshow("grayo", gray)
        cv2.imshow("frameo", frame)
        key = cv2.waitKey(1) & 0xFF
     
        # if the `q` key is pressed, break from the lop
        if key == ord("q"):
            break
     
    # cleanup the camera and close any open windows
    vs.stop() if args.get("video", None) is None else vs.release()
    cv2.destroyAllWindows()


#motion_detect()









if __name__ == '__main__':

    main()
