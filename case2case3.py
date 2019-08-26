import matplotlib.pyplot as plt
import math
import time
import numpy as np
from time import sleep
import serial
import matplotlib.pyplot as plt
import math
import time
from time import sleep
import serial
ser = serial.Serial('/dev/ttyACM3', 9600) # Establish the connection on a specific port

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
        #if show_animation:
          #  plt.plot(current.x * reso, current.y * reso, "xc")
           # if len(closedset.keys()) % 10 == 0:
            #    plt.pause(0.001)

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

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 1.0  # [m]
    robot_size = 1.0  # [m]

    ox , oy = [], []
    x=[]
    y=[]
    w=[]
    h2=[]
    a=0.0264*2*45

    #ox , oy=object_detect()
    x,y,w,h2 = object_detect()
    print(len(x))

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

    

    
    for i in range(len(x)):
        z1=int(x[i]*a)
        z2=int(y[i]*a)
        z3=int(w[i])
        z4=int(h2[i])
        print("obj")
        print(z1)
        print(z2)
        print(z3)
        print(z4)
        print("ob1")
        '''
        for j in range(z1,z1+z3+1):
            flag=0
            for l in (ox):  
                if(l==j):
                    flag=1
                else:
                    flag=0
            if(flag==0):
                ox.append(j)
                oy.append(z2)

        flag=0        
            
        for k in range(z2-z4,z2+1):
            flag=0
            for l in (ox):  
                if(l==k):
                    flag=1
                else:
                    flag=0
            if(flag==0):
                ox.append(z1)
                oy.append(k)

        flag=0  






        for g in range(z1,z1+z3+1):
            flag=0
            for l in (ox):  
                if(l==g):
                    flag=1
                else:
                    flag=0
            if(flag==0):
                ox.append(g)
                oy.append(z2-z4)

        flag=0  






        for h in range(z2-z4,z2+1):
            flag=0
            for l in (ox):  
                if(l==h):
                    flag=1
                else:
                    flag=0
            if(flag==0):
                ox.append(z1+z3)
                oy.append(h)

        flag=0  



        '''





            
        
    






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





    rx, ry = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    print(rx)
    print(ry)


    if(z1<15 and z2<10):
        #move the motor right for some time and then make it move forward
        print("tom1")
        ser.write(str.encode(chr(82)))
        sleep(0.4)
        print("tom2")
        ser.write(str.encode(chr(90)))
        print("stopped")
        while(1):
            ser.write(str.encode(chr(83)))

    #p is left
    #q is right
    elif((z1<40 and z1>20) or (z2>10 and z1<20)):
        print("right")
        ser.write(str.encode(chr(81)))#right
        sleep(0.8)
        print("forward")
        ser.write(str.encode(chr(70)))#forward1
        sleep(0.7)
        print("left")
        ser.write(str.encode(chr(80)))#left
        sleep(0.6)
        print("forward")
        ser.write(str.encode(chr(67)))#foward2
        sleep(1.5)
        print("stopped")
        while(1):
            ser.write(str.encode(chr(83)))




            







    import numpy as np
    '''
    rx.reverse()
    ry.reverse()
    c=0
    for i in rx:
        if(c==15):
            x11=i
            c=c+1
        elif(c==30):
            x22=i
            c=c+1
        elif(c==45):
            x33=i
            c=c+1
        else:
            c=c+1
        
    c=0
    for i in rx:
        if(c==15):
            y11=i
            c=c+1
        elif(c==30):
            y22=i
            c=c+1
        elif(c==45):
            y33=i
            c=c+1
        else:
            c=c+1

    velocity=5.4978
    theta1=(np.arctan((y22-y11)/(x22-x11))*180)/3.141
    distance1=np.sqrt((x22-x11)*(x22-x11)+(y22-y11)*(y22-y11))
    #time1=distance1/velocity
    time1=0.03183*theta1
    #ser.write(str.encode(chr(82)))
    #sleep(time1)
    time=0
    while 1:
        time=time+0.2
        ser.write(str.encode(chr(82)))
        if(time==time1):
            break
    w=(distance1/velocity)/100

    time=0
    while 1:
        time=time+0.2
        ser.write(str.encode(chr(90)))
        if(time==w):
            break




    distance2=np.sqrt((x33-x22)*(x33-x22)+(y33-y22)*(y33-y22))





    distance2=np.sqrt((50.0-x33)*(50.0-x33)+(50.0-y33)*(50.0-y33))
    

    '''
    '''
    n = int(len(rx))
    temp1x=list()
    temp2x=list()
    temp1y=list()
    temp2y=list()
    grad=list()
    for i in range (0,n-1):
        if(i % 2 == 0):
            temp1x.append(rx[i])
        else:
            temp2x.append(rx[i])

    for j in range (0,n-1):
        if(j % 2 == 0):
            temp1y.append(ry[j])
        else:
            temp2y.append(ry[j])

    for k in range (0,int((n-1)/2)):
        if((temp2x[k] - temp1x[k]) == 0):
            grad.append(np.abs(0.0))
            continue
        grad.append(np.abs(float((temp2y[k] - temp1y[k])/(temp2x[k] - temp1x[k]))))

    print(grad)


    #int e=(gy-sy)/(sx-sx)

    #if(e==45):
        #ser.write(st.encode(chr(82)))

            

    '''


'''
    
    c=0
    for i in grad:
        if(i==1.0):
            print("tom1")
            ser.write(str.encode(chr(82)))
            c=c+1
            if(c==3):
                ser.write(str.encode(chr(65)))
                break
            #print(ser.readline())
        elif(i==0.0):
            print("tom2")
            ser.write(str.encode(chr(90)))
            c=c+1
            #print(ser.readline())
        else:
            print("tom3")
            ser.write(str.encode(chr(76)))
            c=c+1
            #print(ser.readline())
        sleep(0.1)
    print("stopped")
    while 1:
        ser.write(str.encode(chr(83)))
    
'''
    


from imutils.video import VideoStream
import argparse
import datetime
import imutils
import time
import cv2
import urllib.request


#parsing arguments 
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the video file")
ap.add_argument("-a", "--min-area", type=int, default=500, help="minimum area size")
args = vars(ap.parse_args())




def object_detect():
        #reading camera feed
    url='http://192.168.43.1:8080/shot.jpg'

    while True:

# Use urllib to get the image from the IP camera
        imgResponse = urllib.request.urlopen(url)
 
 # Numpy to convert into a array
        imgNp = np.array(bytearray(imgResponse.read()),dtype=np.uint8)
 
 # Decode the array to OpenCV usable format
        frame = cv2.imdecode(imgNp,-1)
        cv2.imshow('r',frame)
        # resize the frame, convert it to grayscale, and blur it
        frame = imutils.resize(frame, width=500)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
     
        # compute the absolute difference between the current frame and
        # first frame
        thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)[1]
        
        # dilate the thresholded image to fill in holes, then find contours
        # on thresholded image
        thresh = cv2.dilate(thresh, None, iterations=2)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        p = list()
        q = list()
        r = list()
        s = list()
        i = 0
     
        # loop over the contours
        for c in cnts:
            # if the contour is too small, ignore it
            if cv2.contourArea(c) < args["min_area"]:
                continue
     
            # compute the bounding box for the contour, draw it on the frame,
            # and update the text
            
            (x1, y1, w1, h1) = cv2.boundingRect(c)
            
            #if the coordinates of the oject and bot are same it will skip one iteration of the loop
            '''if (x1,y1,w1,h1) == (x,y,w,h):
                continue''' 
            
            #this is the avoiding part or the not colliding with the objects part
            a=0.0264*2
            cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 2)
            if(x1*a==0):
                continue
            p.append(int(x1*a))
            q.append(int(y1*a))
            r.append(int(w1*a))
            s.append(int(h1*a))





            

           
        # draw the text and timestamp on the frame
        cv2.imshow("Thresho", thresh)
        cv2.imshow("grayo", gray)
        cv2.imshow("frameo", frame)
        
        time.sleep(3)
        return p,q,r,s 
   


if __name__ == '__main__':
    main()

    
    
    


