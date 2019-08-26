import numypy as np 
import math
#getting an array of gradients from the device called g ranging from g[0] to g[int((n-1)/2)]
for i in range (0, int((n-1)/2)):
	anglesn_rad[i] = np.arctan(g[i])
	anglesn_deg[i] = (anglesn_rad[i] * 180 ) / math.pi #finding angles in degrees of new path due to error
	angleso_rad[i] = np.arctan(grad[i])
	angleso_deg[i] = (angleso_rad[i] * 180 ) / math.pi #finding angles in degrees of expected actual path
	t = anglesn_deg[i] - angleso_deg[i] #difference error in angles
	k = (t * math.pi) / 180 #converting back to radians
	if(t >= 15): #if angle difference is more than 15 degrees  we do negative correction
		g[i] = np.tan(np.arctan(g[i]) - k) 
	elif(t <= -15 ):
		g[i] = np.tan(np.arctan(g[i]) + k)
	else:
		continue

for i in range (0, int((n-1)/2)):
	return g[i]

		

