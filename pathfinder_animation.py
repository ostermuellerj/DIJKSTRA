import cv2
import numpy as np
import math,random

#get connected neighbors
def getNeighbors(cell,terrain):
	x,y=cell
	m=[]
	for dx in (-1,0,1):
		for dy in (-1,0,1):
			if dx==dy==0:
				continue
			if tuple(terrain[y+dy,x+dx])!=(0,0,0):
				m.append((x+dx,y+dy))
	return m

cost=cv2.imread("maze4.png")
maze_viz=cv2.imread("maze4.png")

h,w=cost.shape[:2]
print(h)
print(w)
#in terms of cell coordinates instead of pixel coordinates.

distance=np.zeros((h,w),dtype=np.float32)+h*w+1
distance[1,1]=0

visited=set()
fringe=set()
current=(1,1)
fringe.add(current)
i=0
while True:
	#print(len(fringe))
	currentCost=h*w+1
	if not fringe:
		break
	for xf,yf in fringe:
		if distance[yf,xf]<currentCost:
			current=(xf,yf)
			currentCost=distance[yf,xf]
	x,y=current
	maze_viz[y,x,1]=0
	if i%5==0:
		cv2.imshow("",cv2.resize(maze_viz,(0,0),fx=4,fy=4,interpolation=0))
		cv2.waitKey(1)
	i+=1
	for xn,yn in getNeighbors(current,cost):
		if (xn,yn) not in visited:
			fringe.add((xn,yn))
		dist=math.hypot(x-xn,y-yn)
		mod=1
		if tuple(cost[yn,xn])==(255,0,0):
			mod=2
		if tuple(cost[yn,xn])==(0,255,0):
			mod=.5
		if distance[yn,xn]>currentCost+mod*dist:
			distance[yn,xn]=currentCost+mod*dist
	visited.add(current)
	fringe.remove(current)
	#go through the fringe and pick the smallest
cv2.waitKey(0)
cv2.destroyAllWindows()

"""
distance[distance>1000]=0
distance=distance*1.0
distance/=np.max(distance)
distance*=255
distance=np.uint8(distance)
cv2.imwrite("distance.png",distance)
"""

maze_color=cv2.imread("maze4.png")
current=(w-2,h-2)
maze_color[h-2,w-2]=(0,255,0)
while current!=(1,1):
	x,y=current
	possibles=[]
	for xn,yn in getNeighbors(current,cost):
		dist=math.hypot(x-xn,y-yn)
		mod=1
		if tuple(cost[y,x])==(255,0,0):
			mod=2
		if tuple(cost[y,x])==(0,255,0):
			mod=.5
		if abs(distance[yn,xn]-distance[y,x]+mod*dist)<.1:
			possibles.append((xn,yn))
	try:
		current=random.choice(possibles)
	except:
		print(x,y)
		print(distance[y,x])
	maze_color[current[1],current[0]]=(0,255,0)
	maze_viz[y,x,0]=0

	cv2.imshow("",cv2.resize(maze_viz,(0,0),fx=4,fy=4,interpolation=0))
	cv2.waitKey(1)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("solved.png",maze_color)

"""
1)Assign to every node a tentative distance value: set it to zero for 
our initial node and to infinity for all other nodes.

2)Set the initial node as current. Mark all other nodes unvisited. 
Create a set of all the unvisited nodes called the unvisited set.

3)For the current node, consider all of its neighbors and calculate their 
tentative distances. Compare the newly calculated tentative distance to 
the current assigned value and assign the smaller one. For example, if 
the current node A is marked with a distance of 6, and the edge connecting 
it with a neighbor B has length 2, then the distance to B (through A) will 
be 6 + 2 = 8. If B was previously marked with a distance greater than 8 
then change it to 8. Otherwise, keep the current value.


4) When we are done considering all of the neighbors of the current node, 
mark the current node as visited and remove it from the unvisited set. 
A visited node will never be checked again.

If the destination node has been marked visited (when planning a route 
between two specific nodes) or if the smallest tentative distance among 
the nodes in the unvisited set is infinity (when planning a complete 
traversal; occurs when there is no connection between the initial node
 and remaining unvisited nodes), then stop. The algorithm has finished.

Otherwise, select the unvisited node that is marked with the smallest 
tentative distance, set it as the new "current node", and go back to step 3.
"""
