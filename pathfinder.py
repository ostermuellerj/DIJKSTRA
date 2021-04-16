import cv2
import numpy as np
import math,random


STARTX=20
STARTY=20

ENDX=90
ENDY=70

def blackWhite(img, threshold=128):
	img[img>threshold]=255
	img[img<threshold]=0
	return img
"""
- cost = (dx^2 + dy^2)^.5 + dz^2 distances in px
- get terrain and change color of roads you build
- pre-game day: calculate cost from your city to all others
- game day: send in text file of points for each road

- city1 name
- city2 name
- x1 y1
- x2 y2
...

- game day map is black and white. cannot build road on black. can on white
- game day will have text file of existing roads

- build 3 roads with lowest cost
"""


#add black rim around outside of terrain


#get connected neighbors
def getNeighbors(cell,terrain):
	x,y=cell
	m=[] #valid neighbors
	for dx in (-1,0,1):
		for dy in (-1,0,1):
			if dx==dy==0:
				continue
			if terrain[y+dy,x+dx]!=0: #if !black then the cell is a valid option, and add to potential neighbors
				m.append((x+dx,y+dy))
	return m

cost=cv2.imread("maze.png",0)#[STARTY-1:ENDY,STARTX-1:ENDX]
maze=cv2.imread("maze.png",0)

#MAKE BOX:
cv2.rectangle(maze,(STARTX-1,STARTY-1),(ENDX+1,ENDY+1),0)

h,w=cost.shape[:2]
#I am going to talk in terms of cell coordinates instead of pixel coordinates.

dh=abs(STARTY-ENDY)
dw=abs(STARTX-ENDX)

Y=ENDY+1
X=ENDX+1

#distance=np.zeros((h,w),dtype=np.float32)+h*w+1
distance=np.zeros((dh+Y,dw+X),dtype=np.float32)+(dh+Y)*(dw+X)+1
print(distance.shape)
distance[STARTY,STARTX]=0

visited=set()
fringe=set()
current=(STARTX,STARTY)
fringe.add(current)
i=0
while True:
	currentCost=dh*dw+1
	if not fringe:
		break
	for xf,yf in fringe:
		if distance[yf,xf]<currentCost:
			current=(xf,yf)
			currentCost=distance[yf,xf]
	x,y=current
#	if i%5==0: #animation
#		cv2.imshow("",cv2.resize(maze,(0,0),fx=4,fy=4,interpolation=0))
#		cv2.waitKey(1)	
	i+=1
	for xn,yn in getNeighbors(current,cost):
		if (xn,yn) not in visited:
			fringe.add((xn,yn))
		#dist=(dx^2 + dy^2)^.5 + dz^2
		dist=math.hypot(x-xn,y-yn) + np.square(maze[y,x]-maze[yn,xn])
		if xn<0:
			print("NEGATIVE",dist,xn,yn)
		#print(distance[77,94])		
		if distance[yn,xn]>currentCost+dist: #adjust
			distance[yn,xn]=currentCost+dist
	visited.add(current)
	fringe.remove(current)
	#go through the fringe and pick the smallest
cv2.waitKey(0)
cv2.destroyAllWindows()

maze_path=cv2.imread("maze.png",0)
cv2.rectangle(maze_path,(STARTX-1,STARTY-1),(ENDX+1,ENDY+1),0)
current=(ENDX,ENDY)
maze_path[ENDY,ENDX]=255
while current!=(STARTX,STARTY):
	x,y=current
	print(current)
	best=current
	bestDistance=255
	for xn,yn in getNeighbors(current,cost): #find min neighbor
		dist=math.hypot(x-xn,y-yn) + np.square(maze[y,x]-maze[yn,xn])
		mod=1
		if abs(distance[yn,xn]-distance[y,x]+mod*dist)<.1 and dist<bestDistance:
			best=(xn,yn)
	current=best	
	maze_path[best[1],best[0]]=255
	maze_path[y,x]=255
	cv2.imshow("",cv2.resize(maze_path,(0,0),fx=4,fy=4,interpolation=0))
	cv2.waitKey(1)
cv2.waitKey(0)
cv2.destroyAllWindows()
	
cv2.imshow("solved.png",cv2.resize(blackWhite(maze_path,254),None,fx=5, fy=5, interpolation = cv2.INTER_NEAREST))
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("solved.png",maze_path)
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
tentative distance, set it as the new "current node", and go back to step 3."""
