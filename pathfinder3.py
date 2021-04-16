import cv2
import numpy as np
import heapq
import math,random
import pickle

#Squeeville 2890 985 2463528
file = open("data.txt","w")

scale=.2

terr=cv2.imread("maze.png",0)*1.0
cities=cv2.imread("cities.png",0)

h,w=terr.shape[:2]

class heap:
	def __init__(self):
		self.h=[]
		self.entries={}
		self.count=0
	def push(self,x,y,cost):
		if(x,y) not in self.entries:
			self.count+=1
			entry=[cost,x,y,0]
			heapq.heappush(self.h, entry) #if removed 0 or 1
			self.entries[(x,y)]=entry
		else:
			if cost<self.entries[(x,y)][0]:
				#update
				entry=[cost,x,y,0]
				self.entries[(x,y)][-1]=1
				self.entries[(x,y)]=entry	
				heapq.heappush(self.h, entry)
	def pop(self):
		cost,x,y,r=heapq.heappop(self.h)
		while r:
			cost,x,y,r=heapq.heappop(self.h)
		self.count-=1
		return x,y
					
def getNeighbors(x,y):
	m=[] #valid neighbors
	for dx in (-1,0,1):
		for dy in (-1,0,1):
			if dx==dy==0 or dx+x<0 or dy+y<0 or dx+x>w or dy+y>h:
				continue
			m.append((x+dx,y+dy))
	return m

#START CITY:

#two town map
#town_x=833
#town_y=1841

#small map
#town_x=512
#town_y=384

#smaller map
#town_x=50
#town_y=70

#Squeeville 2890 985 2463528
town_x=2890
town_y=985

#Squirrelton 1923 1923 22087805

costs=np.zeros((h,w),dtype=np.float64)+h*w*10+1

def makeCostMap():
	print("GENERATING COST MAP")
	file.write("GENERATING COST MAP"+"\n") 

	fringe=heap()
	
	for i in range(town_x-1,town_x+251):
		for j in range(town_y-1,town_y+251):
			costs[j,i]=0
			fringe.push(i,j,0)
		
	#MAKE COST MAP:	
	i=0		
	while fringe.count:
		i+=1
		if i%10000==0: #update frequency
			#print (fringe.count)
			viz=costs*1.0
			viz[viz>8000]=0
			viz/=np.max(viz)
			viz*=255.9
			viz=np.uint8(viz)
	#		cv2.imshow("",viz[::10,::10])
			cv2.imshow("",cv2.resize(viz[::20,::20],(0,0),fx=4,fy=4,interpolation=0))
			cv2.waitKey(1)
		x,y =fringe.pop()
		cost=costs[y,x]
		#z=terr[y,x]
		for xn,yn in getNeighbors(x,y):
			if (xn,yn) in fringe.entries:
				continue
			if xn < w and yn < h:
				dz=terr[y,x]-terr[yn,xn]
				dcost=((x-xn)**2+(y-yn)**2)**.5+dz**2+999999*cities[yn,xn] #sqrt(dx^2+dy^2)+dz^2 and cities
				if cost+dcost<costs[yn,xn]:
					costs[yn,xn]=cost+dcost
					fringe.push(xn,yn,cost+dcost)
					
	#PICKLE
	pickle.dump(costs, open("dijkstramap.pickle", "wb"))
					
	cv2.imwrite("costs.png",costs)
#	cv2.waitKey(0)
#	cv2.destroyAllWindows()

#ENDX/Y = CITY YOU ARE TRAVELLING TO
def findPath(ENDX, ENDY, currentMap):
	maze_viz=cv2.imread("maze.png",0)
	maze_path=cv2.imread("maze.png",0)
	maze_path=currentMap
	current=(ENDX,ENDY) #start from the bottom
	k=0
	print(costs[current])
	file.write(str(costs[current])+"\n") 
	while True:
		if current[0] > town_x-2 and  current[1] > town_y-2 and current[0] < town_x+251 and current[1] < town_y+251:
			break
		x,y=current
		#print(current)
		best=current
		bestCost=99999
		for xn,yn in getNeighbors(current[0],current[1]): #find min neighbor
			if costs[yn,xn]<bestCost: #and cities[yn,xn]!=255: # and maze_path[yn,xn]!=255
				best=xn,yn
				bestCost=costs[yn,xn]
		current=best	
		maze_path[best[1],best[0]]=255
		maze_path[y,x]=255
		k+=1
	cv2.imwrite("solved.png",maze_path)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	return maze_path
	
def animatePath(ENDX, ENDY, currentMap):
	maze_viz=cv2.imread("maze.png",0)
	maze_path=cv2.imread("maze.png",0)
	maze_path=currentMap
	current=(ENDX,ENDY)
	k=0
	maze_path[town_y-1,town_x-1]=254
	maze_path[town_y+251,town_x+251]=254
	
	print(costs[current])
	file.write(str(costs[current])+"\n") 
	i=0
	while True:
		if current[0] > town_x-1 and  current[1] > town_y-1 and current[0] < town_x+251 and current[1] < town_y+251:
			continue
		x,y=current
		#print(current)
		best=current
		bestCost=99999
		for xn,yn in getNeighbors(current[0],current[1]): #find min neighbor
			if costs[yn,xn]<bestCost: #and cities[yn,xn]!=255: # and maze_path[yn,xn]!=255
				best=xn,yn
				bestCost=costs[yn,xn]
		current=best	
		maze_path[best[1],best[0]]=255
		maze_path[y,x]=255
		k+=1
	#	cv2.imshow("",cv2.resize(maze_path,(0,0),fx=.2,fy=.2,interpolation=0))
		i+=1
		if i%100==0: #update frequency	
			cv2.imshow("",cv2.resize(maze_path,(0,0),fx=scale,fy=scale,interpolation=0))
			cv2.waitKey(1)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	cv2.imwrite("solved.png",maze_path)
	return maze_path

cityList = [[512,384],[700,2600],[930,3660],[2221,869],[2296,2986],[833,1841],[276,3628],[2800,1955],[1923,1923],[3333,2578],[2487,3360],[130,2409],[3259,1073],[1500,1500]]
#[2890,985],[3000,3000],
def bestPath(index):
	best=0,0
	bestCost=10000
#	if index==9:
#		print(costs)
	for i in range (cityList[index][0]-1, cityList[index][0]+251):
		for j in range (cityList[index][1]-1, cityList[index][1]+251):
			if costs[j,i]<bestCost:
				bestCost=costs[j,i]
				best=i,j
	return best

#makeCostMap()

print("Reading pickled array")
file.write("Reading pickled array"+"\n") 
costs = pickle.load(open("dijkstramap.pickle", "rb"))

coord=bestPath(0)

supermap = cv2.imread("maze.png")
for i in range (len(cityList)):
	print("city: #" + str(i))
	file.write("city: #" + str(i) + str(cityList[i])+ " ")
	coord=bestPath(i)
	supermap = findPath(coord[0],coord[1], supermap)
cv2.imwrite("solved.png",supermap)	

print("costs[city 9]: ", costs[2575,3450])
file.write("costs[city 9]: " + str(costs[2575,3450])+"\n")
print("costs[city 10]: ", costs[3358,2738])
file.write("costs[city 10]: " + str(costs[3358,2738])+"\n")
#x=3450
#y=2575

#cv2.imshow("",cv2.resize(findPath(coord[0],coord[1]),(0,0),fx=scale,fy=scale,interpolation=0))

#coord2=bestPath(0)
#findPath(findPath(coord[0],coord[1])

cv2.waitKey(0)		
cv2.destroyAllWindows()
#findPath(5, 2)
viz=costs*1.0
viz[viz>8000]=0
viz/=np.max(viz)
viz*=255.9
viz=np.uint8(viz)
#		cv2.imshow("",viz[::10,::10])
cv2.imwrite("costs.png",viz)
cv2.waitKey(0)
cv2.destroyAllWindows()

file.close() 
