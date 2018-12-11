#mesh_sp.py
#COMP7720 Online Algorithms 2018 Fall
#Author: Ying Ying Liu
#Date: 2018-12-10

import random
import sys
import numpy as np
import time
import heapq

class Vertex: 
	def __init__(self, node):
		self.id = node 
		#self.x = node%(n+1)
		#self.y = node/(n+1)
		self.visited = False
		# set dijkstra distance to inifinity for all nodes 
		self.dijkstraDistance = sys.maxint 
		# distance for greedy and weighted alg
		self.xActTime = sys.maxint 
		self.yActTime = sys.maxint 
		self.xEstTime = sys.maxint 
		self.yEstTime = sys.maxint 		
		#paths 
		self.previous = None
		self.next = None  
	
	def set_previous(self, node): 
		self.previous = node 

	def set_next(self, node): 
		self.next = node 		

	def set_dijkstraDistance(self, dist): 
		self.dijkstraDistance = dist 

	def reset_vertex(self):
		self.visited = False
		# set dijkstra distance to inifinity for all nodes 
		self.dijkstraDistance = sys.maxint 
		# distance for greedy and weighted alg
		self.xActTime = sys.maxint 
		self.yActTime = sys.maxint 
		self.xEstTime = sys.maxint 
		self.yEstTime = sys.maxint 
		#paths 
		self.previous = None
		self.next = None		

class mesh(object):
	def __init__(self, n):
		self.n = n 
		self.xWeight = np.ones((n+1,n), dtype=np.int32)
		self.yWeight = np.ones((n+1,n), dtype=np.int32)

	#just create a time based mesh, inverval: 1 time step
	def create_time_table(self, elapse): 
		self.xWeightTimeTable = np.ones((elapse,self.n+1,self.n), dtype=np.int32)
		self.yWeightTimeTable = np.ones((elapse,self.n+1,self.n), dtype=np.int32)
		for i in range(0, elapse):
			for x in range(0, self.n):
				for y in range(0, self.n+1):
					self.xWeightTimeTable[i][y][x] = random.randrange(1,mu+1)
		for i in range(0, elapse):
			for y in range(0, self.n):
				for x in range(0, self.n+1):
					self.yWeightTimeTable[i][x][y] = random.randrange(1,mu+1)

	def reset(self):
		self.xWeight = np.ones((self.n+1,self.n), dtype=np.int32)
		self.yWeight = np.ones((self.n+1,self.n), dtype=np.int32)	

	def add_vertices(self): 
		self.vertex_dict = {}
		for y in range(0, self.n+1):
			for x in range(0, self.n+1):
				node = y*(self.n+1)+x
				new_vertex = Vertex(node)
				self.vertex_dict[node] = new_vertex
	
	def reset_vertices_ranges(self,node): 
		currX = node%(self.n+1)
		currY = node/(self.n+1)
		for y in range(currY, self.n+1):
			for x in range(currX, self.n+1):
				if (x == currX and y == currY):
					continue
				node = y*(self.n+1)+x
				new_vertex = Vertex(node)
				self.vertex_dict[node] = new_vertex				
	
	def reset_vertices(self): 
		for y in range(0, self.n+1):
			for x in range(0, self.n+1):
				node = y*(self.n+1)+x
				vertex = self.vertex_dict[node]
				vertex.reset_vertex

	def find_path(self, start, end):
		currVertex = self.vertex_dict[end]
		path = [[end, currVertex.dijkstraDistance]]
		#while currVertex.previous is not None: 
		while currVertex.id != start:
			prevNode = currVertex.previous
			prevVertex = self.vertex_dict[prevNode]
			#print 'currVertex',currVertex.id,'prevVertex',prevVertex.id
			#print 'find path:', path
			path.append([prevNode, prevVertex.dijkstraDistance])
			currVertex = prevVertex
			#print 'currVertex now:',currVertex.id,'prevVertex now:',currVertex.previous
		path.reverse()
		return path

def set_nextNode(mesh, node, algTimeTable, goRight, goDown, time):
	x = node%(mesh.n+1)
	y = node/(mesh.n+1)
	currVertex = mesh.vertex_dict[node]
	if goRight:
			nextNode = node + 1
			nextVertex = mesh.vertex_dict[nextNode]

	if goDown: 
			nextNode = node + (mesh.n + 1)
			nextVertex = mesh.vertex_dict[nextNode]		
	
	currVertex.next = nextNode	
	nextVertex.previous = node	
	nextVertex.xActTime = currVertex.xActTime
	nextVertex.yActTime = currVertex.yActTime
	#next vertex is appended to algTimeTable if the vehicle is on the way to it
	#the calling method needs to check when vehicle actually arrives at next node, and that's the next time when the greedy method is called
	#since least weight is 1 and traffic changes every 1 time step,
	#it is guaranteed that at next time step, the alg is still on the way or arrived at nextVertex	
	algTimeTable.append([nextNode, time])
	return

def calculate_PredictArrivalTime(mesh, currTime, currX, currY, directionFlag):
	distInFuture = 0
	j = 0
	while distInFuture < 1: 
		#print 'currX',currX,'currY',currY,'j',j,'distInFuture',distInFuture		
		if directionFlag == 0: #go down
			distInFuture = distInFuture + 1*1.0/mesh.yWeightTimeTable[currTime+j][currX][currY]
		if directionFlag == 1: #go right
			distInFuture = distInFuture + 1*1.0/mesh.xWeightTimeTable[currTime+j][currY][currX]
		j = j+1
		if j > 10: #j should not be larger than mu, set as 5. This is just to avoid inifite loop in case of a bug
			break
	return j

#assume that the destination is always the right-bottom node 
def greedy(mesh, node, time, algTimeTable, algFlag):

	x = node%(mesh.n+1)
	y = node/(mesh.n+1)

	#destination reached, just record the time. 
	if x == mesh.n and y == mesh.n: 
		algTimeTable.append([node, time])
		return

	currVertex = mesh.vertex_dict[node]
	goRight = False
	goDown = False 

	if x < mesh.n and y < mesh.n: 
		if algFlag < 3:
			if algFlag == 0 or algFlag == 1:
				nextEstX = mesh.xWeightTimeTable[time][y][x] 
				nextEstY = mesh.yWeightTimeTable[time][x][y]
			if algFlag == 2: 
				nextEstX = calculate_PredictArrivalTime(mesh, time, x, y, 1)
				nextEstY = calculate_PredictArrivalTime(mesh, time, x, y, 0)		
		#weighted 
		if algFlag >= 3:
			if algFlag == 3 or algFlag == 4:
				weightX = 0
				for i in range(x,mesh.n):
					weightX = weightX + mesh.xWeightTimeTable[time][y][i] 
				weightY = 0
				for i in range(y,mesh.n):
					weightY = weightY + mesh.xWeightTimeTable[time][x][i] 
				nextEstX = 1.0*mesh.xWeightTimeTable[time][y][x]/weightX
				nextEstY = 1.0*mesh.yWeightTimeTable[time][x][y]/weightY
			if algFlag == 5:
				weightX = 0
				predictTime = time
				for i in range(x,mesh.n):
					j = calculate_PredictArrivalTime(mesh, predictTime, i, y, 1)
					weightX = weightX + j
					predictTime = predictTime + j 				
				weightY = 0
				predictTime = time
				for i in range(y,mesh.n):
					j = calculate_PredictArrivalTime(mesh, predictTime, x, i, 0)
					weightY = weightY + j
					predictTime = predictTime + j
				nextEstX = 1.0*calculate_PredictArrivalTime(mesh, time, x, y, 1)/weightX
				nextEstY = 1.0*calculate_PredictArrivalTime(mesh, time, x, y, 0)/weightY				

		#deterministic 
		if algFlag == 0 or algFlag == 2 or algFlag == 3 or algFlag == 5:
			if (nextEstX < nextEstY): 
				goRight = True 
				goDown = False 
			if (nextEstX > nextEstY):
				goDown = True
				goRight = False 
			if (nextEstX == nextEstY):
				rand = random.randrange(0,2)
				if rand == 0:
					goRight = True
					goDown = False 
				if rand == 1: 
					goDown = True
					goRight = False
		#randomization
		if algFlag == 1 or algFlag == 4:
			#probX = 1.0*nextEstX/(nextEstX+nextEstY) #wrong, the smaller edge should have higher probably 
			probX = 1-1.0*nextEstX/(nextEstX+nextEstY)
			probY = 1-probX
			rand = random.uniform(0, 1)
			if rand <= probX:
				goRight = True
				goDown = False
			else:
				goDown = True
				goRight = False 

	if x == mesh.n and y < mesh.n: 
		goDown = True
		goRight = False
	if y == mesh.n and x < mesh.n: 
		goRight = True
		goDown = False

	set_nextNode(mesh, node, algTimeTable, goRight, goDown, time)
	return

def aco(mesh, limitAntNumberFlag): 
	mesh.add_vertices()
	numAnts = pow(2,mesh.n/10) 
	if limitAntNumberFlag == 1:#don't use a lot of ants for performance concern. at most 32
		if numAnts > 32:
			numAnts = 32
	paths = np.zeros((numAnts,2*mesh.n+1,2), dtype=np.int32) #second dimension: 2*n+1 nodes on any path on the mesh, third dimension: index 0: node id, index 1: cost(arrival time) 
	pheromone = np.zeros((2,mesh.n+1,mesh.n), dtype=np.float32) #0 for y weights, 1 for x weights

	q=0.5 #threshold in equation 1
	q0=0.5
	beta = -1.0 #in equation 1 and 2, when beta is -1.0 rather than -2.0, iteration quality fluctuates(i.e. converge later), like the experiment on ro
	gamma_decay = 0.1 #global phermone decay parameter, doesn't really matter
	ro = 0.1 #local phermone decay parameter, if ro increases, the quality of each iteration may have more fluctuation
	Q = 100 #constant used in equation 5, doesn't really matter what the value is, the result is the same	
	#initialize pheromone 
	#print 'aco'
	dijkstra(mesh, 0, 0, 0)
	#print 'aco initialization'
	lastNode = (mesh.n+1)*(mesh.n+1)-1
	lastVertex = mesh.vertex_dict[lastNode]
	iniLen = lastVertex.dijkstraDistance
	#tow0 = 1.0/(mesh.n*mesh.n*iniLen)
	tow0 = 1.0/(iniLen)
	#print 'tow0',tow0	
	for i in range(0,2):
		for j in range(0,mesh.n+1):
			for k in range(0,mesh.n):
				pheromone[i][j][k] = tow0
				#print 'pheromone[',i,'][',j,'][',k,']',pheromone[i][j][k]

	#place ants
	#print 'place',numAnts,' ants'
	#make sure they are on different paths
	for i in range(0,numAnts): 
		#the path is the binary code of the ant id
		#0:go down 
		#1:go right
		paths[i][0] = [0,0] #start at node 0 at time 0
		currNode = 0
		currTime = 0
		bitStr = "{0:b}".format(i)
		#print 'ant ',i,'bitStr',bitStr
		for j in range(0,len(bitStr)):
			currX = currNode%(mesh.n+1)
			currY = currNode/(mesh.n+1)
			direction = int(bitStr[j])
			#print 'currX',currX,'currY',currY,'direction',direction
			if direction == 0:
				nextNode = currNode + (mesh.n+1)
			if direction == 1:
				nextNode = currNode + 1				
			#calculate the real cost on the path
			actArrivalTime = currTime + calculate_PredictArrivalTime(mesh, currTime, currX, currY, direction)
			#print 'nextNode',nextNode,'act arrival time',actArrivalTime
			paths[i][j+1] = [nextNode, actArrivalTime]
			currNode = nextNode
			currTime = actArrivalTime
		#print 'ant:',i
		#print 'path'
		#print paths[i]
		for j in range(len(bitStr), 2*mesh.n+1):	
			currX = currNode%(mesh.n+1)
			currY = currNode/(mesh.n+1)
			if currX == mesh.n and currY == mesh.n:
				break		
			if currX == mesh.n and currY < mesh.n: 
				direction = 0
			if currY == mesh.n and currX < mesh.n:
				direction = 1
			if currX < mesh.n and currY < mesh.n: 
				#Now find the next node using ACO
				#go down: 
				val0 = 1.0*pheromone[0][currX][currY]*pow(mesh.yWeightTimeTable[currTime][currX][currY]*1.0,beta)
				#go right:
				val1 = 1.0*pheromone[1][currY][currX]*pow(mesh.xWeightTimeTable[currTime][currY][currX]*1.0,beta)
				#print 'j',j,'currTime',currTime,'mesh.yWeightTimeTable[currTime][currX][currY]',mesh.yWeightTimeTable[currTime][currX][currY],'pheromone[0][currX][currY]',pheromone[0][currX][currY],'val0',val0,'val1',val1
				q = random.uniform(0, 1)
				if q<=q0: 
					#exploitation
					if val0 >= val1:
						direction = 0
					else:
						direction = 1
				else: 
					#exploration
					pk0 = 1.0*val0/(val0+val1)
					pk1 = 1-pk0
					rand = random.uniform(0, 1)
					if rand <= pk0:
						direction = 0
					else:
						direction = 1 

			#calculate the real cost on the path
			#although the actual travel time on edge is calculated by calling the look ahead function, this is 
			#not a advice, as the ant is already going on this direction
			actTravelTimeOnEdge = calculate_PredictArrivalTime(mesh, currTime, currX, currY, direction)
			actArrivalTime = currTime + actTravelTimeOnEdge 
			if direction == 0:
				nextNode = currNode + (mesh.n+1)			
				#update pheromone: the smaller the cost, the larger increase on pheromone. ro is decay factor
				pheromone[0][currX][currY] = (1-ro)*pheromone[0][currX][currY]+ro/actTravelTimeOnEdge
			if direction == 1:
				nextNode = currNode + 1	
				pheromone[1][currY][currX] = (1-ro)*pheromone[1][currY][currX]+ro/actTravelTimeOnEdge			
			#print 'nextNode',nextNode,'act arrival time',actArrivalTime
			paths[i][j+1] = [nextNode, actArrivalTime]
			currNode = nextNode
			currTime = actArrivalTime		

	minCost = sys.maxint
	for i in range(0,numAnts):
		#print 'ant',i,'cost',paths[i][-1][1]
		if paths[i][-1][1] < minCost:
			minCost = paths[i][-1][1]

	return minCost


def callAlg(mesh,elapse,algFlag):
	algTimeTable = [[0,0]] #start at node 0, and at timestep 0
	for i in range(0, elapse): 
		#print 'i=',i
		#print 'xWeightTimeTable'
		#print m.xWeightTimeTable[i]
		#print 'yWeightTimeTable'
		#print m.yWeightTimeTable[i]
		
		#if at the start node, call alg		
		if i == 0: 	
			#Greedy	and Weighted Greedy	
			if algFlag <= 5:
				greedy(mesh, 0, 0, algTimeTable, algFlag)	

			#Dijkstra
			if algFlag == 6 or algFlag ==7:
				#path = dijkstra(m, 0, 0, 0)
				path = [[0,0]]	
				mesh.add_vertices()
				dijkstra(mesh, 0, 0, algFlag-6)
				lastNode = (mesh.n+1)*(mesh.n+1)-1
				pathTemp = mesh.find_path(0,lastNode)	
				nextPath = pathTemp[1]
				path.append(nextPath)
				#print 'i=0, dijkstra path:'
				#print path	

		else: 
			if algFlag <= 5: 
				algTimeTableCurrentIdx = len(algTimeTable)-1
				currNode = algTimeTable[algTimeTableCurrentIdx][0]
				prevNode = algTimeTable[algTimeTableCurrentIdx-1][0]
				#only the last node appears on the algTimeTable twice
				if currNode == prevNode: 
					break;			
				currNodeStartTime = algTimeTable[algTimeTableCurrentIdx][1]
			
			if algFlag == 6 or algFlag == 7: 
				currNode = path[-1][0]
				if currNode == (mesh.n+1)*(mesh.n+1)-1:
					path[-1][1] = i#update the cost of currNode from estimated cost to actual cost	
					break				
				prevNode = path[-2][0]
				currNodeStartTime = path[-2][1] 
				#print 'currNode:',currNode, 'prevNode:',prevNode,'currNodeStartTime:',currNodeStartTime

			currVertex = mesh.vertex_dict[currNode]
			currX = currNode%(mesh.n+1)
			currY = currNode/(mesh.n+1)			
			prevVertex = mesh.vertex_dict[prevNode]
			actDistSoFar = 0
			for j in range(currNodeStartTime, i): 	
				#go right
				if prevNode == currNode - 1:	
					#actual distance is the sum of (1 time step * speed of previous time step (1/est travel time))
					actDistSoFar = actDistSoFar + 1*1.0/m.xWeightTimeTable[j][currY][currX-1]

				#go down 
				if prevNode == currNode - (mesh.n+1):
					actDistSoFar = actDistSoFar + 1*1.0/m.yWeightTimeTable[j][currX][currY-1]

				#print 'i=',i,',j=',j,'currNode=',currNode,'prevNode=',prevNode, 'actDistSoFar=',actDistSoFar

				#For simplicity, assume that if a vehicle arrives at a node earlier than projected time step, it 
				#just wait there until the next time step. This is to ensure that the total travel time is an integer
				#For example, prevNode = 0, currNode = 1
				#At time 0, est travel time = 3, so speed = 1/3, act dist = 0 
				#At time 1 (i=1, j = 0 ), est travel time = 2, so speed = 1/2, act dist = 0 + 1*1/3
				#At time 2 (i=2, j = 0, 1), est travel time = 1, speed = 1, act dist = 0 + 1*1/3 + 1*1/2
				#At time 3 (i=3, j = 0, 1, 2), est travel time doesn't matter, because act dist = 0 + 1*1/3 + 1*1/2 + 1*1 > 1. 
				#In fact, the vehicle arrived at 2 + 1/6, but waited there until time 3
				#So when i = 3, the algorithm is called again
				if actDistSoFar >= 1.0:
					if prevNode == currNode - 1:
						prevVertex.xActTime = prevVertex.xActTime + (i- currNodeStartTime) 					
					if prevNode == currNode - (mesh.n+1):
						prevVertex.yActTime = prevVertex.yActTime + (i- currNodeStartTime)					

					#greedy and weighted
					if algFlag <= 5:
						greedy(mesh, currNode, i, algTimeTable, algFlag)
					#Dijkstra
					if algFlag == 6 or algFlag == 7: 
						path[-1][1] = i#update the cost of currNode from estimated cost to actual cost					
						#mesh.add_vertices()
						mesh.reset_vertices_ranges(currNode)						
						dijkstra(mesh, currNode, i, algFlag-6)
						lastNode = (mesh.n+1)*(mesh.n+1)-1
						pathTemp = mesh.find_path(currNode,lastNode)	
						#print 'pathTemp'
						#print pathTemp	
						#print 'path'
						#print path									
						if len(pathTemp) > 1:
							nextPath = pathTemp[1]
							path.append(nextPath)
							#print 'pathTemp[1]:',pathTemp[1]									

	#convert algTimeTable to path
	#print algTimeTable
	if algFlag <= 5: 	
		path=[[0,0]]
		for i in range(1, len(algTimeTable)-1):
			node = algTimeTable[i][0]
			nodeArrivalTime = algTimeTable[i+1][1]
			path.append([node,nodeArrivalTime])
	
	return path 

def dijkstra(mesh, startNode, time, algFlag):
	startVertex = mesh.vertex_dict[startNode]
	#startVertex.dijkstraDistance = 0
	startVertex.dijkstraDistance = time
	startX = startNode%(mesh.n+1)
	startY = startNode/(mesh.n+1)
	unvisited_queue = []
	for x in range(startX, mesh.n+1):
		for y in range(startY, mesh.n+1):
			node = y*(mesh.n+1)+x
			vertex = mesh.vertex_dict[node]
			unvisited_queue.append([vertex.dijkstraDistance,node]) 

	heapq.heapify(unvisited_queue)

	while len(unvisited_queue):
		#pop the node with smallest dijkstra distance
		minNode = heapq.heappop(unvisited_queue)
		currNode = minNode[1]
		currX = currNode%(mesh.n+1)
		currY = currNode/(mesh.n+1)
		currVertex = mesh.vertex_dict[currNode]
		currVertex.visited = True

		if currX == mesh.n and currY == mesh.n: 
			return

		else: 
			if currX == mesh.n or currY == mesh.n: 			
				if currY < mesh.n: 
						nextNode = currNode+(mesh.n+1)
						nextVertex = mesh.vertex_dict[nextNode]
						if nextVertex.visited:
							continue 
						if algFlag == 0: 
							new_dist = currVertex.dijkstraDistance + mesh.yWeightTimeTable[time][currX][currY]						
						#difficult to make dijkstra randomized
						#with prediction
						if algFlag == 1: 
							j = calculate_PredictArrivalTime(mesh, time, currX, currY, 0)
							new_dist = currVertex.dijkstraDistance + j					

				if currX < mesh.n: 
						nextNode = currNode+1
						nextVertex = mesh.vertex_dict[nextNode]
						if nextVertex.visited:
							continue 
						if algFlag == 0: 
							new_dist = currVertex.dijkstraDistance + mesh.xWeightTimeTable[time][currY][currX]
						#with prediction
						if algFlag == 1: 
							j = calculate_PredictArrivalTime(mesh, time, currX, currY, 1)
							new_dist = currVertex.dijkstraDistance + j							
		
				if new_dist < nextVertex.dijkstraDistance:
					nextVertex.dijkstraDistance = new_dist 
					nextVertex.previous = currNode
					currVertex.next = nextNode
			else: 
				# 2 possible nextNodes
				for nextNode in range(currNode+1, currNode+(mesh.n+1)+1, mesh.n): 
					nextVertex = mesh.vertex_dict[nextNode]
					if nextVertex.visited:
						continue 
					if nextNode == currNode+1:
						if algFlag == 0:
							new_dist = currVertex.dijkstraDistance + mesh.xWeightTimeTable[time][currY][currX]
						#with prediction
						if algFlag == 1: 
							j = calculate_PredictArrivalTime(mesh, time, currX, currY, 1)
							new_dist = currVertex.dijkstraDistance + j	

					if nextNode == currNode+(mesh.n+1):
						if algFlag == 0:
							new_dist = currVertex.dijkstraDistance + mesh.yWeightTimeTable[time][currX][currY]
						#with prediction
						if algFlag == 1: 
							j = calculate_PredictArrivalTime(mesh, time, currX, currY, 0)
							new_dist = currVertex.dijkstraDistance + j								
					
					if new_dist < nextVertex.dijkstraDistance:
						nextVertex.dijkstraDistance = new_dist 
						nextVertex.previous = currNode
						currVertex.next = nextNode 
	
			#rebuild heap
			#1. pop every time
			while len(unvisited_queue): 
				heapq.heappop(unvisited_queue)
			#2. put all vertices not visited into the queue
			unvisited_queue = []
			for x in range(startX, mesh.n+1):
				for y in range(startY, mesh.n+1):
					node = y*(mesh.n+1)+x
					vertex = mesh.vertex_dict[node]
					if not vertex.visited: 
						unvisited_queue.append([vertex.dijkstraDistance,node]) 
			heapq.heapify(unvisited_queue)

def print_mesh_time_table(mesh, elapse):
	for i in range(0,elapse):
		print 'i=',i
		print 'xWeight:'
		print mesh.xWeightTimeTable[i]
		print 'yWeight:'
		print mesh.yWeightTimeTable[i]	

if __name__ == '__main__':

	for scale in range (10,160,10): 
		n = scale #n*n mesh, assume the length on each edge is 1. has (n+1)^2 nodes  
		mu = 5 #largest travel time on an edge 
		m = mesh(n)
		#worst case arrival time
		elapse = 2*n*mu		
		m.create_time_table(elapse)
		#print_mesh_time_table(m, elapse)
		#for some reason reset vertices does not work 
		#m.reset_vertices()

		for iter in range(0,3): 
			for i in range(0,6):
			#for i in range(7,8):
				m.add_vertices()
				startTime = time.time()
				path = callAlg(m,elapse,i)
				exeTime = time.time()-startTime
				#if i==0:
					#print 'Greedy'
				#if i==1: 
					#print 'Greedy with randomization'
				#if i==2:
					#print 'Greedy with advice'
				#if i==3:
					#print 'Weighted Greedy'
				#if i==4:
					#print 'Weighted Greedy with randomization'
				#if i==5:
					#print 'Weighted Greedy with advice'
				#if i==6:
					#print 'Dijkstra'
				#if i==7:
					#print 'Dijkstra with advice'
				#print 'Executime time:', exeTime
				#print path
				print 'iter=',iter,'n=',n,'alg=',i,',cost=',path[-1][1],',exeTime=',exeTime
			#m.add_vertices()
			#startTime = time.time()
			#acoCost = aco(m,1) #limit number of ants to 32. Results not good, exe time not much different from unlimited version 
			#exeTime = time.time()-startTime
			#print 'iter=',iter,'n=',n,'alg=',8,',cost=',acoCost,',exeTime=',exeTime
			#m.add_vertices()
			#startTime = time.time()
			#acoCost = aco(m,0) #does not limit number of ants
			#exeTime = time.time()-startTime
			#print 'iter=',iter,'n=',n,'alg=',9,',cost=',acoCost,',exeTime=',exeTime			

