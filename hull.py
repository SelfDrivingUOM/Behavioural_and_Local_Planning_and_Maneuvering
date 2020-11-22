import sys
import numpy as np
import matplotlib.pyplot as plt
import time
# Function to know if we have a CCW turn
def RightTurn(p1, p2, p3):
	if (p3[1]-p1[1])*(p2[0]-p1[0]) >= (p2[1]-p1[1])*(p3[0]-p1[0]):
		return False
	return True
	
# Main algorithm:
def GrahamScan(P):
	P.sort()			# Sort the set of points
	L_upper = [P[0], P[1]]		# Initialize upper part
	# Compute the upper part of the hull
	for i in range(2,len(P)):
		L_upper.append(P[i])
		while len(L_upper) > 2 and not RightTurn(L_upper[-1],L_upper[-2],L_upper[-3]):
			del L_upper[-2]
	L_lower = [P[-1], P[-2]]	# Initialize the lower part
	# Compute the lower part of the hull
	for i in range(len(P)-3,-1,-1):
		L_lower.append(P[i])
		while len(L_lower) > 2 and not RightTurn(L_lower[-1],L_lower[-2],L_lower[-3]):
			del L_lower[-2]
	del L_lower[0]
	del L_lower[-1]
	L = L_upper + L_lower		# Build the full hull
	return np.array(L)

# toc = time.time()

# L = GrahamScan([(0,0),(1,0),(5,0),(1,1),(5,5),(4,3),(0,3),(1,3),(2,2),(2,1),(3,1),(3,2),(4,6)])

# print(L,time.time()-toc)