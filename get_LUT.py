from path_optimizer import PathOptimizer
import numpy as np
import time

from numpy import savez_compressed
from collections import defaultdict


a = PathOptimizer()
degs = np.arange(-np.pi,np.pi,np.deg2rad(1))
import concurrent.futures

def proc(start,end):
    L = np.zeros((end-start,401,361,3))

    for x in range(start,end,1):

        print(x)
        x_f = 0.05*x
        for y in range(-200,201,1):
            
            y_f = 0.05*y
            for t in range(-180,181,1):
                tf = np.deg2rad(t)
                # toc = time.time()
                path,vals,goal_state = a.optimize_spiral(x_f,y_f,tf,[0,0,0])
                # tic = time.time()
                # print(vals)
                L[x-start,y+200,t+180] = np.array(vals)
    return(L,(start,end))
    

vals = [[1,21],[21,41],[41,61],[61,81],[81,101],[101,121],[121,141],[141,161],[161,181],[181,201],[201,221],[221,241],[241,261],[261,281],[281,301],[301,321],[321,341],[341,361],[361,381],[381,401]]


dict_ = defaultdict()
with concurrent.futures.ProcessPoolExecutor() as executor:

    exec_ = [executor.submit(proc,goal_state[0],goal_state[1]) for goal_state in vals]

    for res in concurrent.futures.as_completed(exec_):
    
        L,val = res.result()
        dict_[val] = L
        

keys = sorted(dict_.keys())

print(keys)
final = []

for i in keys:
    final.extend(dict_[i])

final = np.array(final)

savez_compressed("LUT.npz",final)
