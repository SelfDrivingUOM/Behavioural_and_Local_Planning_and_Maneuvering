
import numpy as np

def get_line(point_list):
    # print(point_list)
    n_ = point_list.shape[0]
    line_info = np.zeros((n_-1,2))
    

    grad_vals = point_list - np.append(point_list[:-1,:])
    # print(grad_vals)
    crit = (grad_vals[:,0] ==0)
    crit_ = np.logical_not(crit)
    grad_vals_ = grad_vals[crit_][:,1] /grad_vals[crit_][:,0]
    c_vals = point_list[:-1,1][crit_] - grad_vals_*point_list[:-1,0][crit_]
    
    # print(grad_vals_,c_vals)
    grad_vals_ = grad_vals_.reshape((grad_vals_.shape[0],1))
    c_vals = c_vals.reshape((grad_vals_.shape[0],1))
    line_info[crit_] = np.hstack((grad_vals_,c_vals))
    line_info[crit] = np.array([None,point_list[:-1][crit][0,0]])
    

    ##### put this bit into numpy later
    temp_ = point_list[0] - point_list[-1]
    if(temp_[0] ==0):
        line_info = np.append(line_info,[[None,temp_[0]]])
    else:
        m_ = temp_[1]/temp_[0]

        line_info = np.append(line_info,[[m_,point_list[0][1]-m_*point_list[0][0]]])

    # line_info  

    return line_info

def solve_lines(lines):
    intersections = np.empty((0,2))

    for i in range(-1,lines.shape[0]-1,1):

        if(np.isnan(lines[i+1][0])): 
            x_ = lines[i+1][1]
            y_ = lines[i][0]*x_+ lines[i][1]
        
        elif(np.isnan(lines[i][0])):
            x_ = lines[i][1]
            y_ = lines[i+1][0]*x_+ lines[i+1][1]
            
        else:
            x_ = (lines[i+1][1]-lines[i][1])/(lines[i][0]-lines[i+1][0])
            y_ = lines[i][0]*x_ + lines[i][1]
        intersections = np.append(intersections,[[x_,y_]],axis =0)

    return intersections


# print(solve_lines(get_line(np.array([(1,1),(2,2),(2,3),(5,3)]))))
def within_polygon(points,location):
    within = False
    sign = None
    for i in range(-1,points.shape[0]-1,1):
        cross = np.cross(points[i+1]-points[i],location-points[i])
        # print(cross)
        if sign == None:
            if(cross>=0):
                sign = 1
            else:
                sign = -1

        else:
            if(sign*cross>=0):
                continue
            else:
                break
    else:
        within = True

    return(within)


print(within_polygon(np.array([[-141,103],[-141,76],[-109,75]]),np.array([-130,105])))


# for i in range(5):
#     print(i)
#     if(i==None):
#         break
# else:
#     print("LOL")