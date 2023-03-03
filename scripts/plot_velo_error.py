import tikzplotlib
import matplotlib.pyplot as plt 
import numpy as np




def get_velo_error(data_x,data_y,data_z,ref_x,ref_y,ref_z,rate):
    #rate : frame rate of lidar   For Ouster lidar, rate is 10 hz
    x_arr = []
    y_arr = []
    z_arr = []
    arr = []
    for i in range(len(data_x)-1):
        s_x = abs(rate*(data_x[i+1]-data_x[i]))        
        s_y = abs(rate*(data_y[i+1]-data_y[i]))
        s_z = abs(rate*(data_z[i+1]-data_z[i]))
        
        sr_x = abs(rate*(ref_x[i+1]-ref_x[i]))
        sr_y = abs(rate*(ref_y[i+1]-ref_y[i]))
        sr_z = abs(rate*(ref_z[i+1]-ref_z[i]))
        x_arr.append(abs(sr_x-s_x))
        y_arr.append(abs(sr_y-s_y))
        z_arr.append(abs(sr_z-s_z))
        
        
 

    for i in range(len(x_arr)):
        arr.append(np.linalg.norm(np.asarray([x_arr[i],y_arr[i],z_arr[i]]) - np.asarray([0,0,0])))

    return x_arr,y_arr,z_arr,arr
        

#### load your data that get from evo jupyter scripts  
data_o_x = np.load('./test_3_ours/data_x.npy')
data_o_y = np.load('./test_3_ours/data_y.npy')
data_o_z = np.load('./test_3_ours/data_z.npy')
ref_o_x = np.load('./test_3_ours/ref_x.npy')
ref_o_y = np.load('./test_3_ours/ref_y.npy')
ref_o_z = np.load('./test_3_ours/ref_z.npy')



print(len(ref_o_x),len(data_o_x),np.max(data_o_x))
speed_o_x = []
speed_o_y = []
speed_o_z = []
speed_o = []




speed_o_x,speed_o_y,speed_o_z,speed_o = get_velo_error(data_o_x,data_o_y,data_o_z,ref_o_x,ref_o_y,ref_o_z,10)   # velo error in X, Y, Z

##### Plot your Velo_error_error here
 
##### ......................