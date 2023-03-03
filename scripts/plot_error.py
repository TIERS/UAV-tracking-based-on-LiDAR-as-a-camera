import tikzplotlib
import matplotlib.pyplot as plt 
import numpy as np

#### load your data that get from evo jupyter scripts 
data_o_x = np.load('./test_3_ours/data_x.npy')
data_o_y = np.load('./test_3_ours/data_y.npy')
data_o_z = np.load('./test_3_ours/data_z.npy')
ref_o_x = np.load('./test_3_ours/ref_x.npy')
ref_o_y = np.load('./test_3_ours/ref_y.npy')
ref_o_z = np.load('./test_3_ours/ref_z.npy')



diff_o_x = abs(ref_o_x-data_o_x)
diff_o_y = abs(ref_o_y-data_o_y)
diff_0_z = abs(ref_o_z-data_o_z)

##### Plot your Velo_error_error here
 
##### ......................

