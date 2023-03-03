import numpy as np
import open3d as o3d
import copy
import cv2




class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):
        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")
        self.n = F.shape[1]
        self.m = H.shape[1]
        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0
    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x
    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P),
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)



def get_init_position(pnt,eps,min_points):
    """ extract drone pointcloud

    Args:
        pnt: N x 3 point clouds
        eps: the distance to neighbors in a cluster
        min_points: the minimum number of points

    Returns:
        [ndarray]: N x 3 point clouds
    """
    pointcloud = copy.deepcopy(pnt)
    
    labels = np.array(pointcloud.cluster_dbscan(eps, min_points, print_progress=True))
    dis = []
    for label_i in np.unique(labels):
        person_label = np.array(np.where(labels==label_i))
        person_pnt = pointcloud.select_by_index(person_label[0])
        if (np.linalg.norm(person_pnt.get_center() - np.array([0,0,0]))) == 0:
            dis.append(1000)
        else: 
            dis.append(np.linalg.norm(person_pnt.get_center() - np.array([0,0,0])))

    index = np.where(dis == np.min(dis))
    drone_label = np.array(np.where(labels==index[0][0]-1))
    return pointcloud.select_by_index(drone_label[0])



def get_drone(pnt,target,eps,min_points):
    """ extract drone pointcloud

    Args:
        pnt: N x 3 point clouds
        eps: the distance to neighbors in a cluster
        min_points: the minimum number of points
        target: target drone pcd

    Returns:
        [ndarray]: N x 3 point clouds
    """
    pointcloud = copy.deepcopy(pnt)
    labels = np.array(pointcloud.cluster_dbscan(eps, min_points, print_progress=True))
    dis = []

    for label_i in np.unique(labels):
        try: 
            person_label = np.array(np.where(labels==label_i))
            person_pnt = pointcloud.select_by_index(person_label[0])
            print(person_pnt)
            if (np.linalg.norm(person_pnt.get_center() - np.array([0,0,0]))) == 0:
                dis.append(1000)
            else: 
                dis.append(np.linalg.norm(person_pnt.get_center() - target.get_center()))
        except RuntimeError:
                dis.append(1000)
                pass
    dis = np.asarray(dis)
    print(dis)
    if np.min(dis) > 0.1:
        flag = 0
    else:
        flag = 1
    index = np.where(dis == np.min(dis))
    print(index)
    drone_label = np.array(np.where(labels==index[0][0]-1))
    if abs(len(target.points)-len(pointcloud.select_by_index(drone_label[0]).points))>100:
        flag = 0
    else:
        flag = 1
    return pointcloud.select_by_index(drone_label[0]),flag


def get_roi_drone(pnt,target,eps,):
    """ extract drone pointcloud

    Args:
        pnt: N x 3 point clouds
        eps: the distance to neighbors in a cluster
        target: target drone pcd

    Returns:
        [ndarray]: N x 3 point clouds
    """
    num = len(np.asarray(target.points))

    if num > 200:
        num = 80
    
    pointcloud = copy.deepcopy(pnt)
    labels = np.array(pointcloud.cluster_dbscan(eps, int(0.625*num), print_progress=True))
    dis = []

    for label_i in np.unique(labels):
        try: 
            person_label = np.array(np.where(labels==label_i))
            person_pnt = pointcloud.select_by_index(person_label[0])
            print(person_pnt)
            if (np.linalg.norm(person_pnt.get_center() - np.array([0,0,0]))) == 0:
                dis.append(1000)
            else: 
                dis.append(abs(np.linalg.norm(person_pnt.get_center() - target.get_center())))
        except RuntimeError:
                dis.append(1000)
                pass
    dis = np.asarray(dis)
    print(dis)
    if np.min(dis) > 0.1:
            flag = 0
    else:
        flag = 1
    index = np.where(dis == np.min(dis))
    print(index)
    if len(np.where(np.unique(labels)==-1)[0]) ==0:
        drone_label = np.array(np.where(labels==index[0][0]))
        if abs(len(target.points)-len(pointcloud.select_by_index(drone_label[0]).points))>15:
            flag = 0
        else:
            flag = 1
    else:
        drone_label = np.array(np.where(labels==index[0][0]-1))
        if (index[0][0]-1) == -1:
            flag = 0
        else:
            flag = 1
            if abs(len(target.points)-len(pointcloud.select_by_index(drone_label[0]).points))>15:
                flag = 0
            else:
                flag = 1
        
    return pointcloud.select_by_index(drone_label[0]),flag





def get_roi(center,data,therhold):
    x_min = center[0] - therhold
    x_max = center[0] + therhold
    y_min = center[1] - therhold
    y_max = center[1] + therhold
    z_min = center[2] - 0.5
    z_max = center[2] + 0.5

    mask = (data[:, 0] >= x_min) & (data[:, 0] <= x_max) & \
    (data[:, 1] >= y_min) & (data[:, 1] <= y_max) & \
    (data[:, 2] >= z_min) & (data[:, 2] <= z_max)

    return data[mask]





def record_track(center,trans_init):
    pcd_center = NumpyToPCD(center.reshape((-1, 3)))
    return np.asarray(pcd_center.transform(trans_init).points)[0]

def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return  pcd_fpfh

def image_ehancement(img):
    fI = img/255.0
    gamma = 0.4
    img = np.power(fI, gamma)
    drc = np.zeros_like(img)
    img_result = cv2.normalize(img, drc, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    
    return img_result


def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    print(source_down, target_down)
    distance_threshold = voxel_size * 0.5
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))

    return result

def compare_pcd(source,target):
    source_tem = copy.deepcopy(source)
    target_tem = copy.deepcopy(target)
    source_fpfh = preprocess_point_cloud(source_tem,0.03)
    target_fpfh = preprocess_point_cloud(target_tem,0.03)
    result_ransac = execute_fast_global_registration(source_tem, target_tem,
                                            source_fpfh, target_fpfh,
                                            0.03)
    return result_ransac.fitness,result_ransac.inlier_rmse,result_ransac.transformation

def get_boxrs_ori(box,index):
    x1 = int(box[index][0])
    y1 = int(box[index][1])
    x2 = int(box[index][2])
    y2 = int(box[index][3])
    return x1,y1,x2,y2




def locate_2d_area(x,y,z):
    """ find the 2d area of the target

    Args:
        (x,y,z)

    Returns:
        location of rectangle in 2d image
    """
    a,b,c = cart2sph(x,y,z)
    img_x= int(180*a/(0.18*np.pi)+1160)
    img_y= int(180*b/(0.7*np.pi)+64)
    print("img_loc",img_x,img_y)
                                              # //////////////////////////////////
    X1,Y1 = resize_pos(img_x-150,img_y-20,[2000,128],[1024,300])
    X2,Y2 = resize_pos(img_x+150,img_y+20,[2000,128],[1024,300])
    print(X1,Y1,X2,Y2)
    return X1,300-Y1,X2,300-Y2
    # return X1,Y1,X2,Y2

def resize_pos(x1,y1,src_size,tar_size):
 
    w1=src_size[0]
    h1=src_size[1]
    w2=tar_size[0]
    h2=tar_size[1]
    y2=(h2/h1)*y1
    x2=(w2/w1)*x1
    return int(x2),int(y2)

def cart2sph(x, y, z):
    hxy = np.hypot(x, y)
    r = np.hypot(hxy, z)
    el = np.arctan2(z, hxy) #+ (0.226944444*np.pi)
    az = np.arctan2(y, x) #+ (0.226944444*np.pi)
    return az, el, r 

def remove_ground(data):
    """ remove ground plane

    Args:
        xyz (ndarray): 

    Returns:
        [ndarray]: N x 3 point clouds
    """
    pointcloud = NumpyToPCD(data)
    z_value = data[:,2]
    print(len(z_value))
    if np.min(z_value)<0:
        label = np.array(np.where(z_value>np.sort(z_value)[10]+0.1))
        ground = pointcloud.select_by_index(label[0])
        print('ground!')
    else:
        ground = NumpyToPCD(data)
        print('no ground!')
    pc_np = PCDToNumpy(ground)
    x_value = pc_np[:,0]
    label = np.array(np.where(x_value != 0))
    ground = ground.select_by_index(label[0])
    return ground

def next_area(x1,y1,x2,y2,k):
    X1 = x1*(1+k)/2.0 +x2*(1-k)/2.0
    Y1 = y1*(1-k)/2.0 +y2*(1+k)/2.0
    X2 = x1*(1-k)/2.0 +x2*(1+k)/2.0
    Y2 = y1*(1+k)/2.0 +y2*(1-k)/2.0
    if Y2 <0 :
        Y2 = 0
    if Y1 >300:
        Y1 = 300
    
    return int(X1),int(Y1),int(X2),int(Y2)



def get_person_index(list_1):
    index_person=[]
    for i in range(len(list_1)):
        if list_1[i] == 0.0:
            index_person.append(i)
        else:
            pass
    return index_person



def ReadPlyPoint(fname):
    """ read point from ply

    Args:
        fname (str): path to ply file

    Returns:
        [ndarray]: N x 3 point clouds
    """

    pcd = o3d.io.read_point_cloud(fname)

    return PCDToNumpy(pcd)


def remove_zero(data):
    """ remove ground plane

    Args:
        xyz (ndarray): 

    Returns:
        [ndarray]: N x 3 point clouds
    """
    pointcloud = NumpyToPCD(data)
    x_value = data[:,0]
    label = np.array(np.where(x_value != 0))
    print(label)
    ground = pointcloud.select_by_index(label[0])
    return ground

def extract_person(data,eps,min_points):
    """ extract pointcloud of object person from source pointcloud

    Args:
        xyz (ndarray): 

    Returns:
        [ndarray]: open3d pointcloud
    """
    data = NumpyToPCD(data)
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(data.cluster_dbscan(eps, min_points, print_progress=True))
    max_label = labels.max()    # 获取聚类标签的最大值 [-1,0,1,2,...,max_label]，label = -1 为噪声，因此总聚类个数为 max_label + 1
    person_label = np.array(np.where(labels==max_label))
    person_pnt = data.select_by_index(person_label[0])

    return person_pnt

def NumpyToPCD(xyz):
    """ convert numpy ndarray to open3D point cloud 

    Args:
        xyz (ndarray): 

    Returns:
        [open3d.geometry.PointCloud]: 
    """

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd


def PCDToNumpy(pcd):
    """  convert open3D point cloud to numpy ndarray

    Args:
        pcd (open3d.geometry.PointCloud): 

    Returns:
        [ndarray]: 
    """

    return np.asarray(pcd.points)


def RemoveNan(points):
    """ remove nan value of point clouds

    Args:
        points (ndarray): N x 3 point clouds

    Returns:
        [ndarray]: N x 3 point clouds
    """

    return points[~np.isnan(points[:, 0])]



def DrawResult(points, colors):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])