import numpy as np
import open3d as o3d
import copy



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


    # 使用 NumPy 的布尔索引筛选出符合条件的向量
    mask = (data[:, 0] >= x_min) & (data[:, 0] <= x_max) & \
    (data[:, 1] >= y_min) & (data[:, 1] <= y_max) & \
    (data[:, 2] >= z_min) & (data[:, 2] <= z_max)

    return data[mask]