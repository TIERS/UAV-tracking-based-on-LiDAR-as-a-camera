from ouster import client
from contextlib import closing
import numpy as np
import cv2
import yolov5
import open3d as o3d
from utils import *
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped


### load pcap file
metadata_path = './result.json'
with open(metadata_path, 'r') as f:
    metadata = client.SensorInfo(f.read())
    from ouster import pcap
pcap_path = './result.pcap' 
pcap_file = pcap.Pcap(pcap_path, metadata)


##           load yolov5 model 
model = yolov5.load('final.pt')
model.conf = 0.3
model.iou = 0.25
model.agnostic = False
model.multi_label = False
model.max_det = 1000

##           create ros node 
pub = rospy.Publisher('drone', PoseStamped, queue_size=1)
rospy.init_node('pose_publisher', anonymous=True)
rate = rospy.Rate(2) # Hz
goalMsg = PoseStamped()
goalMsg.header.frame_id = "/base_link"

##           initilize KF
dt = 2.0/1
F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H = np.array([1, 0, 0]).reshape(1, 3)
Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R = np.array([0.5]).reshape(1, 1)
measure_x = 0
measure_y = 0
measure_z = 0
predictions_x = 0
predictions_y = 0
predictions_z = 0
kf1 = KalmanFilter(F = F, H = H, Q = Q, R = R)
kf2 = KalmanFilter(F = F, H = H, Q = Q, R = R)
kf3 = KalmanFilter(F = F, H = H, Q = Q, R = R)


##           other para
track_array = []
point_color = (0, 255, 0) # BGR
thickness = 2 
lineType = 4
counter = 0
temp_pc = o3d.geometry.PointCloud()
temp_window = []
ROI_NUM = 0



with closing(client.Scans(pcap_file)) as scans:
    for scan in scans:
        counter += 1
        goalMsg = PoseStamped()
        ref_field = scan.field(client.ChanField.REFLECTIVITY)
        xyzlut = client.XYZLut(metadata)
        xyz_destaggered = client.destagger(metadata, xyzlut(scan))
        ref_val = client.destagger(pcap_file.metadata, ref_field)
        ref_val = np.divide(ref_val, np.amax(ref_val), dtype=np.float32)
        ref_val *= 255
        ref_img = ref_val.astype(np.uint8)
        ref_img = image_ehancement(ref_img)
        ref_img_resise = cv2.resize(ref_img,(1024,300),interpolation = cv2.INTER_AREA)
        result = model(ref_img_resise)
        predictions_resize = result.pred[0]
        boxes_re = predictions_resize[:, :4] 
        boxes_re = boxes_re.tolist()
        scores_re = predictions_resize[:, 4]
        categories_re = predictions_resize[:, 5].tolist()
        print('----------------------------------------------------------------------------------')
        print("counter:",counter)
        if boxes_re:
            box_size = (boxes_re[0][2]-boxes_re[0][0])*(boxes_re[0][3]-boxes_re[0][1])
        else :
            box_size = 10
        
        
        if len(categories_re) == 0 and box_size <= 810:
            ROI_NEW = get_roi(temp_window,xyz_destaggered.reshape((-1, 3)),1.5)

            if len(ROI_NEW) > 10000:
                track_array.append(np.asarray([predictions_x,predictions_y,predictions_z]))
                goalMsg.header.stamp = rospy.Time.now()
                goalMsg.pose.position.z = predictions_z
                goalMsg.pose.position.x = predictions_x
                goalMsg.pose.position.y = predictions_y
                goalMsg.pose.orientation.w = 1.0
                pub.publish(goalMsg)
                continue
            else:
                ground = remove_ground(ROI_NEW)
                drone,flag = get_roi_drone(ground,temp_pc,0.3)  
                if flag == 0:
                    track_array.append(np.asarray([predictions_x,predictions_y,predictions_z]))

                    goalMsg.header.stamp = rospy.Time.now()
                    goalMsg.pose.position.z = predictions_z
                    goalMsg.pose.position.x = predictions_x
                    goalMsg.pose.position.y = predictions_y
                    goalMsg.pose.orientation.w = 1.0
                    pub.publish(goalMsg)
                    continue
                else:
                    temp_window = drone.get_center()
                    track_array.append(np.asarray([temp_window[0],temp_window[1],temp_window[2]]))
                    temp_pc = drone
                    measure_x = temp_window[0]
                    measure_y = temp_window[1]
                    measure_z = temp_window[2]
                    predictions_x= np.dot(H,  kf1.predict())[0][0]
                    predictions_y = np.dot(H,  kf2.predict())[0][0]
                    predictions_z = np.dot(H,  kf3.predict())[0][0]
                    kf1.update(measure_x)
                    kf2.update(measure_y)
                    kf3.update(measure_z)
                    goalMsg.header.stamp = rospy.Time.now()
                    goalMsg.pose.position.z = temp_window[2]
                    goalMsg.pose.position.x = temp_window[0]
                    goalMsg.pose.position.y = temp_window[1]
                    goalMsg.pose.orientation.w = 1.0
                    pub.publish(goalMsg)
                

                
        else:
            
            if (0.0 in categories_re) and  box_size > 810: 
                index = get_person_index(categories_re)[0]
                x1,y1,x2,y2 = get_boxrs_ori(boxes_re,index)
                x1_temp,y1_temp,x2_temp,y2_temp = x1,y1,x2,y2
                cv2.rectangle(ref_img_resise, (x1,y2),(x2,y1), point_color, thickness, lineType)
                X1,Y1 = resize_pos(x1,y1,[1024,300],[1024,128])
                X2,Y2 = resize_pos(x2,y2,[1024,300],[1024,128])   

                ROI = xyz_destaggered[Y1:Y2:, X1:X2, :].reshape((-1, 3))
                ROI_NUM = len(ROI)
                if len(temp_window)==0 :
                    ground = remove_ground(ROI)
                    drone = get_init_position(ground,eps=0.2,min_points=20)
                    temp_window = drone.get_center()
                    temp_pc = drone
                    goalMsg.header.stamp = rospy.Time.now()
                    goalMsg.pose.position.z = temp_window[2]
                    goalMsg.pose.position.x = temp_window[0]
                    goalMsg.pose.position.y = temp_window[1]
                    goalMsg.pose.orientation.w = 1.0
                    pub.publish(goalMsg)
                else:
                    ground = remove_ground(ROI)
                    drone,flag = get_drone(ground,temp_pc,eps=0.2,min_points=20)
                    temp_window = drone.get_center()
                    track_array.append(np.asarray([temp_window[0],temp_window[1],temp_window[2]]))
                    temp_pc = drone
                    measure_x = temp_window[0]
                    measure_y = temp_window[1]
                    measure_z = temp_window[2]
                    predictions_x= np.dot(H,  kf1.predict())[0][0]
                    predictions_y = np.dot(H,  kf2.predict())[0][0]
                    predictions_z = np.dot(H,  kf3.predict())[0][0]
                    kf1.update(measure_x)
                    kf2.update(measure_y)
                    kf3.update(measure_z)
                    goalMsg.header.stamp = rospy.Time.now()
                    goalMsg.pose.position.z = temp_window[2]
                    goalMsg.pose.position.x = temp_window[0]
                    goalMsg.pose.position.y = temp_window[1]
                    goalMsg.pose.orientation.w = 1.0
                    pub.publish(goalMsg)

                cv2.imshow('frame', ref_img_resise)
                

                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break        
cv2.destroyAllWindows()
np.save('traj.npy',track_array)
