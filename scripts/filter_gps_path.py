import rospy
import copy
import csv
import math

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

with open('gps_path.csv', 'rb') as f:
    reader = csv.reader(f)
    gps_data = list(reader)
    gps_data = [[float(y) for y in x] for x in gps_data]

rospy.init_node('filtered_path_node')
path_pub = rospy.Publisher('/filtered_path', Path, queue_size=10)

path = Path()
path.header.seq = 111
path.header.stamp = rospy.Time.now()
path.header.frame_id = 'gps_path'

before_gps = gps_data[0]
filtered_gps = []
first = True
for gps in gps_data:
    d_x = gps[0]-before_gps[0]
    d_y = gps[1]-before_gps[1]
    if first:
        before_theta = math.atan2(d_y, d_x)
        first = False
    theta = math.atan2(d_y, d_x)
    if theta > 180:
        theta = theta - 360
    print(math.degrees(theta))
    if abs(d_x) < 2 and abs(d_y) < 2: 
        if abs(math.degrees(theta - before_theta)) > 60 and (abs(d_x) > 0.4 or abs(d_y) > 0.4):
        #if abs(math.degrees(theta)) < 130 and abs(d_x) > 0.3 and abs(d_y) > 0.3:
            continue
        filtered_gps.append(gps)
        before_gps = gps
        before_theta = theta

        pose_stamped = PoseStamped()
        pose_stamped.header = path.header
        pose_stamped.pose.position.x = gps[0]
        pose_stamped.pose.position.y = gps[1]
        pose_stamped.pose.position.z = 0

        pose_stamped.pose.orientation.x = 0
        pose_stamped.pose.orientation.y = 0
        pose_stamped.pose.orientation.z = 0
        pose_stamped.pose.orientation.w = 1

        a = copy.deepcopy(pose_stamped)
        path.poses.append(a)

def LPF(x, pre_x, pre_y, tau, ts):
    return (tau * pre_y + ts * x )/(tau + ts) 

LPF_gps = []
before_gps = filtered_gps[0]
before_result = filtered_gps[0]
for gps in filtered_gps:
    result_x = LPF(gps[0], before_gps[0], before_result[0], 4.0, 1.0)
    result_y = LPF(gps[1], before_gps[1], before_result[1], 4.0, 1.0)

    LPF_gps.append([result_x, result_y])
    before_result = [result_x, result_y]
    before_gps = gps

    pose_stamped = PoseStamped()
    pose_stamped.header = path.header
    pose_stamped.pose.position.x = result_x
    pose_stamped.pose.position.y = result_y
    pose_stamped.pose.position.z = 0

    pose_stamped.pose.orientation.x = 0
    pose_stamped.pose.orientation.y = 0
    pose_stamped.pose.orientation.z = 0
    pose_stamped.pose.orientation.w = 1

    #a = copy.deepcopy(pose_stamped)
    #path.poses.append(a)

r = rospy.Rate(100) # 10hz
while not rospy.is_shutdown():
    path_pub.publish(path)
