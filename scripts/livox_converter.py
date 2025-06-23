#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
import std_msgs.msg
import struct

# Global publisher
pub = None

def callback(custom_msg):
    global pub
    pc2_msg = PointCloud2()
    
    # Copy header
    pc2_msg.header = custom_msg.header

    # Set dimensions
    pc2_msg.height = 1
    pc2_msg.width = custom_msg.point_num

    # Define fields. Livox points have x, y, z, and reflectivity.
    # We will cast the uint8 reflectivity to a float32 for the intensity field.
    pc2_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
    ]

    pc2_msg.is_bigendian = False
    pc2_msg.point_step = 16  # 4 fields * 4 bytes/field
    pc2_msg.row_step = pc2_msg.point_step * custom_msg.point_num
    pc2_msg.is_dense = True

    # Pack point data
    point_data = []
    for point in custom_msg.points:
        point_data.append(struct.pack('ffff', point.x, point.y, point.z, float(point.reflectivity)))
    
    pc2_msg.data = b"".join(point_data)

    pub.publish(pc2_msg)

def start_node():
    global pub
    rospy.init_node('livox_custom_to_pc2_converter', anonymous=True)
    
    # Publisher for the new PointCloud2 topic
    pub = rospy.Publisher('/livox/lidar_pc2', PointCloud2, queue_size=10)
    
    # Subscriber for the original CustomMsg topic
    rospy.Subscriber('/livox/lidar', CustomMsg, callback, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass