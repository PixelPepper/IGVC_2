# rclpy (ROS 2のpythonクライアント)の機能を使えるようにします。
import rclpy
# rclpy (ROS 2のpythonクライアント)の機能のうちNodeを簡単に使えるようにします。こう書いていない場合、Nodeではなくrclpy.node.Nodeと書く必要があります。
from rclpy.node import Node
# ROS 2の文字列型を使えるようにimport
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import nav_msgs.msg as nav_msgs
import numpy as np
import math
#import open3d as o3d
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import os
import time
import transforms3d
from geometry_msgs.msg import Quaternion

#map save
#ros2 run nav2_map_server map_saver_cli -t /reflect_map_global -f ~/ros2_ws/src/map/test_map --ros-args -p map_subscribe_transient_local:=true -r __ns:=/namespace
#ros2 run nav2_map_server map_saver_cli -t /reflect_map_global --occ 0.10 --free 0.05 -f ~/ros2_ws/src/map/test_map2 --ros-args -p map_subscribe_transient_local:=true -r __ns:=/namespace
#--occ:  occupied_thresh  この閾値よりも大きい占有確率を持つピクセルは、完全に占有されていると見なされます。
#--free: free_thresh	  占有確率がこの閾値未満のピクセルは、完全に占有されていないと見なされます。

# C++と同じく、Node型を継承します。
class OdomFast(Node):
    # コンストラクタです、Mid360Subscriberクラスのインスタンスを作成する際に呼び出されます。
    def __init__(self):
        # 継承元のクラスを初期化します。（https://www.python-izm.com/advanced/class_extend/）今回の場合継承するクラスはNodeになります。
        super().__init__('livox_node')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth = 10
        )
        
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth = 10
        )
        
        # Subscriptionを作成。CustomMsg型,'/livox/lidar'という名前のtopicをsubscribe。
        self.subscription = self.create_subscription(nav_msgs.Odometry,'/Odometry', self.get_fastlio_odom, qos_profile_sub)
        self.subscription  # 警告を回避するために設置されているだけです。削除しても挙動はかわりません。
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Publisherを作成
        self.reflect_map_obs_publisher = self.create_publisher(nav_msgs.Odometry, 'odom_fast', qos_profile)        
        
        #パラメータ
        
        #mid360 positon init
        self.position_x = 0.0 #[m]
        self.position_y = 0.0 #[m]
        self.position_z = 0.0 #[m]
        self.theta_x = 0.0 #[m]
        self.theta_y = 0.0 #[m]
        self.theta_z = 0.0 #[m]
        
        
        #buff init
        self.imu_sec_buff = 0
        self.imu_nanosec_buff = 0
        
    
    def timer_callback(self):
        #
        pass
                
    def get_fastlio_odom(self, msg):
        #print(f"!get fastlio odom!")
        t_stamp_imu_sec = msg.header.stamp.sec
        #print(f"t_stamp_odom_sec ={t_stamp_imu_sec}")
        t_stamp_imu_nanosec = msg.header.stamp.nanosec
        #print(f"t_stamp_odom_nanosec ={t_stamp_imu_nanosec}")
        
        imu_time = (t_stamp_imu_sec + t_stamp_imu_nanosec/1000000000)
        imu_buff_time = (self.imu_sec_buff + self.imu_nanosec_buff/1000000000)
        
        if self.imu_sec_buff == 0:
            imu_dt = 0
        else :
            imu_dt = imu_time - imu_buff_time
        #print(f"imu_time ={imu_time}")
        #print(f"imu_buff_time ={imu_buff_time}")
        #print(f"imu_dt ={imu_dt}")
        
        #buff update
        self.imu_sec_buff = t_stamp_imu_sec
        self.imu_nanosec_buff = t_stamp_imu_nanosec
        
        
        self.position_x = msg.pose.pose.position.x
        self.position_y = -msg.pose.pose.position.y
        self.position_z = -msg.pose.pose.position.z
        #print(f"flio_pos_x ={self.position_x}")
        #print(f"flio_pos_y ={self.position_y}")
        #print(f"flio_pos_z ={self.position_z}")
        
        flio_q_x = msg.pose.pose.orientation.x
        flio_q_y = msg.pose.pose.orientation.y
        flio_q_z = msg.pose.pose.orientation.z
        flio_q_w = msg.pose.pose.orientation.w
        
        roll, pitch, yaw = quaternion_to_euler(flio_q_x, flio_q_y, flio_q_z, flio_q_w)
        
        self.theta_x = 0 #roll /math.pi*180
        self.theta_y = 0 #pitch /math.pi*180
        self.theta_z = -yaw /math.pi*180
        
        #print(f"roll ={self.theta_x}")
        #print(f"pitch ={self.theta_y}")
        #print(f"yaw ={self.theta_z}")
        
        odom_msg = odometry_msg(self.position_x, self.position_y, self.position_z, self.theta_x, self.theta_y, self.theta_z, msg.header.stamp)        
        
        #publish
        self.reflect_map_obs_publisher.publish(odom_msg)

def quaternion_to_euler(x, y, z, w):
    # クォータニオンから回転行列を計算
    rot_matrix = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
    ])

    # 回転行列からオイラー角を抽出
    roll = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
    pitch = np.arctan2(-rot_matrix[2, 0], np.sqrt(rot_matrix[2, 1]**2 + rot_matrix[2, 2]**2))
    yaw = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
    return roll, pitch, yaw

def odometry_msg(pos_x, pos_y, pos_z, theta_x, theta_y, theta_z, stamp):
    odom_msg = nav_msgs.Odometry()
    odom_msg.header.stamp = stamp
    odom_msg.header.frame_id = 'odom'
    
    # 位置情報を設定
    odom_msg.pose.pose.position.x = pos_x 
    odom_msg.pose.pose.position.y = pos_y
    odom_msg.pose.pose.position.z = pos_z
    
    # YawをQuaternionに変換
    roll = theta_x /180*math.pi
    pitch = theta_y /180*math.pi
    yaw = theta_z /180*math.pi
    quat = transforms3d.euler.euler2quat(roll, pitch, yaw)
    odom_msg.pose.pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
    
    return odom_msg

# mainという名前の関数です。C++のmain関数とは異なり、これは処理の開始地点ではありません。
def main(args=None):
    # rclpyの初期化処理です。ノードを立ち上げる前に実装する必要があります。
    rclpy.init(args=args)
    # Mid360Subscriberクラスのインスタンスを作成
    odom_fast = OdomFast()
    # spin処理を実行、spinをしていないとROS 2のノードはデータを入出力することが出来ません。
    rclpy.spin(odom_fast)
    '''
    try:
        while rclpy.ok():
            rclpy.spin_once(mid360_subscriber)
            time.sleep(0.1)  # 0.1秒ごとにspin_onceを呼び出す
    except KeyboardInterrupt:
        pass
    '''
    # 明示的にノードの終了処理を行います。
    odom_fast.destroy_node()
    # rclpyの終了処理、これがないと適切にノードが破棄されないため様々な不具合が起こります。
    rclpy.shutdown()

# 本スクリプト(publish.py)の処理の開始地点です。
if __name__ == '__main__':
    # 関数`main`を実行する。
    main()
