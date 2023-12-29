#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
import requests
import time
import json
from roslibpy import Message
import json
import math


def calculate_speed(x, y, z):
    speed = math.sqrt(x**2 + y**2 + z**2)
    return speed

def mode_guided(mode):
    if mode == True:
        return 0
    else: 
        return 1


def quaternion_to_euler(x, y, z, w):
    # Quaternion order: w, x, y, z
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return yaw, pitch, roll



def callback(imu_msg, battery_msg, rel_altitude_msg, position_msg, speed_sub, state_sub):
    data_dict = {
        "takim_numarasi" : 1,
        "IHA_enlem": position_msg.latitude,
        "IHA_boylam": position_msg.longitude,
        "IHA_irtifa": rel_altitude_msg.data,
        "IHA_yonelme": imu_msg.orientation.x,
        "IHA_dikilme": imu_msg.orientation.y,
        "IHA_yatis": imu_msg.orientation.z,
        "IHA_hiz": calculate_speed(speed_sub.twist.linear.x, speed_sub.twist.linear.y, speed_sub.twist.linear.z),
        "IHA_batarya": int(battery_msg.percentage * 100),
        "IHA_otonom": mode_guided(state_sub.guided)
    }
    
    #TODO: kitlenme verilerini ekle
    server_url = rospy.get_param('~server_ip',"http://127.0.0.1:5000/update_data")

    response = requests.post(server_url, json=data_dict)
    #TODO: parse the response

    if response.status_code == 200:
        print(response.status_code)
        print(response.json())
    else:
        print(f"Hata: {response.status_code}")
    
    
    
def synchronize_topics():
    rospy.init_node('sync_node', anonymous=True)

    imu_sub = Subscriber('/mavros/imu/data', Imu)
    battery_sub = Subscriber('/mavros/battery', BatteryState)
    rel_altitude_sub = Subscriber('/mavros/global_position/rel_alt', Float64)
    position_sub = Subscriber('/mavros/global_position/global', NavSatFix)
    speed_sub = Subscriber('/mavros/local_position/velocity_local', TwistStamped)
    state_sub = Subscriber('/mavros/state' , State)

    # ApproximateTimeSynchronizer to synchronize messages based on timestamps
    sync = ApproximateTimeSynchronizer(
        [imu_sub, battery_sub, rel_altitude_sub, position_sub, speed_sub, state_sub],
        queue_size=10,
        slop=0.1,  # Adjust this parameter based on your message timestamp tolerances
        allow_headerless=True
    )
    sync.registerCallback(callback)

    rospy.spin()


if __name__ == '__main__':
    synchronize_topics()
   
   
