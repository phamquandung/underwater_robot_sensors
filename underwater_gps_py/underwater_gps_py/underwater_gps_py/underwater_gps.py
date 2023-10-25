#!/usr/bin/env python3

from scipy import rand
import math
import sys
import traceback
from termcolor import colored
import time
import requests
from requests.structures import CaseInsensitiveDict
from scipy.spatial.transform import Rotation as rotation
import pymap3d
import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, IntegerRange

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPointStamped
from ublox_msgs.msg import NavRELPOSNED9

from underwater_gps_py.gps_interface import GPSInteraface

class UnderwaterGPS_node(Node):
    def __init__(self):
        super().__init__('underwatergps')
        parameters = {
            'ros_rate': 2.0,
            'ros_rate_topside': 10.0,
            'waterlinked_url': 'http://demo.waterlinked.com',
            'use_ros_based_frame_transform': False,
            'waterlinked_api_external_master_path': '/api/v1/external/master',
            'wl_api_use_external_gps_fixed': False,
            'external_gps_fixed_lat': 0.0,
            'external_gps_fixed_lon': 0.0,
            'wl_api_use_external_gps_measurements': True,
            'wl_api_use_external_heading_fixed': False,
            'external_heading_fixed_value': 0.0,
            'wl_api_use_external_heading_measurements': True,
            'use_ros_based_locator_relative_position': False
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)

        # configure from given params
        self.configureFromParams()     
        self.add_on_set_parameters_callback(self.cb_params)
        self.init_properties()
        # init gps interface
        print(f'Waterlinked URL: {self.WATERLINKED_URL}')
        self.gps = GPSInteraface(self.WATERLINKED_URL)
        self.timer = self.create_timer(1.0/self.get_parameter('ros_rate').value, self.timer_callback)
        debug = False
        if debug:
            self.run_tests()
    
    def init_subscribers(self):
        print("Initializing ROS subscribers")
        if (self.USE_ROS_BASED_FRAME_TRANSFORM or
                (self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS and self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS)):
            """self.create_subscription(
                NavSatFix, self.EXTERNAL_GPS_MEASUREMENTS_TOPIC, self.external_gps_measurements_callback, 10)                
            self.create_subscription(
                Vector3Stamped, self.EXTERNAL_NED_MEASUREMENTS_TOPIC, self.external_ned_measurements_callback, 10)
            self.create_subscription(
                GeoPointStamped, self.EXTERNAL_MAP_ORIGIN_MEASUREMENTS_TOPIC, self.external_map_origin_measurements_callback, 10)"""
            # Subscribers to external ublox GPS and heading measurements
            self.create_subscription(
                NavSatFix, 'fix', self.external_gps_measurements_callback, 10)
            self.create_subscription(
                NavRELPOSNED9, 'navrelposned', self.external_heading_measurements_callback, 10)
        if (self.USE_ROS_BASED_LOCATOR_RELATIVE_POSITION):
            self.create_subscription(
                Vector3Stamped, 'locator_position_relative_wrt_topside', self.external_locator_relative_position_callback, 10)

    def init_publishers(self):
        print("Initializing ROS publishers")
        if not self.USE_ROS_BASED_LOCATOR_RELATIVE_POSITION:
            self.pos_relative_wrt_topside = self.create_publisher(
                Vector3Stamped, "locator_position_relative_wrt_topside", 10)
        self.gps_pub = self.create_publisher(
            GeoPointStamped, "locator_position_global", 10)
        self.ned_pub = self.create_publisher(
            Vector3Stamped, "locator_position_topside_ned", 10)
        
    def configureFromParams(self, changes = []):
        # get current params
        params = self.get_parameters(['ros_rate','ros_rate_topside','waterlinked_url',
                                    'use_ros_based_frame_transform',
                                    'waterlinked_api_external_master_path',
                                    'wl_api_use_external_gps_fixed',
                                    'external_gps_fixed_lat', 'external_gps_fixed_lon',
                                    'wl_api_use_external_gps_measurements',
                                    'wl_api_use_external_heading_fixed',
                                    'external_heading_fixed_value',
                                    'wl_api_use_external_heading_measurements',
                                    'use_ros_based_locator_relative_position'])
        params = dict((param.name, param.value) for param in params)
        # override with requested changes, if any
        params.update(dict((param.name, param.value) for param in changes)) 
        self.RATE = params['ros_rate']
        self.RATE_TOPSIDE = params['ros_rate_topside']
        self.WATERLINKED_URL = params['waterlinked_url']
        self.USE_ROS_BASED_FRAME_TRANSFORM = params['use_ros_based_frame_transform']
        self.WATERLINKED_API_EXTERNAL_MASTER_PATH = params['waterlinked_api_external_master_path']
        self.WL_API_USE_EXTERNAL_GPS_FIXED = params['wl_api_use_external_gps_fixed']
        self.EXTERNAL_GPS_FIXED_LAT_VALUE = params['external_gps_fixed_lat']
        self.EXTERNAL_GPS_FIXED_LON_VALUE = params['external_gps_fixed_lon']
        self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS = params['wl_api_use_external_gps_measurements']
        self.WL_API_USE_EXTERNAL_HEADING_FIXED = params['wl_api_use_external_heading_fixed']
        self.EXTERNAL_HEADING_FIXED_VALUE = params['external_heading_fixed_value']
        self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS = params['wl_api_use_external_heading_measurements']
        self.USE_ROS_BASED_LOCATOR_RELATIVE_POSITION = params['use_ros_based_locator_relative_position']
        self.init_subscribers()
        self.init_publishers()

    def cb_params(self, params):
        self.configureFromParams(params)
        return SetParametersResult(successful=True)
    
    def set_ros_params(self):
        return
    
    def init_properties(self):
        # Check the validity of params
        if (not self.USE_ROS_BASED_FRAME_TRANSFORM):
            if not (self.WL_API_USE_EXTERNAL_GPS_FIXED ^ self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS):
                print(colored("WL_API_USE_EXTERNAL_GPS_FIXED and WL_API_USE_EXTERNAL_GPS_MEASUREMENTS must not have the same value!", "red"))

            if not (self.WL_API_USE_EXTERNAL_HEADING_FIXED ^ self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS):
                print(colored(
                    "WL_API_USE_EXTERNAL_HEADING_FIXED and USE_EXTERNAL_HEADING_ASV_MEASUREMENTS must not have the same value!", "red"))

            if (self.WL_API_USE_EXTERNAL_GPS_FIXED and self.WL_API_USE_EXTERNAL_HEADING_FIXED):
                self.topside_external_lat_deg_dec = self.EXTERNAL_GPS_FIXED_LAT_VALUE
                # self.topside_external_lon_deg_dec_dec is external lon
                self.topside_external_lon_deg_dec_dec = self.EXTERNAL_GPS_FIXED_LON_VALUE
                self.topside_external_heading_deg = self.EXTERNAL_HEADING_FIXED_VALUE
                self.sendHttpPutRequestToTopsideAsExternalMaster()

        if (self.USE_ROS_BASED_FRAME_TRANSFORM and
            (self.WL_API_USE_EXTERNAL_GPS_FIXED or self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS or
             self.WL_API_USE_EXTERNAL_HEADING_FIXED or self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS)):
            print(colored(
                "When USE_ROS_BASED_FRAME_TRANSFORM is True then all other WL_API* booleans must be False!", "red"))

        if (not (self.USE_ROS_BASED_FRAME_TRANSFORM or
                 self.WL_API_USE_EXTERNAL_GPS_FIXED or self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS or
                 self.WL_API_USE_EXTERNAL_HEADING_FIXED or self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS)):
            print(colored(
                "Source of external IMU/GPS measurements must be used, either ROS or WL_API_FIXED/MEASUREMENTS!", "red"))

    def sendHttpPutRequestToTopsideAsExternalMaster(self):
        url = self.WATERLINKED_URL + self.WATERLINKED_API_EXTERNAL_MASTER_PATH
        headers = CaseInsensitiveDict()
        headers["accept"] = "application/vnd.waterlinked.operation_response+json"
        headers["Content-Type"] = "application/json"
        data = dict(lat=self.topside_external_lat_deg_dec,
                    lon=self.topside_external_lon_deg_dec,
                    orientation=self.topside_external_heading_deg
                    )
        try:            
            resp = requests.put(url, json=data, timeout=1.0/self.RATE)
        except requests.exceptions.RequestException as exc:
            print(colored("Exception occured {}".format(exc), "red"))

    def external_ned_measurements_callback(self, msg):
        """
        # Assuming Vector3Stamped NED coordinates msgs
        self.topside_external_pos_north = 0.0
        self.topside_external_pos_east = 0.0
        self.topside_external_pos_down = 0.0 
        """
        return 0

    def external_map_origin_measurements_callback(self, msg):
        """
        # Assuming GeoPointStamped NED origin msgs
        self.topside_external_origin_lat = 0.0
        self.topside_external_origin_lon = 0.0
        self.topside_external_origin_h = 0.0 
        """
        return 0

    def external_gps_measurements_callback(self, msg):
        # Parse topside's lat-lon coordinates 
        self.topside_external_lat_deg_dec = msg.latitude
        self.topside_external_lon_deg_dec = msg.longitude     

        if (self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS and self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS
                and hasattr(self, 'topside_external_lat_deg_dec') 
                and hasattr(self, 'topside_external_lon_deg_dec') and hasattr(self, 'topside_external_heading_deg')):
            self.sendHttpPutRequestToTopsideAsExternalMaster() 

    def external_heading_measurements_callback(self, msg):
        # Parse topside's heading
        self.topside_external_heading_deg = msg.rel_pos_heading/100000.0; # degrees

        if (self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS and self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS
                and hasattr(self, 'topside_external_lat_deg_dec') 
                and hasattr(self, 'topside_external_lon_deg_dec') and hasattr(self, 'topside_external_heading_deg')):
            self.sendHttpPutRequestToTopsideAsExternalMaster() 

    def external_navigation_status_measurements_callback(self, msg):
        time_ = time.time()
        time_nanosec, time_sec = math.modf(time_)
        self.topside_pos_time_new = time_sec + time_nanosec
                
        # Assuming NavigationStatus msgs
        # Parse local NED origin lat-lon coordinates
        self.topside_external_origin_lat = msg.origin.latitude
        self.topside_external_origin_lon = msg.origin.longitude
        self.topside_external_origin_h = msg.origin.altitude

        # Parse topside's lat-lon coordinates
        self.topside_external_lat_deg_dec = msg.global_position.latitude
        self.topside_external_lon_deg_dec = msg.global_position.longitude

        # Parse topside's NED frame coordinates
        self.topside_external_pos_north = msg.position.north
        self.topside_external_pos_east = msg.position.east
        self.topside_external_pos_down = msg.position.depth

        # Orientation and orientation rate are in radians and radians/sec using RPY
        self.topside_external_roll_rad = msg.orientation.x
        self.topside_external_pitch_rad = msg.orientation.y
        self.topside_external_heading_rad = msg.orientation.z

        self.topside_external_heading_deg = self.topside_external_roll_rad * \
            180.0/math.pi  # degrees
        if (self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS and self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS
                and not self.USE_ROS_BASED_FRAME_TRANSFORM):
            self.sendHttpPutRequestToTopsideAsExternalMaster()        

    def external_locator_relative_position_callback(self, msg):
        time_ = time.time()
        time_nanosec, time_sec = math.modf(time_)
        self.locator_rel_pos_time_new = time_sec + time_nanosec
        
        self.locator_wrt_base_relative_x = msg.vector.x
        self.locator_wrt_base_relative_y = msg.vector.y
        self.locator_wrt_base_relative_z = msg.vector.z
    
    def get_waterlinked_measuremets_global(self):
        # Debug
        # pos = self.gps.get_global_position(self.WATERLINKED_URL)
        pos = self.gps.get_global_position()
        if (pos and not self.USE_ROS_BASED_FRAME_TRANSFORM):
            self.locator_global_lat = pos["lat"]
            self.locator_global_lon = pos["lon"]

    def get_waterlinked_measuremets_relative(self):
        time_ = time.time()
        time_nanosec, time_sec = math.modf(time_)
        self.locator_rel_pos_time_new = time_sec + time_nanosec
        
        # Debug
        # data = self.gps.get_acoustic_position(self.WATERLINKED_URL)
        data = self.gps.get_acoustic_position()
        if data:
            self.locator_wrt_base_relative_x = data["x"]
            self.locator_wrt_base_relative_y = data["y"]
            self.locator_wrt_base_relative_z = data["z"]

    def transform_relative_to_ned_position(self):
        if self.USE_ROS_BASED_FRAME_TRANSFORM:
            # Debug
            # print(hasattr(self, 'topside_external_pos_north'))
            # print(hasattr(self, 'topside_external_pos_east'))
            # print(hasattr(self, 'topside_external_pos_down'))
            # print(hasattr(self, 'topside_external_heading_rad'))
            # print(hasattr(self, 'topside_external_pitch_rad'))
            # print(hasattr(self, 'topside_external_roll_rad'))
            # print(hasattr(self, 'locator_wrt_base_relative_x'))
            # print(hasattr(self, 'locator_wrt_base_relative_y'))
            # print(hasattr(self, 'locator_wrt_base_relative_z'))
            
            if (hasattr(self, 'topside_external_pos_north') and hasattr(self, 'topside_external_pos_east') and
                hasattr(self, 'topside_external_pos_down') and hasattr(self, 'topside_external_heading_rad') and
                hasattr(self, 'topside_external_pitch_rad') and hasattr(self, 'topside_external_roll_rad') and
                hasattr(self, 'locator_wrt_base_relative_x') and hasattr(self, 'locator_wrt_base_relative_y') and
                    hasattr(self, 'locator_wrt_base_relative_z')):
                topside_pos_ned = np.array([self.topside_external_pos_north,
                                            self.topside_external_pos_east,
                                            self.topside_external_pos_down])
                euler = np.array([self.topside_external_heading_rad,  # CHECK !
                                self.topside_external_pitch_rad, self.topside_external_roll_rad])
                R = rotation.from_euler('zyx', euler, degrees=False)
                pos_relative = np.array([self.locator_wrt_base_relative_x,
                                        self.locator_wrt_base_relative_y,
                                        self.locator_wrt_base_relative_z])
                self.locator_pos_ned = R.apply(pos_relative) + topside_pos_ned

            else:
                print(
                    colored("Transformation to topside NED frame lacking arguments!", "red"))

    def transform_ned_to_global_position(self):
        if self.USE_ROS_BASED_FRAME_TRANSFORM:
            if (hasattr(self, 'locator_pos_ned') and hasattr(self, 'topside_external_origin_lat') and
                    hasattr(self, 'topside_external_origin_lon') and hasattr(self, 'topside_external_origin_h')):
                n = self.locator_pos_ned[0]
                e = self.locator_pos_ned[1]
                d = self.locator_pos_ned[2]
                lat0 = self.topside_external_origin_lat
                lon0 = self.topside_external_origin_lon
                h0 = self.topside_external_origin_h
                lat, lon, h = pymap3d.ned2geodetic(
                    n, e, d, lat0, lon0, h0, ell=None, deg=True)
                self.locator_global_lat = lat
                self.locator_global_lon = lon
            else:
                print(colored(
                    "Transformation from topside NED frame to WGS84 frame lacking arguments!", "red"))
    def pub_locator_wrt_base_relative_pos(self):
        if hasattr(self, 'locator_wrt_base_relative_x') and not self.USE_ROS_BASED_LOCATOR_RELATIVE_POSITION:
            msg = Vector3Stamped()
            time_ = time.time()
            time_nanosec, time_sec = math.modf(time_)
            time_sec = int(time_sec)
            time_nanosec = int(1e9*time_nanosec)
            msg.header.stamp.sec = time_sec
            msg.header.stamp.nanosec = time_nanosec
            msg.vector.x = float(self.locator_wrt_base_relative_x)
            msg.vector.y = float(self.locator_wrt_base_relative_y)
            msg.vector.z = float(self.locator_wrt_base_relative_z)
            self.pos_relative_wrt_topside.publish(msg)

    def pub_locator_global_pos(self):
        if hasattr(self, 'locator_global_lat'):
            msg = GeoPointStamped()
            time_ = time.time()
            time_nanosec, time_sec = math.modf(time_)
            time_sec = int(time_sec)
            time_nanosec = int(1e9*time_nanosec)  # +2**32
            msg.header.stamp.sec = time_sec
            msg.header.stamp.nanosec = time_nanosec
            msg.position.latitude = float(self.locator_global_lat)
            msg.position.longitude = float(self.locator_global_lon)
            # or -self.locator_wrt_base_relative_z
            msg.position.altitude = -float(self.locator_wrt_base_relative_z)
            self.gps_pub.publish(msg)

    def pub_locator_pos_ned(self):
        if hasattr(self, 'locator_pos_ned'):
            msg = Vector3Stamped()
            time_ = time.time()
            time_nanosec, time_sec = math.modf(time_)
            time_sec = int(time_sec)
            time_nanosec = int(1e9*time_nanosec)
            msg.header.stamp.sec = time_sec
            msg.header.stamp.nanosec = time_nanosec
            msg.vector.x = self.locator_pos_ned[0]
            msg.vector.y = self.locator_pos_ned[1]
            msg.vector.z = self.locator_pos_ned[2]
            self.ned_pub.publish(msg)

    def publish_all_waterlinked_variables(self):
        time_ = time.time()
        time_nanosec, time_sec = math.modf(time_)
        self.time_pub = time_sec + time_nanosec
        if hasattr(self, 'locator_rel_pos_time_new'):
            delta_time_locator = self.time_pub - self.locator_rel_pos_time_new  
            if (delta_time_locator <= 3/self.RATE):     
                self.pub_locator_wrt_base_relative_pos()
                self.pub_locator_pos_ned()
                self.pub_locator_global_pos()
            if delta_time_locator>3/self.RATE:
                print(colored("Locator data timedout!", "red"))

    def timer_callback(self):
        self.set_ros_params()
        if not self.USE_ROS_BASED_FRAME_TRANSFORM:
            self.get_waterlinked_measuremets_global()
        if not self.USE_ROS_BASED_LOCATOR_RELATIVE_POSITION:
            self.get_waterlinked_measuremets_relative()

        self.transform_relative_to_ned_position()
        self.transform_ned_to_global_position()
        self.publish_all_waterlinked_variables() 
        
    def run_tests(self):
        # Test external GPS+heading measurements HTTP request sending
        if (self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS or
                self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS):
            print("Testing WaterLinked API external GPS+heading PUT requests")
            for iter in range(5):
                self.topside_external_lat_deg_dec = iter*10
                self.topside_external_lon_deg_dec_dec = iter*10
                self.topside_external_heading_deg = iter*360/5
                url = self.WATERLINKED_URL + self.WATERLINKED_API_EXTERNAL_MASTER_PATH
                headers = CaseInsensitiveDict()
                headers["accept"] = "application/vnd.waterlinked.operation_response+json"
                headers["Content-Type"] = "application/json"
                data = dict(lat=self.topside_external_lat_deg_dec,
                            lon=self.topside_external_lon_deg_dec_dec, orientation=self.topside_external_heading_deg)
                print(data)
                try:
                    resp = requests.put(url, json=data, timeout=1.0/self.RATE)
                    print(resp.status_code)
                    print(resp.reason)
                except requests.exceptions.RequestException as exc:
                    print(colored("Exception occured {}".format(exc), "red"))
                time.sleep(2)

        # Test frame transforms
        n = 10000.0*np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        e = n
        d = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        r = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        p = r
        y = np.array([0.0, math.pi/4.0, math.pi/2.0, math.pi, math.pi*5/4.0, math.pi *
                      3/2.0, 0.0, math.pi/4.0, math.pi/2.0, math.pi, math.pi*5/4.0, math.pi*3/2.0])
        lat0 = np.array([43.0, 43.0, 43.0, 43.0, 43.0, 43.0,
                         43.0, 43.0, 43.0, 43.0, 43.0, 43.0])
        lon0 = lat0/43.0*16.0
        x_rel = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                          1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        y_rel = x_rel
        z_rel = x_rel*5.0
        for iter in range(len(n)):
            self.topside_external_pos_north = n[iter]
            self.topside_external_pos_east = e[iter]
            self.topside_external_pos_down = d[iter]

            self.topside_external_roll_rad = r[iter]
            self.topside_external_pitch_rad = p[iter]
            self.topside_external_heading_rad = y[iter]

            self.topside_external_origin_lat = lat0[iter]
            self.topside_external_origin_lon = lon0[iter]
            self.topside_external_origin_h = 0.0

            self.locator_wrt_base_relative_x = x_rel[iter]
            self.locator_wrt_base_relative_y = y_rel[iter]
            self.locator_wrt_base_relative_z = z_rel[iter]

            self.transform_relative_to_ned_position()
            self.transform_ned_to_global_position()
            print(iter+1)
            print(self.locator_pos_ned)
            print(self.locator_global_lat)
            print(self.locator_global_lon)

def main(args=None):
    print("Started")
    rclpy.init()

    try:
        node = UnderwaterGPS_node()
        rclpy.spin(node)
        # interface.run()
    except:
        print(colored("Exception caught!", "red"))
        e = sys.exc_info()[0]
        traceback.print_tb("<p>Error: %s</p>" % e)
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
