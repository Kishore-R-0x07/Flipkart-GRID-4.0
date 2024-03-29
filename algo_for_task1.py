from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(_file_))))

import time
import math
import argparse
import numpy as np


from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from lib_aruco_pose import *

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------    

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
    print("dlat, dlon", dLat, dLon)

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return(newlat, newlon)

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
    
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x*2 + angle_y*2) <= angle_desc)
        
#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
print('Connecting...')
vehicle = connect(args.connect)  #  ADD CONNECTION CODE HERE

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def precision_hover(tracker):
    while True:                

        marker_found, x_cm, y_cm, z_cm = tracker.track(loop=False)

        if marker_found:
            x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
            uav_location        = vehicle.location.global_relative_frame
            
            print("UAV Altitude (m) = %f" %uav_location.alt)

            #-- If high altitude, use baro rather than visual
            if uav_location.alt >= 5.0:
                z_cm = uav_location.alt*100.0
                
            angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)

            
            if time.time() >= time_0 + 1.0/freq_send:
                time_0 = time.time()
                print(" ")
                print("Altitude = %.0fcm"%z_cm)
                print("Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg))
                
                north, east             = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
                print("Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, vehicle.attitude.yaw*rad_2_deg))
                
                marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01)  
                #-- If angle is good, descend
                if check_angle_descend(angle_x, angle_y, angle_descend):
                    print("Low error: descending")
                    location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.01/freq_send))

                else:
                    location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                    
                vehicle.simple_goto(location_marker)
                print("UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon))
                print("Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon))
                
                if uav_location.alt <= hover_height:
                    break
                
    if uav_location.alt <= hover_height:
        # break
        # detect object
        while True:                
            markerobj_found, xobj_cm, yobj_cm, zobj_cm = tracker_object.track(loop=False)
            # Marker object found 
            # CODE HERE
            #
            # 
            #

            # 10s past
            if not markerobj_found:
                vehicle.simple_goto(LocationGlobalRelative(vehicle.location.lat, vehicle.location.lon, rise_height))
                while True:
                    print(" Altitude: ", vehicle.location.global_frame.alt)
                    # Break and return from function just below target altitude.
                    if vehicle.location.global_relative_frame.alt >= rise_height * 0.95:
                        print("Reached target altitude")
                        break
                    time.sleep(1)

            break
        '''--- Command to land
        if z_cm <= land_alt_cm:
            if vehicle.mode == "GUIDED":
                print (" -->>COMMANDING TO LAND<<")
                vehicle.mode = "LAND"
                time.sleep(5)
                break
        '''

def object_hover():
    pass
#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
id_to_find      = 33
marker_size     = 30 #- [cm]
marker_size_obj = 7.5 #- [cm]
freq_send       = 1 #- Hz

#For the 4 squares :
'''         ___
TAKEOFF    |2|3|
DROP       |1|4|
'''
rise_height = 5
hover_height = 2

id1 = 1
id2 = 2
id3 = 3
id4 = 4
id_drop = 5
id_takeoff = 6
land_alt_cm         = 50.0
angle_descend       = 20*deg_2_rad
land_speed_cms      = 30.0



#--- Camera calibration matrices
camera_matrix = np.array([[1.43709319e+03, 0.00000000e+00, 6.49533056e+02], [0.00000000e+00, 1.44123661e+03, 3.56905912e+02], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
camera_distortion = np.array([[ 1.16247899e-01, -3.08672662e-01,  6.75064751e-03, -1.83413572e-03, 2.80469509e+00]])                                                                           

# Tracker
tracker_takeoff = ArucoSingleTracker(id_to_find=id_takeoff, marker_size=marker_size, show_video=True, camera_matrix=camera_matrix, camera_distortion=camera_distortion)
tracker_drop = ArucoSingleTracker(id_to_find=id_drop, marker_size=marker_size, show_video=True, camera_matrix=camera_matrix, camera_distortion=camera_distortion)

tracker1 = ArucoSingleTracker(id_to_find=id1, marker_size=marker_size, show_video=True, camera_matrix=camera_matrix, camera_distortion=camera_distortion)
tracker2 = ArucoSingleTracker(id_to_find=id2, marker_size=marker_size, show_video=True, camera_matrix=camera_matrix, camera_distortion=camera_distortion)
tracker3 = ArucoSingleTracker(id_to_find=id3, marker_size=marker_size, show_video=True, camera_matrix=camera_matrix, camera_distortion=camera_distortion)                
tracker4 = ArucoSingleTracker(id_to_find=id4, marker_size=marker_size, show_video=True, camera_matrix=camera_matrix, camera_distortion=camera_distortion)

tracker_object = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size_obj, show_video=True, camera_matrix=camera_matrix, camera_distortion=camera_distortion)   

time_0 = time.time()
#
#
#
# MISSION START
#
#
#
markerobj_found = False

arm_and_takeoff(rise_height)

precision_hover(tracker=tracker_drop)
loc_drop = LocationGlobalRelative(vehicle.location.lat, vehicle.location.lon, vehicle.location.alt)
# put the above line inside the function

#------------------------------------------------------------------------
precision_hover(tracker=tracker1)
#####
loc1 = LocationGlobalRelative(vehicle.location.lat, vehicle.location.lon, vehicle.location.alt)
# Above line also should go inside function

if markerobj_found:
    object_hover()
    # Electromagnet picks it up
    # Right now at 15cm (height of electromagnet + package)
    # After picking up
    vehicle.simple_goto(loc_drop)
    while True:
        print(" Altitude: ", vehicle.location.global_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= rise_height * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    # We have reached 5m above dropzone
    dropzone_hover() # Code will be pretty similar to object_hover
    # DROPPED

    vehicle.simple_goto(loc1)
    while True:
        print(" Altitude: ", vehicle.location.global_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= rise_height * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

#----------------------------------------------------------------------REPEATS

precision_hover(tracker=tracker2)
if markerobj_found:
    object_hover()
precision_hover(tracker=tracker3)
if markerobj_found:
    object_hover()
precision_hover(tracker=tracker4)
if markerobj_found:
    object_hover()
# DEAL WITH EXCEPTIONS!!!

try:
    pass
except: # Ctrl+C
    #goto(home)
    pass
