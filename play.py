import time
from datetime import datetime
import math


from spherov2 import scanner
from spherov2.toy.bb8 import BB8
from spherov2.sphero_edu import EventType, SpheroEduAPI
from spherov2.types import Color
from typing import Dict
from spherov2.commands.power import Power

print("finding BB8")
toy = scanner.find_BB8(toy_name="BB-9684")
if toy is None:
    raise "could not find BB8"
print("found BB8")

# toy.sensor_control.set_interval(150)

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

readings = []

bb8_diameter_cm = 30

def get_current_speed(droid: SpheroEduAPI) -> float:
    v = droid.get_velocity()
    vx = v["x"] 
    vy = v["y"] 
    return math.sqrt(vx*vx+vy*vy)
    

def distance(loc1 : Dict[str,float], loc2: Dict[str,float]) -> float:
    dx = loc1["x"]-loc2["x"]
    dy = loc1["y"]-loc2["y"]
    
    return math.hypot(dx, dy)

def clamp(x, min_x, max_x):
    if x < min_x: return min_x
    if x > max_x: return max_x
    return x


def stop(droid: SpheroEduAPI):
    # print("stopping")
    droid.set_speed(0)
    while True:
        v = droid.get_velocity()

        # logger.log(droid)
        # print(f"x:{loc['x']:4} y:{loc['y']:4} vx:{v['x']:6.2f} vy:{v['y']:<6.2f} ax:{a['x']:6.2f} ay:{a['y']:6.2f} az:{a['z']:6.2f} gyro_x:{gyro['x']:6.1f} gyro_y:{gyro['y']:6.1f} gyro_z:{gyro['z']:6.1f}")
        vx = v['x']
        vy = v['y']
        if math.hypot(vx,vy) < 10:
            # print("stopped")
            break
        time.sleep(0.05)

def go_straight(droid: SpheroEduAPI, goal_cm: float):
    """ travel a straight line until goal_cm is reached, an obstacle is hit, or travel times out

    returns a tuple (result, result_location): 

    result:
       "ok"        - goal reached
       "crashed"   - stopped by a detectable crash, location will be set to crash location
       "stalled"   - stopped by lack of forward progress, location will be set to location where progess halted
       "timed out" - timed out trying for goal while still making progress
    
    result_location - dict of robot "x" and "y" location where result occured

    """
    goal_tolerance = 5
    k_p = 6
    k_d = 3
    d = 0
    timeout_seconds = goal_cm / 20.0 + 2;
    start_loc = droid.get_location()

    result = "ok"

    start_time = time.time()
    logger.reset_crash_detector()
    while d + goal_tolerance < goal_cm:
        if time.time()-start_time > timeout_seconds:
            result = "timed out"
            result_location = droid.get_location()
            print("go_straight timed out")
            break
        if logger.crash_detected:
            result = "crashed"
            result_location = logger.crash_location
            print("crashed, aborting go straight");
            break
        error_p = goal_cm - d
        error_d = - get_current_speed(droid)
        sp = k_p * error_p + k_d * error_d
        droid.set_speed(int(clamp(sp, 0, 255)))
        time.sleep(0.01)
        d = distance(droid.get_location(), start_loc)
    stop(droid)
    if result == "ok":
        result_location = droid.get_location()
    
    return (result, result_location)

# returns the signed difference between two angle in degrees
def angle_delta(target, source):
    a = target - source
    delta = (a + 180) % 360 - 180
    return delta

def turn_to_yaw(droid: SpheroEduAPI, goal_yaw: float):
    heading_tolerance = 3
    droid.set_heading(-int(goal_yaw)) # using negative because set_heading uses cw, and yaw is ccw

    while True:
        yaw = droid.get_orientation()["yaw"]
        heading_error = abs(angle_delta(yaw, goal_yaw))
        if heading_error < heading_tolerance:
            break
        time.sleep(0.05)

def x_rotation(vector,theta):
    """Rotates 3-D vector around x-axis"""
    R = np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0, np.sin(theta), np.cos(theta)]])
    return np.dot(R,vector)

def y_rotation(vector,theta):
    """Rotates 3-D vector around y-axis"""
    R = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta), 0, np.cos(theta)]])
    return np.dot(R,vector)

def z_rotation(vector,theta):
    """Rotates 3-D vector around z-axis"""
    R = np.array([[np.cos(theta), -np.sin(theta),0],[np.sin(theta), np.cos(theta),0],[0,0,1]])
    return np.dot(R,vector)

class DataLogger:

    def __init__(self):
        self.accelerations = []
        self.velocities = []
        self.locations = []
        self.gyros = []
        self.orientations = []
        self.elapsed_times = []
        self.start_time = time.time()
        self.pause = False
        self.world_frame_accelerations = []
        self.world_frame_acceleration = np.array([0, 0, 0])
        self.reset_crash_detector()
    

    def reset_crash_detector(self):
        self.crash_detected = False
        self.crash_location = None

    def log(self, droid: SpheroEduAPI):
        if self.pause: return
        loc = droid.get_location()
        v = droid.get_velocity()
        a = droid.get_acceleration()
        gyro = droid.get_gyroscope()
        orientation = droid.get_orientation()
        self.locations.append(loc)
        self.accelerations.append(a)
        self.velocities.append(v)
        self.gyros.append(gyro)
        self.orientations.append(orientation)
        self.elapsed_times.append(time.time()-self.start_time)

        world_a = y_rotation(x_rotation(np.array([a["x"], a["y"], a["z"]]), -orientation["pitch"]*math.pi/180),-orientation["roll"]*math.pi/180)
        self.world_frame_acceleration={"x":world_a[0],"y":world_a[1], "z":world_a[2]}
        self.world_frame_accelerations.append( self.world_frame_acceleration)
        if self.crash_detected == False and self.world_frame_acceleration["y"] < -1.0:
            self.crash_detected = True
            self.crash_location = loc
    
    def get_dataframe(self):
        self.pause = True
        df = pd.DataFrame()
        df["t"] = [t for t in self.elapsed_times]
        df["a_x"] = [a["x"] for a in self.accelerations]
        df["a_y"] = [a["y"] for a in self.accelerations]
        df["a_z"] = [a["z"] for a in self.accelerations]

        df["world_frame_a_x"] = [a["x"] for a in self.world_frame_accelerations]
        df["world_frame_a_y"] = [a["y"] for a in self.world_frame_accelerations]
        df["world_frame_a_z"] = [a["z"] for a in self.world_frame_accelerations]

        df["v_x"] = [v["x"] for v in self.velocities]
        df["v_y"] = [v["y"] for v in self.velocities]
        
        df["loc_x"] = [loc["x"] for loc in self.locations]
        df["loc_y"] = [loc["y"] for loc in self.locations]

        df["gyro_x"] = [gyro["x"] for gyro in self.gyros]
        df["gyro_y"] = [gyro["y"] for gyro in self.gyros]
        df["gyro_z"] = [gyro["z"] for gyro in self.gyros]

        df["roll"] = [orientation["roll"] for orientation in self.orientations]
        df["pitch"] = [orientation["pitch"] for orientation in self.orientations]
        df["yaw"] = [orientation["yaw"] for orientation in self.orientations]
        self.pause = False
        return df
    
    def save_csv(self):
        file_path = f"sphero_log_{datetime.fromtimestamp(self.start_time).isoformat(sep='T', timespec='milliseconds')}.csv"
        self.get_dataframe().to_csv(file_path)

logger = DataLogger()      

def on_collision(droid: SpheroEduAPI):
    print('on_collision')

def on_sensor_msg(droid: SpheroEduAPI):
    logger.log(droid)
    
 

def show_charts():
    global logger
    df = logger.get_dataframe()
    
    plt.subplot(2,2,1)
    plt.gca().set_aspect('equal')
    plt.title("location")
    plt.plot(df.loc_x.to_numpy() ,df.loc_y.to_numpy(),"o",label="y")
    plt.xlabel("x")
    plt.ylabel("y")

    plt.subplot(2,2,2)
    plt.title("location vs. time")
    plt.plot(df.t.to_numpy(), df.loc_x.to_numpy(), "o", label="x")
    plt.plot(df.t, df.loc_y.to_numpy(), "o", label="y")
    plt.xlabel("t")
    plt.ylabel("distance")
    plt.legend()

    plt.subplot(2,2,3)
    plt.title("velocity")
    plt.plot(df.t, df.v_x, "o", label="x")
    plt.plot(df.t, df.v_y, "o", label="y")
    plt.xlabel("t")
    plt.legend()

    plt.subplot(2,2,4)
    plt.title("gyro")
    plt.plot(df.t, df.gyro_x, label="x")
    plt.plot(df.t, df.gyro_y, label="y")
    plt.plot(df.t, df.gyro_z, label="z")
    plt.xlabel("t")
    plt.show()    

def get_yaw(droid: SpheroEduAPI):
    return -droid.get_heading()

def follow_walls_strategy_3(droid: SpheroEduAPI, gap_cm: int, step_cm: int, max_total_cm: int = 1000, max_total_seconds: float = 3*60):
    """ follow walls counterclockwise travelling along rectangular paths that bump into the wall  
    
    assumes that robot starts with blue light pointing at wall
    """

    start_distance = droid.get_distance()
    start_time = time.time()

    wall_normal = get_yaw(droid) # normal is a yaw value pointing away from the wall

    # turn toward wall
    turn_to_yaw(droid, wall_normal+180)

    # ram into wall
    (result, location) = go_straight(droid, gap_cm+step_cm)
    if result == "ok":
        print("PROBLEM - did not reach initial wall")
        stop(droid)
        return

    # perform a step - starts will sitting right next to wall
    while True:

        # check for "done" conditions

        if droid.get_distance() - start_distance >= max_total_cm:
            print("done following walls - max distance reached")
            break;

        if time.time() - start_time >= max_total_seconds:
            print("done following walls - time limit reached")
            break;


        # move away from wall
        print("moving away from wall")
        turn_to_yaw(droid, wall_normal)
        (result, location) = go_straight(droid, gap_cm)
        if result != "ok":
            print(f"PROBLEM - could not move away from wall, result was {result}")
            stop(droid)
            return

        # move foward along wall
        turn_to_yaw(droid, wall_normal - 90)
        (result, location) = go_straight(droid, step_cm)
        if result != "ok":
            print("looks to be a left turning corner, continue stepping along this newly discovered wall")
            wall_normal += 90
            continue

        # try ramming into wall
        turn_to_yaw(droid, wall_normal+180)
        (result, location) = go_straight(droid, gap_cm + step_cm)
        if result != "ok":
            print("found same wall, continuing along this wall")
            continue

        print("seems we have a right turning corner, trying to slam into new all")
        wall_normal -= 90
        turn_to_yaw(droid, wall_normal+180)
        (result, location) = go_straight(droid, gap_cm + step_cm)
        if result == "ok":
            print("PROBLEM, could not find wall after turning right")
            stop(droid)
            return
        continue

while True: # make with breakable
    with  SpheroEduAPI(toy) as droid:

        # initialize
        droid.set_main_led(Color(r=0, g=0, b=0))
        droid.set_back_led(255)
        toy.sensor_control.set_interval(50) # set up faster sensor polling
        droid.register_event(EventType.on_sensor_streaming_data, on_sensor_msg)
        droid.register_event(EventType.on_collision, on_collision)

        # calibrate
        droid.set_back_led(255)
        droid.set_main_led(Color(r=0, g=0, b=0))
        while True:
            try:
                x = int(input("Angle to move light toward wall, or Enter to continue:"))
            except:
                break
            droid.set_heading(droid.get_heading()+x)

        original_yaw = -droid.get_heading()


        follow_walls_strategy_3(droid, gap_cm=20, step_cm = 50, max_total_cm=3000)


        # # make two small squares
        # for _ in range(2):
        #     print("tracing a square")
        #     turn_to_yaw(droid, original_yaw+0);
        #     go_straight(droid, 60);
        #     turn_to_yaw(droid, original_yaw+90);
        #     go_straight(droid, 60);
        #     turn_to_yaw(droid, original_yaw+180);
        #     go_straight(droid, 60);
        #     turn_to_yaw(droid, original_yaw+270);
        #     go_straight(droid, 60);
        
        # # ram into the wall
        # print("ramming into wall")
        # turn_to_yaw(droid, original_yaw+180);
        # (result, result_location) = go_straight(droid, 100);
        # print(f"result: {result} result_location: ({result_location['x']}, {result_location['y']})")

        # # run away from
        # print("leaving wall")
        # turn_to_yaw(droid, 0);
        # go_straight(droid, 60);

        print("saving results")
        logger.save_csv()
        print("done")
        break

print("done")
