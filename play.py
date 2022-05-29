import time
from datetime import datetime
import math


from spherov2 import scanner
from spherov2.toy.bb8 import BB8
from spherov2.sphero_edu import EventType, SpheroEduAPI
from spherov2.types import Color
from typing import Dict

print("finding droid")
toy = scanner.find_BB8(toy_name="BB-9684")
if toy is None:
    raise "could not find droid"
print("found droid")

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
    print("stopping")
    droid.set_speed(0)
    while True:
        loc = droid.get_location()
        v = droid.get_velocity()
        a = droid.get_acceleration()
        gyro = droid.get_gyroscope()

        # logger.log(droid)
        print(f"x:{loc['x']:4} y:{loc['y']:4} vx:{v['x']:6.2f} vy:{v['y']:<6.2f} ax:{a['x']:6.2f} ay:{a['y']:6.2f} az:{a['z']:6.2f} gyro_x:{gyro['x']:6.1f} gyro_y:{gyro['y']:6.1f} gyro_z:{gyro['z']:6.1f}")
        vx = v['x']
        vy = v['y']
        if math.sqrt(vx*vx+vy*vy) < 10:
            print("stopped")
            break
        time.sleep(0.1)

def go_straight(droid: SpheroEduAPI, goal_cm: float):
    goal_tolerance = 5
    k_p = 3
    k_d = 3
    d = 0
    start_loc = droid.get_location()
    while d + goal_tolerance < goal_cm:
        error_p = goal_cm - d
        error_d = - get_current_speed(droid)
        sp = k_p * error_p + k_d * error_d
        droid.set_speed(int(clamp(sp, 0, 255)))
        time.sleep(0.1)
        d = distance(droid.get_location(), start_loc)
    stop(droid)

class DataLogger:

    def __init__(self):
        self.accelerations = []
        self.velocities = []
        self.locations = []
        self.gyros = []
        self.elapsed_times = []
        self.start_time = time.time()
    
    def log(self, droid):
        loc = droid.get_location()
        v = droid.get_velocity()
        a = droid.get_acceleration()
        gyro = droid.get_gyroscope()
        self.locations.append(loc)
        self.accelerations.append(a)
        self.velocities.append(v)
        self.gyros.append(gyro)
        self.elapsed_times.append(time.time()-self.start_time)
    
    def get_dataframe(self):
        df = pd.DataFrame()
        df["t"] = [t for t in self.elapsed_times]
        df["a_x"] = [a["x"] for a in self.accelerations]
        df["a_y"] = [a["y"] for a in self.accelerations]
        df["a_z"] = [a["z"] for a in self.accelerations]

        df["v_x"] = [v["x"] for v in self.velocities]
        df["v_y"] = [v["y"] for v in self.velocities]
        
        df["loc_x"] = [loc["x"] for loc in self.locations]
        df["loc_y"] = [loc["y"] for loc in self.locations]

        df["gyro_x"] = [gyro["x"] for gyro in self.gyros]
        df["gyro_y"] = [gyro["y"] for gyro in self.gyros]
        df["gyro_z"] = [gyro["z"] for gyro in self.gyros]
        return df
    
    def save_csv(self):
        file_path = f"sphero_log_{datetime.fromtimestamp(self.start_time).isoformat(sep='T', timespec='milliseconds')}.csv"
        self.get_dataframe().to_csv(file_path)

logger = DataLogger()      

def on_sensor_msg(*argv):
    droid = argv[0]
    loc = droid.get_location()
    v = droid.get_velocity()
    a = droid.get_acceleration()
    gyro = droid.get_gyroscope()

    logger.log(droid)
    print(f"x:{loc['x']:4} y:{loc['y']:4} vx:{v['x']:6.2f} vy:{v['y']:<6.2f} ax:{a['x']:6.2f} ay:{a['y']:6.2f} az:{a['z']:6.2f} gyro_x:{gyro['x']:6.1f} gyro_y:{gyro['y']:6.1f} gyro_z:{gyro['z']:6.1f}")


while True: # make with breakable
    with  SpheroEduAPI(toy) as droid:
        time.sleep(1.0) # give robot time to get some messages
        droid.register_event(EventType.on_sensor_streaming_data, on_sensor_msg)
        go_straight(droid, 10.0)
        logger.save_csv()
        break
        #t.sensor_control.enable('accelerometer')
        #t.sensor_control.set_count(1)
        #t.sensor_control.set_interval(20)
        # t.add_sensor_streaming_data_notify_listener() throws exceptoin

        droid.set_speed(0)
        time.sleep(0.1)
        # print(droid.get_location()['x'])
        droid.set_main_led(Color(r=0, g=0, b=0))
        droid.set_back_led(255)

        logger = DataLogger()
        droid.set_main_led(Color(r=0, g=255, b=0))
    

        droid.set_speed(200)
        print("should be moving now")

        
        for _ in range(50):
            loc = droid.get_location()
            v = droid.get_velocity()
            a = droid.get_acceleration()
            gyro = droid.get_gyroscope()

            logger.log(droid)
            print(f"x:{loc['x']:4} y:{loc['y']:4} vx:{v['x']:6.2f} vy:{v['y']:<6.2f} ax:{a['x']:6.2f} ay:{a['y']:6.2f} az:{a['z']:6.2f} gyro_x:{gyro['x']:6.1f} gyro_y:{gyro['y']:6.1f} gyro_z:{gyro['z']:6.1f}")
            time.sleep(0.1)

        print("stopping")
        droid.set_speed(0)
        while True:
            loc = droid.get_location()
            v = droid.get_velocity()
            a = droid.get_acceleration()
            gyro = droid.get_gyroscope()

            logger.log(droid)
            print(f"x:{loc['x']:4} y:{loc['y']:4} vx:{v['x']:6.2f} vy:{v['y']:<6.2f} ax:{a['x']:6.2f} ay:{a['y']:6.2f} az:{a['z']:6.2f} gyro_x:{gyro['x']:6.1f} gyro_y:{gyro['y']:6.1f} gyro_z:{gyro['z']:6.1f}")
            vx = v['x']
            vy = v['y']
            if math.sqrt(vx*vx+vy*vy) < 10:
                print("stopped")
                break
            time.sleep(0.1)

        logger.save_csv()

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


        droid.set_main_led(Color(r=0, g=255, b=255))
        break

        

print("done")
