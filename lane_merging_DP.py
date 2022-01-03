from vpython import *
import random
import math

scene2 = canvas(title = 'Simulation', width = 600, height = 200, center = vector(0,0,0), background = color.black)

### define constant
# 長度單位: m; 速度單位: m/s; 加速度單位: m/s^2
Amax = 4; Amin = -4; Vmax = 27; Vmin = 0
IR_LEN = 10; S = 5
W_EQ = S / Vmax; W_PL = IR_LEN / Vmax
CAR_LEN = 4.5; CAR_WIDTH = 1.8
LANE_NUM = 2; LANE_WIDTH = 3.65
ROAD_VEC = [vec(0, 0, 0), vec(1, 0, 0), vec(sqrt(3), 1, 0)]
TURN_ANGLE = radians(30)
IR_CENTER = vec(0, 0, 0)
IR = box(length=IR_LEN, width=1, height=CAR_WIDTH, pos=vec(IR_LEN / 2, 0, -10), axis=ROAD_VEC[1], color=color.white)
dt = 0.00005
CAR_COLOR = [color.white, color.cyan, color.orange]
DPtable = []
backtrack = []

class vehicle:
    def __init__(self, lane_id:int, car_id:int, velocity:float, accel_velocity:float, distance2IR:float):
        self.lid = lane_id
        self.cid = car_id
        self.vel = velocity
        self.acc = accel_velocity
        self.dis = distance2IR
        self.time = -1
        self.prev = None
        self.body = box(length=CAR_LEN, width=1, height=CAR_WIDTH, pos=IR_CENTER - (distance2IR + CAR_LEN / 2) * ROAD_VEC[lane_id], axis=ROAD_VEC[lane_id], color=CAR_COLOR[lane_id])
    
    def details(self):
        print(self.lid, self.cid, self.vel, self.acc, self.dis, self.time)

### Init vehicle profiles (Input data)
lane1 = []
lane1.append(vehicle(1, 1, 20, 0, 55))
lane1.append(vehicle(1, 2, 22, 0, 62))
lane1.append(vehicle(1, 3, 21, 0, 69))
lane1.append(vehicle(1, 4, 25, 0, 84))
lane1.append(vehicle(1, 5, 23, 0, 91))
lane2 = []
lane2.append(vehicle(2, 1, 22, 0, 40))
lane2.append(vehicle(2, 2, 20, 0, 60))
lane2.append(vehicle(2, 3, 25, 0, 73))
lane2.append(vehicle(2, 4, 22, 0, 80))
lane2.append(vehicle(2, 5, 21, 0, 87))
              
def min_time(car:vehicle, dis):
    if(car == None):
        return 65535
    tmp = (Vmax-car.vel)/Amax*(Vmax+car.vel)/2 # length to become max speed
    if(tmp < dis):
        return (Vmax-car.vel)/Amax + (dis-tmp)/Vmax
    else: # (v+v+at)*t/2=dis
        return (math.sqrt(car.vel**2+2*Amax*dis)-car.vel)/Amax    

def generate_result(DPtable:list, backtrack:list, lane1:list, lane2:list):
    result_stack = []
    lane_count = [len(lane1), len(lane2)]
    prev = None
    if DPtable[len(lane1)][len(lane2)][0] < DPtable[len(lane1)][len(lane2)][1]:
        result_stack.append((0, DPtable[len(lane1)][len(lane2)][0]))
        prev = backtrack[len(lane1)][len(lane2)][0]
        lane_count[0] -= 1
    else:
        result_stack.append((1, DPtable[len(lane1)][len(lane2)][1]))
        prev = backtrack[len(lane1)][len(lane2)][1]
        lane_count[1] -= 1
    
    while(prev != -1):
        cur_prev = prev
        result_stack.append((prev, DPtable[lane_count[0]][lane_count[1]][prev]))
        prev = backtrack[lane_count[0]][lane_count[1]][prev]
        lane_count[cur_prev] -= 1
    
    prev = None
    delay_time = 0
    final_time = 0
    while len(result_stack) != 0 :
        lid, time = result_stack.pop()
        final_time = time
        if lid == 0:
            lane1[lane_count[0]].time = time
            delay_time += (time - min_time(lane1[lane_count[0]], lane1[lane_count[0]].dis))
            lane1[lane_count[0]].prev = prev
            prev = lane1[lane_count[0]]
            acc = 2 * (lane1[lane_count[0]].dis - lane1[lane_count[0]].vel * time) / (time ** 2)
            lane1[lane_count[0]].acc = min(max(acc, Amin), Amax)
            lane_count[0] += 1
        else:
            lane2[lane_count[1]].time = time
            delay_time += (time - min_time(lane2[lane_count[1]], lane2[lane_count[1]].dis))
            lane2[lane_count[1]].prev = prev
            prev = lane2[lane_count[1]]
            acc = 2 * (lane2[lane_count[1]].dis - lane2[lane_count[1]].vel * time) / (time ** 2)
            lane2[lane_count[1]].acc = min(max(acc, Amin), Amax)
            lane_count[1] += 1
    print("T_last =", final_time)
    print("T_delay =", delay_time / (len(lane1) + len(lane2)))

def DP(lane1:list, lane2:list):
    for i in range(len(lane1) + 1):
        DPtable.append([])
        backtrack.append([])
        for j in range(len(lane2) + 1):
            DPtable[i].append([])
            backtrack[i].append([])
            for k in range(2):
                DPtable[i][j].append(65536)
                backtrack[i][j].append(-1)
                
    DPtable[0][0][0] = 0
    DPtable[0][0][1] = 0
    DPtable[1][0][0] = min_time(lane1[0], lane1[0].dis)
    DPtable[0][1][1] = min_time(lane2[0], lane2[0].dis)
    for i in range(2, len(lane1) + 1):
        DPtable[i][0][0] = max(min_time(lane1[i - 1], lane1[i - 1].dis), DPtable[i - 1][0][0] + W_EQ)
        backtrack[i][0][0] = 0
    
    for j in range(2, len(lane2) + 1):
        DPtable[0][j][1] = max(min_time(lane2[j - 1], lane2[j - 1].dis), DPtable[0][j - 1][1] + W_EQ)
        backtrack[0][j][1] = 1
    
    for i in range(1, len(lane1) + 1):
        for j in range(1, len(lane2) + 1):
            DPtable[i][j][0] = min(max(min_time(lane1[i - 1], lane1[i - 1].dis), DPtable[i - 1][j][0] + W_EQ), 
                                   max(min_time(lane1[i - 1], lane1[i - 1].dis), DPtable[i - 1][j][1] + W_PL))
            DPtable[i][j][1] = min(max(min_time(lane2[j - 1], lane2[j - 1].dis), DPtable[i][j - 1][0] + W_PL),
                                   max(min_time(lane2[j - 1], lane2[j - 1].dis), DPtable[i][j - 1][1] + W_EQ))
            if max(min_time(lane1[i - 1], lane1[i - 1].dis), DPtable[i - 1][j][0] + W_EQ) <= \
               max(min_time(lane1[i - 1], lane1[i - 1].dis), DPtable[i - 1][j][1] + W_PL):
                backtrack[i][j][0] = 0
            else:
                backtrack[i][j][0] = 1
            if max(min_time(lane2[j - 1], lane2[j - 1].dis), DPtable[i][j - 1][1] + W_EQ) <= \
               max(min_time(lane2[j - 1], lane2[j - 1].dis), DPtable[i][j - 1][0] + W_PL):
                backtrack[i][j][1] = 1
            else:
                backtrack[i][j][1] = 0
    
    generate_result(DPtable, backtrack, lane1, lane2)

def update_status(lane:list, lane_id:int):
    for car in lane:
        cur_vel = car.vel
        if car.vel < Vmax:
            car.vel += car.acc * dt
        dis = (cur_vel + car.vel) * dt / 2
        if car.dis > 0:
            car.body.pos += ROAD_VEC[lane_id] * dis
        else: 
            car.body.pos += ROAD_VEC[1] * dis
        car.dis -= dis
        
        if car.dis < car.vel * dt and car.dis >= 0:
            if car.prev != None:
                print(car.lid, car.cid, counttime, car.time, car.dis - car.prev.dis)
            else:
                print(car.lid, car.cid, counttime, car.time)
            car.vel = Vmax
            car.acc = 0
            if lane_id == 2:
                car.body.pos += (CAR_LEN / 2) * ROAD_VEC[lane_id]
                car.body.pos -= (CAR_LEN / 2) * ROAD_VEC[1]
                car.body.rotate(angle=TURN_ANGLE, axis=vec(0, 0, -1))
        
DP(lane1, lane2)
counttime = 0


while True:
    rate(1 / dt)    # set animation rate = 1 / dt                 
    counttime += dt
    if counttime > 10:
        break
    update_status(lane1, 1)
    update_status(lane2, 2) 
