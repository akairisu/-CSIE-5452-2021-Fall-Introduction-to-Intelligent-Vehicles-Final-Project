from vpython import *
import random
import math

scene2 = canvas(title = 'Simulation', width = 600, height = 200, center = vector(0,0,0), background = color.black)
random.seed(1)

### define constant
# 長度單位: m; 速度單位: m/s; 加速度單位: m/s^2
Amax = 5; Amin = -5; Vmax = 25; Vmin = 15
IR_LEN = 10; S = 10; PREPARE = 75
CAR_LEN = 4.5; CAR_WIDTH = 1.8
LANE_NUM = 2; LANE_WIDTH = 3.65
ROAD_VEC = [vec(0, 0, 0), vec(1, 0, 0), vec(sqrt(3), 1, 0)]
TURN_ANGLE = radians(30)
IR_CENTER = vec(0, 0, -10)
IR = box(length=IR_LEN, width=1, height=CAR_WIDTH, pos=IR_CENTER, axis=ROAD_VEC[1], color=color.white)
dt = 0.00005
CAR_COLOR = [color.white, color.cyan, color.orange]

### class definition
class vehicle:
    def __init__(self, lane_id:int, car_id:int, velocity:float, accel_velocity:float, distance2IR:float):
        self.lid = lane_id
        self.cid = car_id
        self.vel = velocity
        self.acc = accel_velocity
        self.dis = distance2IR
        self.prev = None
        self.next = None
        self.merged = False
        self.body = box(length=CAR_LEN, width=1, height=CAR_WIDTH, pos=IR_CENTER - (distance2IR + CAR_LEN / 2) * ROAD_VEC[lane_id], axis=ROAD_VEC[lane_id], color=CAR_COLOR[lane_id])
    
    def details(self):
        print(self.lid, self.cid, self.vel, self.acc, self.dis)

    def estimate(self):
        t1 = time_estimation(self)
        if self.prev is None:
            return t1
        t2 = time_estimation(self.prev) + (self.dis-self.prev.dis)/self.prev.vel
        return max(t1, t2)


class Lane:
    def __init__(self, lane_id:int):
        self.id = lane_id
        self.list = []
    
    def insert(self, car:vehicle, idx=-1)->None:
        # [append]
        if(idx < 0 or idx >= len(self.list)):
            if(len(self.list)>0):
                car.prev = self.list[-1]
                car.prev.next = car
            self.list.append(car)
            return
        # [insert] 0 <= idx < len(self.list)
        if(idx != 0):
            car.prev = self.list[idx-1]
            car.prev.next = car
        car.next = self.list[idx]
        car.next.prev = car
        self.list.insert(idx, car)
        return
    
    def need_merging(self)->vehicle: # just for lane2
        for car in self.list:
            if not car.merged and 0 < car.dis < PREPARE:
                return car
        return None
    
    def number_of_car_infront_before_IR(self, cid)->int:
        ret = 0
        for car in self.list:
            if car.cid == cid:
                break
            if car.dis > 0:
                ret += 1
        return ret
    
    def details(self)->None:
        car = self.list[0]
        while car is not None:
            car.details()
            car = car.next
        return 

### Init vehicle profiles (Input data)
lane1 = Lane(1)
lane1.insert(vehicle(1, 1, 20, 0, 27))
lane1.insert(vehicle(1, 2, 22, 0, 57))
lane1.insert(vehicle(1, 3, 21, 0, 79))
lane1.insert(vehicle(1, 4, 25, 0, 103))
lane1.insert(vehicle(1, 5, 23, 0, 125))

lane2 = Lane(2)
lane2.insert(vehicle(2, 1, 22, 0, 42))
lane2.insert(vehicle(2, 2, 20, 0, 68))
lane2.insert(vehicle(2, 3, 25, 0, 90))
lane2.insert(vehicle(2, 4, 22, 0, 112))
lane2.insert(vehicle(2, 5, 21, 0, 136))

### functions
def time_estimation(car:vehicle):
    if(car.dis < 0):
        return -65535
    if car.acc == 0:
        return car.dis/car.vel
    elif car.acc > 0:
        tmp = (Vmax-car.vel)/car.acc*(Vmax+car.vel)/2 # length to become max speed
        if(tmp < car.dis):
            return (Vmax-car.vel)/car.acc + (car.dis-tmp)/Vmax
        else: # (v+v+at)*t/2=dis
            return (math.sqrt(car.vel**2+2*car.acc*car.dis)-car.vel)/car.acc
    else:
        tmp = (car.vel-Vmin)/car.acc*(Vmin+car.vel)/2 # length to become min speed
        if(tmp < car.dis):
            return (car.vel-Vmin)/car.acc + (car.dis-tmp)/Vmin
        else: # (v+v+at)*t/2=dis
            return (math.sqrt(car.vel**2+2*car.acc*car.dis)-car.vel)/car.acc

def intersection_manager(lane1:Lane, lane2:Lane):
    car_to_insert = lane2.need_merging()
    estimate_t2 = car_to_insert.estimate()
    # decide where to insert
    epsilon = 0.1 # hyperparameter
    prev_car = None; next_car = None
    car = lane1.list[0]
    while car is not None:
        estimate_t1 = car.estimate()
        if estimate_t2 - estimate_t1 > epsilon:
            prev_car = car
        elif estimate_t1 - estimate_t2 > epsilon:
            next_car = car
            break
        else: # compare the number of car in front before IR
            diff = lane1.number_of_car_infront_before_IR(car.cid) - lane2.number_of_car_infront_before_IR(car_to_insert.cid)
            if diff < 0:
                prev_car = car
            elif diff > 0:
                next_car = car
                break
            else: # compare velocity
                if car.vel < car_to_insert.vel:
                    prev_car = car
                else:
                    next_car = car
                    break
        car = car.next
    # maintain linked list
    car_to_insert.prev = prev_car
    if prev_car is not None:
        prev_car.next = car_to_insert
    car_to_insert.next = next_car
    if next_car is not None:
        next_car.prev = car_to_insert
    car_to_insert.merged = True
    return

def CACC(car:vehicle)->float:
    if(car.prev is None):
        return Amax
    dis_to_prev_car = car.dis - car.prev.dis
    if car.prev.lid != car.lid:
        if dis_to_prev_car < IR_LEN:
            return Amin
        if dis_to_prev_car >= IR_LEN:
            if car.vel > car.prev.vel and ((car.vel - car.prev.vel) ** 2) / (Amax * 2) > (dis_to_prev_car + IR_LEN):
                return Amin
            elif car.vel < Vmax and car.prev.acc >= 0:
                return Amax
            elif car.prev.acc < 0 and car.vel >= Vmin:
                return Amin
            else:
                return 0
    if car.prev.lid == car.lid:
        if dis_to_prev_car < S:
            return Amin
        if dis_to_prev_car > S:
            if car.vel > car.prev.vel and ((car.vel - car.prev.vel) ** 2) / (Amax * 2) > (dis_to_prev_car + IR_LEN):
                return Amin
            elif car.vel < Vmax and car.prev.acc >= 0:
                return Amax
            elif car.prev.acc < 0 and car.vel >= Vmin:
                return Amin
            else:
                return 0

def ACC(car:vehicle)->float:
    if(car.prev is None):
        return Amax
    dis_to_prev_car = car.dis - car.prev.dis
    acc = (dis_to_prev_car - S)*8
    return min(max(acc, Amin), Amax)

def update_status(lane:Lane):
    for car in lane.list:
        car.vel = max(min(car.vel+car.acc*dt, Vmax), Vmin)
        if car.dis > 0:
            car.body.pos += ROAD_VEC[car.lid] * car.vel * dt 
        else: 
            car.body.pos += ROAD_VEC[1] * car.vel * dt
        car.dis -= car.vel * dt
        car.acc = CACC(car)
        
        # for animation
        if car.dis <= car.vel * dt and car.dis > 0 and car.lid == 2:
            car.body.pos += (CAR_LEN / 2) * ROAD_VEC[2]
            car.body.pos -= (CAR_LEN / 2) * ROAD_VEC[1]
            car.body.rotate(angle=TURN_ANGLE, axis=vec(0, 0, -1))
            
def gen_car(lane1:Lane, lane2:Lane): # lane1 and lane2 should not be empty
    car1 = lane1.list[-1]; car2 = lane2.list[-1]
    if car1.dis < car2.dis:
        lid = 1; cid = car1.cid+1; dis = math.ceil(car1.dis+2*S)
    else:
        lid = 2; cid = car2.cid+1; dis = math.ceil(car2.dis+2*S)
    
    vel = random.randint(Vmin, Vmax)
    dis = random.randint(dis, dis+2*S)
    if lid == 1:
        lane1.insert(vehicle(lid, cid, vel, 0, dis))
    else:
        lane2.insert(vehicle(lid, cid, vel, 0, dis))
    return
### End of functions

# lane1.details()
# lane2.details()

### Start Simulation
counttime = 0       # set time
gen_clock = 0
while True:
    rate(1 / dt)    # set animation rate = 1 / dt
    counttime = counttime + dt
    gen_clock = gen_clock + dt
    while lane2.need_merging() is not None: # entered IR
        intersection_manager(lane1, lane2)
        print(counttime)
    update_status(lane1)
    update_status(lane2)
    
    if gen_clock > 0.5 and random.random()*1000 < gen_clock:
        gen_car(lane1, lane2)
        gen_clock = 0
    
    if(counttime > 20):
        break