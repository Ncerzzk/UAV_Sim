import numpy as np
from math import cos, sin

class Vector3(np.ndarray):

    def __new__(cls, *args, **kwargs):
        buffer=np.array(args[0],dtype=np.float)
        return super().__new__(cls,(3,),buffer=buffer)

    def __eq__(self, other):
        return super(object).__eq__(other)

    @property
    def x(self):
        return self[0]

    @x.setter
    def x(self,value):
        self[0]=value

    @property
    def y(self):
        return self[1]

    @y.setter
    def y(self,value):
        self[1]=value

    @property
    def z(self):
        return self[2]

    @z.setter
    def z(self,value):
        self[2]=value


class Attitude(Vector3):

    @property
    def roll(self):
        return self[0]

    @roll.setter
    def roll(self,value):
        self[0]=value

    @property
    def pitch(self):
        return self[1]

    @pitch.setter
    def pitch(self,value):
        self[1]=value

    @property
    def yaw(self):
        return self[2]

    @yaw.setter
    def yaw(self,value):
        self[2]=value


class CoordinateSystem(Attitude):

    def Rx(self):
        theta = self[0]
        cos_ = cos(theta)
        sin_ = sin(theta)
        result = np.array([
            [1, 0, 0],
            [0, cos_, -sin_],
            [0, sin_, cos_]
        ])
        return result

    def Ry(self):
        theta = self[1]
        cos_ = cos(theta)
        sin_ = sin(theta)
        result = np.array([
            [cos_, 0, sin_],
            [0, 1, 0],
            [-sin_, 0, cos_]
        ])
        return result

    def Rz(self):
        theta = self[2]
        cos_ = cos(theta)
        sin_ = sin(theta)
        result = np.array([
            [cos_, sin_, 0],
            [-sin_, cos_, 0],
            [0, 0, 1]
        ])
        return result

    def Rzyx(self):
        cos_pitch = cos(self.pitch)
        cos_yaw = cos(self.yaw)
        cos_roll = cos(self.roll)
        sin_pitch=sin(self.pitch)
        sin_yaw=sin(self.yaw)
        sin_roll=sin(self.roll)
        matrix = np.array([
            [cos_pitch * cos_yaw, cos_roll * sin_yaw + cos_yaw * sin_pitch * sin_roll,
             cos_roll * cos_yaw * sin_pitch - sin_roll * sin_yaw],
            [-cos_pitch * sin_yaw, cos_roll * cos_yaw - sin_pitch * sin_roll * sin_yaw,
             - cos_yaw * sin_roll - cos_roll * sin_pitch * sin_yaw],
            [-sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll]
        ])
        return matrix

    def get_rotate_matrix(self, other):
        if other==self:
            temp=np.array([[1,0,0],
                          [0,1,0],
                          [0,0,1]])
            return temp
        sub = other - self
        #result = sub.Rz() @ sub.Ry() @ sub.Rx()
        result=sub.Rzyx()
        return result


class SimObject:
    def __init__(self, position, attitude, mass, J):
        self.origin = Vector3(position)
        self.attitude = CoordinateSystem(attitude)

        self.mass = mass  # 质量
        self.J = Vector3(J)  # 转动惯量（三轴）
        self.ac = Vector3([0, 0, 0])  # 加速度
        self.v = Vector3([0, 0, 0])  # 速度
        self.beta = Vector3([0, 0, 0])  # 角加速度
        self.w = Vector3([0, 0, 0])  # 角速度

        self.forces = []
        self.torques = Vector3([0, 0, 0])
        self.sim_step_time = None  # the sim_step_time would be set when added to the sim_env

        self.centroid_position = self.origin.copy()

    def add_force(self, force):
        self.forces.append(force)
        force.added_call_back(self)

    def remove_force(self,force):
        if force in self.forces:
            self.forces.remove(force)

    def sim_step(self):
        sum_force = Force([0, 0, 0], self.origin, self.attitude)
        torques = np.array([0.0, 0.0, 0.0])
        for i in self.forces:
            sum_force += i
            i_r = i.rotate(self.attitude)
            torques += i_r * i.origin
            i.sim_step(self)
        self.torques = Vector3(torques)
        self.origin += self.v * self.sim_step_time
        self.ac = sum_force.fxyz * (1 / self.mass)
        self.v += self.ac * self.sim_step_time

        self.attitude += self.w * self.sim_step_time
        self.w += self.beta * self.sim_step_time
        self.beta = Vector3(torques) * (1 / self.J)
        # np.array 不能与 Vector3相乘
        # 因为会先调用np.array的左乘函数，将np,array分成单个元素，然后再调用 单个元素* Vector3
        # 而单个元素乘以Vector3会调用Vector3的右乘函数，返回一个Vector3，所以最后np.array * Vector3，会返回 n个 Vector3，与要求不符


class Force:
    def __init__(self, vector_value, origin, coordinate_system):
        self.fxyz = Vector3(vector_value)
        self.origin = Vector3(origin)
        self.coordinate_system = coordinate_system

    def rotate(self, target_coordinate_system):
        if self.coordinate_system == target_coordinate_system:
            return self
        result_force = Force([0, 0, 0], self.origin, target_coordinate_system)
        matrix = self.coordinate_system.get_rotate_matrix(target_coordinate_system)
        new_fxyz = Vector3(matrix @ self.fxyz)
        result_force.fxyz = new_fxyz
        return result_force

    def __add__(self, other):

        result_force = Force([0, 0, 0], self.origin, self.coordinate_system)
        if other.coordinate_system==other.coordinate_system:
            result_force.fxyz=self.fxyz+other.fxyz
        else:
            matrix = other.coordinate_system.get_rotate_matrix(self.coordinate_system)
            other_new_fxyz = Vector3(matrix @ other.fxyz)
            result_force.fxyz = self.fxyz + other_new_fxyz
        return result_force

    def __neg__(self):
        return Force(-self.fxyz, self.origin, self.coordinate_system)

    def __sub__(self, other):
        return self + (-other)

    def __mul__(self, other):
        f = self.fxyz
        sub = self.origin
        if isinstance(other, Vector3):  # 乘以质心
            matrix = np.array([
                [0, f[2], f[1]],
                [f[2], 0, f[0]],
                [f[1], f[0], 0]
            ])
            result = matrix @ sub.T
            return result
        else:
            raise TypeError

    def added_call_back(self,sim_obj):  # this func would be called after added to a sim_obj
        pass

    def sim_step(self,sim_obj):
        # some forces may need to update in each step. If needed, you can redefine the func in subclass
        pass


class Gravity(Force):
    G=9.8

    def __init__(self,world_coordinate_system):
        super().__init__([0,0,0],[0,0,0],world_coordinate_system)

    def added_call_back(self,sim_obj):
        self.fxyz.z=-self.G*sim_obj.mass

class Sim_Env:
    G_CONSTANT = 9.8

    def __init__(self, step_time):
        self.sim_objects = []
        self.sim_step_time = step_time
        self.controllers = []
        self.field_forces=[]
        self.sim_timer=SimTimer(step_time)
        self.sim_call_back=lambda x : None

    def add_sim_object(self, oj):
        self.sim_objects.append(oj)
        oj.sim_step_time = self.sim_step_time
        for i in self.field_forces:
            i_copy=i.copy()
            oj.add_force(i_copy)

    def add_controler(self, controller):
        self.controllers.append(controller)
        controller.set_sim_step_time(self.sim_step_time)

    def sim_step(self):
        for i in self.controllers:
            i.run()

        for i in self.sim_objects:
            i.sim_step()

    def add_field_force(self,force):
        self.field_forces.append(force)

    def add_pulse_force(self,force,sim_obj,start_time,pulse_width):
        self.sim_timer.add_event(start_time,lambda :sim_obj.add_force(force))
        self.sim_timer.add_event(start_time+pulse_width,lambda :sim_obj.remove_force(force))

    def run(self,steps):
        for i in range(0,steps):
            self.sim_step()
            self.sim_timer.run(i)
            self.sim_call_back(self)


class Controller:
    def __init__(self, control_time):
        self.sim_step_time = None
        self.control_time = control_time

        self.cnt = 0

    def set_sim_step_time(self, step_time):
        self.sim_step_time = step_time
        self.cnt_max = self.control_time / self.sim_step_time

    def ready(self):
        self.cnt += 1
        if self.cnt < self.cnt_max:
            return False
        else:
            self.cnt = 0
            return True

    def run(self):
        if self.ready():
            return True
        else:
            return False


class PID_Control:
    def __init__(self, control_time, KP, KD, KI):
        self.kp = KP
        self.kd = KD
        self.ki = KI

        self.control_time = control_time

        self.last_error = 0
        self.i = 0
        self.last_result = 0

    def control(self, target, now_value):
        error = target - now_value
        result = self.kp * error + self.kd * (error - self.last_error) + self.i * self.ki

        self.last_error = error
        self.i += error * self.control_time
        self.last_result = result
        return result


class SimTimer:
    def __init__(self,sim_step_time):
        self.sim_step_time=sim_step_time
        self.events_arr=[]

    def add_event(self,trig_time,func):
        cnt=int(trig_time/self.sim_step_time)
        self.events_arr.append((cnt,func))
        self.events_arr.sort(key=lambda i:i[0])

    def run(self,cnt_value):
        if len(self.events_arr)==0:
            return None
        if cnt_value==self.events_arr[0][0]:
            item=self.events_arr.pop(0)
            item[1]()


