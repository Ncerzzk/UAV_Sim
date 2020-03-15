import numpy as np
from math import cos, sin


class Vector3(object):

    def __init__(self, lst):
        if isinstance(lst, Vector3):
            self.np_data = lst.np_data
        else:
            self.np_data = np.array(lst, dtype=np.float)
        self.dic = {"x": 0, "y": 1, "z": 2}

    def copy(self):
        temp = self.__new__(self.__class__)
        temp.__init__(self.np_data)
        return temp

    def __getattr__(self, item):
        if item in self.dic:
            return self.np_data[self.dic[item]]
        else:
            raise NameError

    def __setattr__(self, key, value):
        if key == "np_data" or key == "dic":
            super().__setattr__(key, value)
        else:
            self.np_data[self.dic[key]] = value

    def __operate(self, other, operate):
        temp = self.__new__(self.__class__)
        if isinstance(other, Vector3):
            operand = other.np_data
        else:
            operand = other
        if operate == "+":
            result = self.np_data + operand
        elif operate == "-":
            result = self.np_data - operand
        elif operate == "*":
            result = self.np_data * operand
        elif operate == "/":
            result = self.np_data / operand
        elif operate == "@":
            result = self.np_data @ operand
        elif operate == "neg":
            result = -self.np_data
        elif operate == "r/":
            result = operand / self.np_data
        elif operate == "r*":
            result = operand * self.np_data
        elif operate == "r+":
            result = operand + self.np_data
        elif operate == "+=":
            self.np_data += operand
            return self
        temp.__init__(result)
        return temp

    def __iadd__(self, other):
        return self.__operate(other, "+=")

    def __add__(self, other):
        return self.__operate(other, "+")

    def __sub__(self, other):
        return self.__operate(other, "-")

    def __mul__(self, other):
        return self.__operate(other, "*")

    def __truediv__(self, other):
        return self.__operate(other, "/")

    def __matmul__(self, other):
        return self.__operate(other, "@")

    def __neg__(self):
        return self.__operate(self, "neg")

    def __radd__(self, other):
        return self.__operate(other, "r+")

    def __rtruediv__(self, other):
        return self.__operate(other, "r/")

    def __rmul__(self, other):
        return self.__operate(other, "r*")

    def __getitem__(self, item):
        if isinstance(item, str):
            return self.__getattr__(item)
        if isinstance(item, int):
            return self.np_data[item]
        else:
            raise TypeError


class Attitude(Vector3):
    def __init__(self, rpy):
        super().__init__(rpy)
        self.dic = {"roll": 0, "pitch": 1, "yaw": 2}


class CoordinateSystem(Attitude):

    def Rx(self):
        theta = self.roll
        cos_ = cos(theta)
        sin_ = sin(theta)
        result = np.array([
            [1, 0, 0],
            [0, cos_, -sin_],
            [0, sin_, cos_]
        ])
        return result

    def Ry(self):
        theta = self.pitch
        cos_ = cos(theta)
        sin_ = sin(theta)
        result = np.array([
            [cos_, 0, sin_],
            [0, 1, 0],
            [-sin_, 0, cos_]
        ])
        return result

    def Rz(self):
        theta = self.yaw
        cos_ = cos(theta)
        sin_ = sin(theta)
        result = np.array([
            [cos_, sin_, 0],
            [-sin_, cos_, 0],
            [0, 0, 1]
        ])
        return result

    def get_rotate_matrix(self, other):
        sub = other - self
        result = sub.Rz() @ sub.Ry() @ sub.Rx()
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


class Drone:
    def __init__(self):
        self.mass_arr = []
        self.mass_sum = 0
        self.centroid = Position()

    def add_part(self, mass_part, mass_position=None):
        if mass_position:
            mass_part.position = mass_position
        self.mass_arr.append(mass_part)
        self.mass_sum += mass_part.mass
        self.centroid = self.cal_centroid()

    def cal_centroid(self):
        centroid_position = Position()
        for i in self.mass_arr:
            centroid_position.x += i.position.x * i.mass / self.mass_sum
            centroid_position.y += i.position.y * i.mass / self.mass_sum
            centroid_position.z += i.position.z * i.mass / self.mass_sum
        return centroid_position


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
        new_fxyz = Vector3(matrix @ self.fxyz.np_data)
        result_force.fxyz = new_fxyz
        return result_force

    def __add__(self, other):
        result_force = Force([0, 0, 0], self.origin, self.coordinate_system)
        matrix = other.coordinate_system.get_rotate_matrix(self.coordinate_system)
        other_new_fxyz = Vector3(matrix @ other.fxyz.np_data)
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
            result = matrix @ sub.np_data.T
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


