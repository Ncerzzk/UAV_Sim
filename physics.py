import numpy as np
from math import cos,sin


class Vector3(object):

    def __init__(self,lst):
        if isinstance(lst,Vector3):
            self.np_data=lst.np_data
        else:
            self.np_data=np.array(lst,dtype=np.float)
        self.dic = {"x": 0, "y": 1, "z": 2}

    def copy(self):
        temp=self.__new__(self.__class__)
        temp.__init__(self.np_data)
        return temp

    def __getattr__(self, item):
        if item in self.dic:
            return self.np_data[self.dic[item]]
        else:
            raise  NameError

    def __setattr__(self, key, value):
        if key=="np_data" or key=="dic":
            super().__setattr__(key,value)
        else:
            self.np_data[self.dic[key]]=value

    def __operate(self,other,operate):
        temp=self.__new__(self.__class__)
        if isinstance(other,Vector3):
            operand=other.np_data
        else:
            operand=other

        if operate=="+":
            result=self.np_data+operand
        elif operate=="-":
            result=self.np_data-operand
        elif operate=="*":
            result=self.np_data*operand
        elif operate=="/":
            result=self.np_data/operand
        elif operate=="@":
            result=self.np_data@operand
        elif operate=="neg":
            result=-self.np_data
        elif operate=="r/":
            result=operand/self.np_data
        elif operate=="r*":
            result=operand*self.np_data
        elif operate=="r+":
            result=operand+self.np_data
        elif operate=="+=":
            self.np_data+=operand
            return self
        temp.__init__(result)
        return temp

    def __iadd__(self, other):
        return self.__operate(other,"+=")

    def __add__(self, other):
        return self.__operate(other,"+")

    def __sub__(self, other):
        return self.__operate(other,"-")

    def __mul__(self, other):
        return self.__operate(other,"*")

    def __truediv__(self, other):
        return self.__operate(other,"/")

    def __matmul__(self, other):
        return self.__operate(other,"@")

    def __neg__(self):
        return self.__operate(self,"neg")

    def __radd__(self, other):
        return self.__operate(other,"r+")

    def __rtruediv__(self, other):
        return self.__operate(other,"r/")

    def __rmul__(self, other):
        return self.__operate(other,"r*")

    def __getitem__(self, item):
        if isinstance(item,str):
            return self.__getattr__(item)
        if isinstance(item,int):
            return self.np_data[item]
        else:
            raise TypeError


class Attitude(Vector3):
    def __init__(self,rpy):
        super().__init__(rpy)
        self.dic={"roll":0,"pitch":1,"yaw":2}


class CoordinateSystem(Attitude):

    def Rx(self):
        theta=self.roll
        result=np.array([
            [1,0,0],
            [0,cos(theta),-sin(theta)],
            [0,sin(theta),cos(theta)]
        ])
        return result

    def Ry(self):
        theta=self.pitch
        result=np.array([
            [cos(theta),0,sin(theta)],
            [0,1,0],
            [-sin(theta),0,cos(theta)]
        ])
        return result

    def Rz(self):
        theta=self.yaw
        result=np.array([
            [cos(theta),sin(theta),0],
            [-sin(theta),cos(theta),0],
            [0,0,1]
        ])
        return result

    def get_rotate_matrix(self,other):
        sub=other-self
        result=sub.Rz()@sub.Ry()@sub.Rx()
        return result



class MassPart:
    def __init__(self,mass,mass_position):
        self.position=mass_position
        self.mass=mass


class SimObject:
    def __init__(self,position,attitude,mass,J,step_time):
        self.origin=Vector3(position)
        self.attitude=CoordinateSystem(attitude)

        self.mass=mass # 质量
        self.J=Vector3(J) # 转动惯量（三轴）
        self.ac=Vector3([0,0,0])   # 加速度
        self.v=Vector3([0,0,0])    # 速度
        self.beta=Vector3([0,0,0]) # 角加速度
        self.w=Vector3([0,0,0]) # 角速度

        self.forces=[]
        self.torques=Vector3([0,0,0])
        self.step_time=step_time

        self.centroid_position=self.origin.copy()

    def add_force(self,force):
        self.forces.append(force)

    def sim_step(self):
        sum_force=Force([0,0,0],self.origin,self.attitude)
        torques=np.array([0.0,0.0,0.0])
        for i in self.forces:
            sum_force+=i
            i_r=i.rotate(self.attitude)
            torques+=i_r*i.origin
        self.torques=Vector3(torques)
        self.origin+=self.v*self.step_time
        self.ac=sum_force.fxyz*(1/self.mass)
        self.v+=self.ac*self.step_time

        self.attitude+=self.w*self.step_time
        self.w+=self.beta*self.step_time
        self.beta=Vector3(torques)*(1/self.J)
        # np.array 不能与 Vector3相乘
        # 因为会先调用np.array的左乘函数，将np,array分成单个元素，然后再调用 单个元素* Vector3
        # 而单个元素乘以Vector3会调用Vector3的右乘函数，返回一个Vector3，所以最后np.array * Vector3，会返回 n个 Vector3，与要求不符




class Drone:
    def __init__(self):
        self.mass_arr=[]
        self.mass_sum=0
        self.centroid=Position()

    def add_part(self,mass_part,mass_position=None):
        if mass_position:
            mass_part.position=mass_position
        self.mass_arr.append(mass_part)
        self.mass_sum+=mass_part.mass
        self.centroid=self.cal_centroid()


    def cal_centroid(self):
        centroid_position=Position()
        for i in self.mass_arr:
            centroid_position.x+=i.position.x*i.mass/self.mass_sum
            centroid_position.y+=i.position.y*i.mass/self.mass_sum
            centroid_position.z+=i.position.z*i.mass/self.mass_sum
        return centroid_position


class Force:
    def __init__(self,vector_value,origin,coordinate_system):
        self.fxyz=Vector3(vector_value)
        self.origin=Vector3(origin)
        self.coordinate_system=coordinate_system

    def rotate(self,target_coordinate_system):
        result_force=Force([0,0,0],self.origin,target_coordinate_system)
        matrix=self.coordinate_system.get_rotate_matrix(target_coordinate_system)
        new_fxyz=Vector3(matrix@self.fxyz.np_data)
        result_force.fxyz=new_fxyz
        return result_force

    def __add__(self, other):
        result_force=Force([0,0,0],self.origin,self.coordinate_system)
        matrix=other.coordinate_system.get_rotate_matrix(self.coordinate_system)
        other_new_fxyz=Vector3(matrix@other.fxyz.np_data)
        result_force.fxyz=self.fxyz+other_new_fxyz
        return result_force

    def __neg__(self):
        return Force(-self.fxyz,self.origin,self.coordinate_system)

    def __sub__(self, other):
        return self+(-other)

    def __mul__(self, other):
        f=self.fxyz
        sub=self.origin
        if isinstance(other,Vector3): # 乘以质心
            matrix=np.array([
                [0,f[2],f[1]],
                [f[2],0,f[0]],
                [f[1],f[0],0]
            ])
            result=matrix@sub.np_data.T
            return result
        else:
            raise TypeError


class Sim_Env:
    G_CONSTANT=9.8
    def __init__(self,step_time):
        self.sim_objects=[]
        self.sim_step_time=step_time

    def add_sim_object(self,oj):
        self.sim_objects.append(oj)

    def sim_step(self):
        for i in self.sim_objects:
            i.sim_step()


class PID_Control:
    def __init__(self,sim_step_tim,control_time,KP,KD,KI):
        self.cnt_max=control_time/sim_step_tim
        self.cnt=0
        self.kp=KP
        self.kd=KD
        self.ki=KI

        self.last_error=0
        self.i=0

        self.control_time=control_time
        self.last_result=0

    def control(self,target,now_value):
        self.cnt+=1
        if self.cnt<self.cnt_max:
            return self.last_result
        else:
            self.cnt=0
            error=target-now_value
            result=self.kp*error+self.kd*(error-self.last_error)+self.i*self.ki

            self.last_error=error
            self.i+=error*self.control_time
            self.last_result=result
            return result
