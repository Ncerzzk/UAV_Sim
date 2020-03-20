from physics import *
import vpython as vp
import math
import matplotlib.pyplot as plt


def vector3_to_vector(vector3):
    return vp.vector(vector3.y, vector3.z, vector3.x)


def vector_to_vector3(vector):
    value = vector.value
    return Vector3([value[2], value[0], value[1]])


def get_new_axis(src, target, keep_axis_vector3):
    matrix = src.get_rotate_matrix(target)
    new_axis_vector3 = Vector3(matrix @ keep_axis_vector3)
    return vector3_to_vector(new_axis_vector3)


class ChinookController(Controller):
    def __init__(self,chinook_env):
        super().__init__(5e-3)
        self.obj=chinook_env.chinook_uav
        self.env=chinook_env
        self.pitch_pid_controller=PID_Control(5e-3,6,30,0)
        for i in chinook_env.parameters:
            self.__setattr__(i,chinook_env.parameters[i])


    def run(self):
        if not self.ready():
            return None
        pitch_result=self.pitch_pid_controller.control(0,self.obj.attitude.pitch)
        theta=pitch_result
        if theta>self.servors_limit:
            theta=self.servors_limit
        elif theta<-self.servors_limit:
            theta=-self.servors_limit
        self.env.left_force.fxyz.x=self.max_thrust*sin(theta)

        self.env.left_force.fxyz.z=self.max_thrust*cos(theta)
        self.env.right_force.fxyz.x=self.max_thrust*sin(theta)
        self.env.right_force.fxyz.z=self.max_thrust*cos(theta)


class ForceDisplay(vp.arrow):
    def __init__(self, force,world_cs):
        super().__init__(pos=vector3_to_vector(force.origin),
                         axis=vector3_to_vector(force.fxyz/20))

        self.coordinate_system = force.coordinate_system
        self.offset = vp.vector(self.pos)
        self.axis_init = vector_to_vector3(self.axis)
        self.force=force


class SimObjDisplay:
    def __init__(self, simenv):
        self.arrows = []
        self.obj = simenv.chinook_uav
        self.obj_box = vp.box(pos=vp.vector(0, 0, 0), length= simenv.length / 10,width=simenv.length / 10,
                              height=simenv.length * 0.8)
        self.obj_box.axis=vp.vector(0,simenv.length / 10,0)  # bug在这

        self.obj_box_init_axis=vector_to_vector3(self.obj_box.axis)

        self.env = simenv
        for i in self.obj.forces:
            arrow = ForceDisplay(i,self.env.world_coordinate_system)
            self.arrows.append(arrow)
            if i.coordinate_system == self.env.world_coordinate_system:
                arrow.color = vp.color.red
            else:
                arrow.color = vp.color.green

        self.last_yaw = 0
        self.result=[]

    def update(self, env):
        self.obj_box.pos = vector3_to_vector(self.obj.origin)

        rotation=-self.obj.attitude.yaw +self.last_yaw
        self.obj_box.rotate(rotation)
        self.last_yaw = self.obj.attitude.yaw
        self.obj_box.axis= get_new_axis(self.env.world_coordinate_system,self.obj.attitude,
                                        self.obj_box_init_axis)   # bug在这
        self.result.append(Vector3(self.obj.attitude))
        for i in self.arrows:
            i.pos= vector3_to_vector( self.env.world_coordinate_system.get_rotate_matrix(self.obj.attitude)@vector_to_vector3(i.offset))+self.obj_box.pos
            i.axis = get_new_axis(self.env.world_coordinate_system,i.coordinate_system,
                                  i.force.fxyz/20)
            i.rotate(rotation)


class ChinookSimEnv(SimEnv):
    def __init__(self,parameter):
        super().__init__(1e-5)
        self.parameter_init(parameter)
        #self.width = width  # 飞行器长度（宽度）
        #self.height = height  # 飞行器升力距离重心的高度
        self.world_coordinate_system = CoordinateSystem([0, 0, 0])
        self.chinook_uav = SimObject([0, 0, 0], self.init_attitude, self.mass, self.J)

        self.left_force = Force([0., 0, 0.5], [0, -self.length / 2, self.height], self.chinook_uav.attitude)
        self.right_force = Force([0., 0, 0.5], [0, self.length / 2, self.height], self.chinook_uav.attitude)

        gravity = Gravity(self.world_coordinate_system)

        self.chinook_uav.add_force(self.left_force)
        self.chinook_uav.add_force(self.right_force)
        self.chinook_uav.add_force(gravity)

        self.add_sim_object(self.chinook_uav)

        self.display = SimObjDisplay(self)
        import time
        time.sleep(0.1)
        self.display.update(None)  # 直接更新一次位置

        self.sim_call_back = self.display.update

        self.parameters={
            "max_thrust":0.7,
            "servors_limit":math.pi/4
        }

        self.add_controler(ChinookController(self))

    def parameter_init(self,parameters):
        self.mass=parameters['mass']
        self.J=parameters['J']
        self.length=parameters['length']
        #self.width=parameters['width']
        self.height=parameters['height']
        self.init_attitude=parameters['init_attitude']


chinook_parameters={
    'mass':0.1,
    'J':[0.125/1000]*3,
    'length':0.2,
    'height':0.1,
    'init_attitude':[0,math.pi/4,math.pi/4]
}

a = ChinookSimEnv(chinook_parameters)
a.run(30000)
plt.plot(a.display.result)
plt.show()
print(a.chinook_uav.attitude)
print("OK")
