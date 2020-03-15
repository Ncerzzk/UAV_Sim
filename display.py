import vpython as vp
from physics import  *
import  math
import profile

class RollController(Controller):
    def __init__(self,control_time,fleft,fright,attitude):
        super().__init__(control_time)
        self.pid_controller=PID_Control(control_time,3,25,0)
        self.left=fleft
        self.right=fright
        self.attitude=attitude

    def run(self):
        if super().run():
            result=self.pid_controller.control(0,self.attitude.roll)
            self.left.fxyz.z = 10 - result
            self.right.fxyz.z = 10 + result

def vector3_to_vector(vector3):
    return vp.vector(vector3.y,vector3.z,vector3.x)

def vector_to_vector3(vector):
    value=vector.value
    return Vector3([value[2],value[0],value[1]])


world_cs = CoordinateSystem([0, 0, 0])
env = Sim_Env(1e-5)
sim_obj = SimObject([0, 0, 0], [math.pi / 4, 0, 0], 0.1, [0.125 / 1000, 0.125 / 1000, 0.125 / 1000])
left = Force([0, 0, 10], [0, -0.1, 0], sim_obj.attitude)
right = Force([0, 0, 10], [0, 0.1, 0], sim_obj.attitude)
gravity = Force([0, 0, -9.8 * sim_obj.mass], [0, 0, 0], world_cs)

boom = Force([0, 0, 20], [0, 0.2, 0], sim_obj.attitude)

sim_obj.add_force(gravity)
sim_obj.add_force(left)
sim_obj.add_force(right)
env.add_sim_object(sim_obj)
env.add_pulse_force(boom,sim_obj,0.2,0.0005)

# pid_controler=PID_Control(1e-5,5e-3,3,25,0)
roll_control = RollController(5e-3, left, right, sim_obj.attitude)
env.add_controler(roll_control)

box=vp.box(pos=vp.vector(0,0,0),length=20,height=5,width=5)

axis=vp.vector(20,0,0)
vector3_axis=vector_to_vector3(axis)

def get_new_axis(sim_obj,world_cs):
    matrix=sim_obj.attitude.get_rotate_matrix(world_cs)
    new_axis_vector3=Vector3(matrix@vector3_axis.np_data)
    return vector3_to_vector(new_axis_vector3)

def render(env):
    box.pos=vector3_to_vector(sim_obj.origin)
    box.axis=get_new_axis(sim_obj,world_cs)

env.sim_call_back=render
#env.run(30000)

p=profile.Profile()
p.runcall(env.run,1000)
p.print_stats()
print("OK")
