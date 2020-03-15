from physics import *
import vpython as vp


def vector3_to_vector(vector3):
    return vp.vector(vector3.y, vector3.z, vector3.x)


def vector_to_vector3(vector):
    value = vector.value
    return Vector3([value[2], value[0], value[1]])


def get_new_axis(src, target, keep_axis_vector3):
    matrix = src.get_rotate_matrix(target)
    new_axis_vector3 = Vector3(matrix @ keep_axis_vector3)
    return vector3_to_vector(new_axis_vector3)


class ForceDisplay(vp.arrow):
    def __init__(self, force):
        super().__init__(pos=vector3_to_vector(force.origin),
                         axis=vector3_to_vector(force.fxyz / 20))
        self.offset = vp.vector(self.pos)
        self.coordinate_system = force.coordinate_system
        self.axis_init = vector_to_vector3(self.axis)


class SimObjDisplay:
    def __init__(self, simenv):
        self.arrows = []
        self.obj = simenv.chinook_uav
        self.obj_box = vp.box(pos=vp.vector(0, 0, 0), length= simenv.width / 10,width=simenv.width / 10,
                              height=simenv.width * 0.8)
        self.obj_box.axis=vp.vector(0,simenv.width / 10,0)

        self.obj_box_init_axis=vector_to_vector3(self.obj_box.axis)
        self.obj_box.rotate(self.obj.attitude.yaw)

        self.env = simenv
        for i in self.obj.forces:
            arrow = ForceDisplay(i)
            self.arrows.append(arrow)
            if i.coordinate_system == self.env.world_coordinate_system:
                arrow.color = vp.color.red
            else:
                arrow.color = vp.color.green

        self.last_yaw = 0

    def update(self, env):
        self.obj_box.pos = vector3_to_vector(self.obj.origin)

        rotation=-self.obj.attitude.yaw +self.last_yaw
        self.obj_box.rotate(rotation)
        self.last_yaw = self.obj.attitude.yaw
        self.obj_box.axis= get_new_axis(self.env.world_coordinate_system,self.obj.attitude,
                                        self.obj_box_init_axis)
        for i in self.arrows:

            i.pos= vector3_to_vector( self.env.world_coordinate_system.get_rotate_matrix(self.obj.attitude)@vector_to_vector3(i.offset))+self.obj_box.pos
            #i.pos = self.obj_box.pos + i.offset +posadd
            i.axis = get_new_axis(self.env.world_coordinate_system,i.coordinate_system,
                                  i.axis_init)
            i.rotate(rotation)


class ChinookSimEnv(SimEnv):
    def __init__(self, mass, J, width, height):
        super().__init__(1e-5)
        self.width = width  # 飞行器长度（宽度）
        self.height = height  # 飞行器升力距离重心的高度
        self.world_coordinate_system = CoordinateSystem([0, 0, 0])
        self.chinook_uav = SimObject([0, 0, 0], [0, 0, 0], mass, J)

        self.left_force = Force([0.5, 0, 0.5], [0, -width / 2, height], self.chinook_uav.attitude)
        self.right_force = Force([-0.5, 0, 0.5], [0, width / 2, height], self.chinook_uav.attitude)

        gravity = Gravity(self.world_coordinate_system)

        self.chinook_uav.add_force(self.left_force)
        self.chinook_uav.add_force(self.right_force)
        self.chinook_uav.add_force(gravity)

        self.add_sim_object(self.chinook_uav)

        self.display = SimObjDisplay(self)

        self.sim_call_back = self.display.update




a = ChinookSimEnv(0.1, [0.125 / 1000, 0.125 / 1000, 0.125 / 1000], 0.2, 0.1)
a.run(30000)
print(a.chinook_uav.attitude)
print("OK")
