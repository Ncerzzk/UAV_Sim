import  unittest
import math
from physics import *
import matplotlib.pyplot as plt

class TestAttitude(unittest.TestCase):
    def test_init(self):
        f=Attitude([1,2,3])
        self.assertEqual(f.roll,1)
        self.assertEqual(f.pitch,2)
        self.assertEqual(f.yaw,3)

    def test_add(self):
        f=Attitude([1,2,3])
        f2=Attitude([1,2,3])
        self.assertTrue(np.all((f + f2).np_data == np.array([1, 2, 3]) + np.array([1, 2, 3])))
        self.assertTrue(np.all((f - f2).np_data == np.array([1, 2, 3]) - np.array([1, 2, 3])))
        self.assertTrue(np.all((f * f2).np_data == np.array([1, 2, 3]) * np.array([1, 2, 3])))
        self.assertTrue(np.all((f / f2).np_data == np.array([1, 2, 3]) / np.array([1, 2, 3])))

        f2=np.array([1,2,3])
        self.assertTrue(np.all((f + f2).np_data == np.array([1, 2, 3]) + np.array([1, 2, 3])))
        self.assertTrue(np.all((f - f2).np_data == np.array([1, 2, 3]) - np.array([1, 2, 3])))
        self.assertTrue(np.all((f * f2).np_data == np.array([1, 2, 3]) * np.array([1, 2, 3])))
        self.assertTrue(np.all((f / f2).np_data == np.array([1, 2, 3]) / np.array([1, 2, 3])))


class TestCoordinateSystem(unittest.TestCase):
    def test(self):
        world_cs=CoordinateSystem([0,0,0])
        cs1=CoordinateSystem([0,0,math.pi/2])
        matrix=cs1.get_rotate_matrix(world_cs)
        print(matrix)


class TestForce(unittest.TestCase):
    world_cs=CoordinateSystem([0,0,0])
    def test_force_add(self):
        world_cs=CoordinateSystem([0,0,0])
        f1=Force([1,2,3],[0,0,0],world_cs)
        f2=Force([2,3,4],[0,0,0],world_cs)
        result=f1+f2
        self.assertTrue(np.all(result.fxyz.np_data==np.array([3,5,7])))
        result=f1-f2
        self.assertTrue(np.all(result.fxyz.np_data==np.array([-1,-1,-1])))

    def test_add_2(self):
        world_cs=CoordinateSystem([0,0,0])
        cs1=CoordinateSystem([0,0,math.pi/2])
        f1=Force([1,0,0],[0,0,0],world_cs)
        f2=Force([1,0,0],[0,0,0],cs1)
        result=f1+f2
        f1+=f2
        self.assertTrue(np.all(result.fxyz.np_data == np.array([1, 1, 0])))
        self.assertTrue(np.all(f1.fxyz.np_data == np.array([1, 1, 0])))

    def test_mul(self):
        world_cs=CoordinateSystem([0,0,0])
        f1=Force([1,0,0],[1,0,0],world_cs)
        f2=Force([0,1,0],[1,0,0],world_cs)
        result=f1*Vector3([0,0,0])
        self.assertTrue(np.all(result==np.array([0,0,0])))
        result=f2*Vector3([0,0,0])
        self.assertTrue((np.all(result==np.array([0,0,1]))))

    def test_rotate(self):
        f1=Force([1,0,0],[0,0,0],self.world_cs)
        t_cs=CoordinateSystem([0,0,math.pi/2])
        result=f1.rotate(t_cs)
        print(result.fxyz.np_data)
        self.assertTrue(np.all(result.fxyz==np.array([0,-1,0])))

class TestSimTimer(unittest.TestCase):
    def test(self):
        timer=SimTimer(1e-5)
        timer.add_event(2e-3,None)
        self.assertEqual(timer.events_arr[0][0],int(2e-3/1e-5))

        timer.add_event(1e-3,None)
        self.assertEqual(timer.events_arr[0][0], int(1e-3 / 1e-5))
        self.assertEqual(timer.events_arr[1][0], int(2e-3 / 1e-5))



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
            self.left.fxyz.z = 1 - result
            self.right.fxyz.z = 1 + result


class TestSimEnv(unittest.TestCase):
    def test(self):
        world_cs=CoordinateSystem([0,0,0])
        env=SimEnv(1e-5)
        sim_obj=SimObject([0,0,0],[math.pi/4,0,0],0.1,[0.125/1000,0.125/1000,0.125/1000])
        left=Force([0,0,10],[0,-0.1,0],sim_obj.attitude)
        right = Force([0, 0, 10], [0, 0.1, 0], sim_obj.attitude)
        gravity=Gravity(world_cs)
        #gravity=Force([0,0,-9.8*sim_obj.mass],[0,0,0],world_cs)

        boom=Force([0, 0, 20], [0, 0.2, 0], sim_obj.attitude)

        env.add_pulse_force(boom,sim_obj,0.2,0.0005)

        sim_obj.add_force(gravity)
        sim_obj.add_force(left)
        sim_obj.add_force(right)
        env.add_sim_object(sim_obj)

        #pid_controler=PID_Control(1e-5,5e-3,3,25,0)
        roll_control=RollController(5e-3,left,right,sim_obj.attitude)
        env.add_controler(roll_control)

        result=[]
        torques=[]
        attituide=[]
        x=[]
        env.sim_call_back=lambda x:attituide.append(sim_obj.attitude[0])
        env.run(60000)

        plt.plot(attituide)
        #plt.plot(result)
        plt.show()


if __name__ == '__main__':
    unittest.main()
