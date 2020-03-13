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

def roll_control(fleft,fright,sim_obj,pid_controler):
    result=pid_controler.control(0,sim_obj.attitude.roll)
    fleft.fxyz.z=10-result
    fright.fxyz.z=10+result




class TestSimEnv(unittest.TestCase):
    def test(self):
        world_cs=CoordinateSystem([0,0,0])
        env=Sim_Env(1e-5)
        sim_obj=SimObject([0,0,0],[math.pi/4,0,0],0.1,[0.125/1000,0.125/1000,0.125/1000],1e-5)
        left=Force([0,0,10],[0,-0.1,0],sim_obj.attitude)
        right = Force([0, 0, 10], [0, 0.1, 0], sim_obj.attitude)
        #gravity=Force([0,0,-9.8*sim_obj.mass],[1,0,0],sim_obj.attitude)
        #sim_obj.add_force(gravity)
        sim_obj.add_force(left)
        sim_obj.add_force(right)
        env.add_sim_object(sim_obj)

        pid_controler=PID_Control(1e-5,5e-3,1,5,0)

        result=[]
        torques=[]
        attituide=[]
        x=[]
        for i in range(0,50000):
            env.sim_step()
            result.append(sim_obj.w.np_data[0])
            attituide.append(sim_obj.attitude.np_data[0])
            torques.append(sim_obj.torques.np_data[0])
            #x.append(i)
            roll_control(left,right,sim_obj,pid_controler)

        print(sim_obj.v.np_data)
        print(sim_obj.origin.np_data)
        print(sim_obj.attitude.np_data)
        plt.plot(attituide)
        #plt.plot(result)
        plt.show()


if __name__ == '__main__':
    unittest.main()
