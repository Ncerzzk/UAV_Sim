### UAV_SIM

无人机力学仿真工具,主要用于仿真飞行器上的重力及各种力矩对飞行器控制的影响。

- 完成刚体的力学建模。

### 代码示例
```python
env=Sim_Env(1e-5)   # add new sim environment,the argument is the step_sim_time
sim_obj=SimObject([0,0,0],[math.pi/4,0,0],0.1,[0.125/1000,0.125/1000,0.125/1000],1e-5)   # add a sim objects, init the attitude and attributes like mass,J

left=Force([0,0,10],[0,-0.1,0],sim_obj.attitude) #  Init two forces to control the roll,one is on the left side of the object, the other one is on the right side
right = Force([0, 0, 10], [0, 0.1, 0], sim_obj.attitude) # and set the force coordinate system as the coordinate system of object

sim_obj.add_force(left)
sim_obj.add_force(right) # register the two forces to the object

env.add_sim_object(sim_obj) # register the object to the environment

pid_controler=PID_Control(1e-5,5e-3,1,5,0) # init a simple PID Controler

atttitude=[]  # arrays to keep the result and plot latter
for i in range(0,50000):
    env.sim_step()
    attituide.append(sim_obj.attitude.np_data[0])
    #x.append(i)
    roll_control(left,right,sim_obj,pid_controler)

plt.plot(attituide)
plt.show()
```

![此处输入图片的描述][1]

[1]: https://raw.githubusercontent.com/Ncerzzk/UAV_SIM/master/pid_control.png