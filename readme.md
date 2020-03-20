### UAV_SIM

无人机力学仿真工具,主要用于仿真飞行器上的重力及各种力矩对飞行器控制的影响。

- 完成刚体的力学建模。
- TODO：
	- Chinook 重构，参数设置统一
	

### 代码示例
```python
chinook_parameters={
    'mass':0.1,
    'J':[0.125/1000]*3,
    'length':0.2,
    'height':0.1,
    'init_attitude':[0,math.pi/4,math.pi/4]
}

a = ChinookSimEnv(chinook_parameters)
a.run(30000)
```

简单的PID控制效果： 
![此处输入图片的描述][1]

[1]: https://raw.githubusercontent.com/Ncerzzk/UAV_SIM/master/pid_control.png

俯仰控制的3D动画效果：
![此处输入图片的描述][2]

[2]: https://raw.githubusercontent.com/Ncerzzk/UAV_SIM/master/example.gif

### 仿真环境执行流程
- 仿真环境执行sim_step
	- 遍历调用所有控制器的run函数
	- 遍历调用所有仿真对象的sim_step函数
		- 计算合力，计算合力矩
			- 调用每个力的sim_step函数（默认函数为空，有些力可能需要不断变化，如空气阻力等，此时可以在该力的类中重写此函数）
		- 更新位置、速度、加速度 角度、角速度、角加速度
- 仿真环境定时器运行（用于在某个时刻触发扰动等）
- 调用仿真环境的callback,用户可自己指定回调函数