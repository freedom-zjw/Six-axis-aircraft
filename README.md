# Six-axis-aircraft
codes for a six Six-axis-aircraft

- 抛飞

## 思路

在抛飞模式下，检测竖直方向加速度是否大于一定值，大于则执行抛飞：直接给电机送去油门值，并持续
一段时间以让飞机达到一定高度。 最后通过迭代油门值，使得飞机平缓下降，最终停止在地面。

+ 修改的文件
  - FTC_Motor.h 增加计时变量，用于控制飞行时间；
  - FTC_Motor.cpp 抛飞代码实现
