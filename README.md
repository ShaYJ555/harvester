# 简介
> 本项目是胡萝卜收割机的电控部分内容主要包含以下内容
- 硬件
- 软件
- 资料

# 硬件
**使用两个esp32配合5颗drv8825来实现功能控制**
![esp32](/source/lolin32_lite.png)

<div>
  <iframe src="hardware/machine_base_board/bom/ibom.html" width="100%" height="600px">
    Your browser does not support iframes.
  </iframe>
</div>


# 软件
> slaveboard&masterboard之间通过硬件串口2通信
## slaveboard
> 用于控制机器底盘的运动。
## masterboard
> 用于控制机器步进电机和舵机的运动。



