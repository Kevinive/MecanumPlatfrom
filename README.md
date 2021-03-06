# MecanumPlatfrom (未完成)

## 0.前言 ##

装备有麦克纳姆轮的小车平台拥有出色的行动能力，非常适合在狭小的空间中进行精确移动。

- 设计目标

	此次设计是针对具有驱动器的四轮麦克纳姆轮底盘，目标是将移动平台的硬件层进一步抽象，方便上层应用调用。

- 软件架构

	![](http://139.199.211.128:8088/Projects/MecanumPlatform/softwareFrame.png)
	
	[1]逆运动学解算 [2]正运动学解算

	软件部分主要实现了两大块的内容（如上图）

	- 左侧部分：上位机下发运动指令，通过控制器进行运动解算之后分发给底盘的四个电机驱动器

	- 右侧部分：底盘向运动控制器上传四个轮子的运动状态，由运动控制器进行运动解算后向上位机上传
	
	运动控制器与上位机的通信使用RS232标准串口，以保证通信可靠性；运动控制器与底盘驱动器通信通过CAN总线实现

- 硬件架构

## 1.软件设计 ##

- 功能要求

	- 通过串口与上位机通信，获取指令，同时上传小车底盘的运动参数
	- 通过CAN总线与电机驱动器通信，下发每个轮子的速度指令，同时收集每个轮子的运行参数
	- 进行麦克纳姆轮底盘的正运动学求解与逆运动学求解

### 1.1运动求解 ###

运动求解的相关资料都已经比较成熟，下面仅对如何运用理论进行实际求解的方法做简要说明。
对于麦克纳姆轮小车，完全描述其底盘运动只需要三个量：二维平面移动速度X,Y和绕Z轴的旋转速度w,而小车实际输出量为四个轮子的转速。

> 参考资料：[麦克纳姆轮及其速度分解计算](https://blog.csdn.net/banzhuan133/article/details/69229922)

- 逆运动学解算

	逆运动学解算为底盘速度->车轮速度的计算，由底盘的三个参数求车轮速度的四个变量，**可以直接求解**。

- 正运动学解算

	逆运动学解算为车轮速度->底盘速度的计算。由于是通过线性方程组，由车轮的四个参数求底盘的三个变量，因此可能出现方程组无解的情况，此处**使用最小二乘法进行拟合**。


## 2.硬件设计 ##

### MCU ###

MCU所需功能：USART x 4、CAN总线控制器 x 1、ADC x 2

MCU选型：STM32F103RET6 (64pin) 在符合硬件需求的情况下尽可能的小

### CAN驱动器 ###

使用了TJA1040芯片作为CAN总线驱动器，设计上包含了120R电阻

### 串口 ###

提供了两种串口连接方式：

- 1 MicroUSB

	此连接方式具有**下载程序**的功能，使用CH340G作为USB转串口芯片，硬件上连接至MCU的**USART1串口**。

- 2	传统9针串口线

	此为传统串口的连接方式，使用SP3232作为电平转换芯片，硬件上连接至MCU的**USART2串口**

### 电压电流检测 ###

*（敬请期待）*

### WIFI（预留） ###

预留了ESP8266串口WIFI的接口，硬件上连接至MCU的**UART4串口**

### 蓝牙（预留） ###

预留了蓝牙串口的接口，硬件上连接至**UART5串口**

