# IMU

## MPU6050姿态解算

### 基本功能

- 3轴陀螺仪

  陀螺仪，测量绕**$XYZ$**轴转动的角速度，对角速度积分可以得到角度

- 3轴加速度计

  加速度计，测量的是**$XYZ$**轴收到的加速度。在静止时，测量的时重力加速度，因此物体倾斜时根据重力的分力可以粗略计算角度。在运动时，除了重力加速度，还叠加了运动产生的加速度。
  
  **满足符合右手螺旋定则**

  ![](./image/MPU6050.png)

  **三轴定义如图示**





### 姿态定义

- 绕IMU**Z轴**旋转：**偏航角Yaw**，  转动**$\theta$**角度
- 绕IMU**X轴**旋转：**俯仰角Pitch**，转动**$\alpha$**角度
- 绕IMU**Y轴**旋转：**横滚角Roll**，   转动**$\beta$**角度

#### 旋转变化

##### 二维旋转变换

<img src="./image/2dimension.png" alt="2dimension" style="zoom:25%" style="float:left;">

##### 三维旋转变换


$$
&\left[ {\begin{array}{*{20}{c}}
{\bar x}\\
{\bar y}\\
{\bar z}
\end{array}} \right] = \left[ {\begin{array}{*{20}{c}}
{\cos (\theta )}&{ - \sin (\theta )}&0\\
{\sin (\theta )}&{\cos (\theta )}&0\\
0&0&1
\end{array}} \right]\left[ {\begin{array}{*{20}{c}}
x\\
y\\
z
\end{array}} \right]
$$


- 由“绕谁谁不变”的原则，同样的方式得到$XYZ$轴旋转的公式；示意图如下

<center class="half">
    <img src="./image/z_Re.jpg" alt="z_Re" style="zoom:33% "/>
    <img src="./image/y_Re.jpg" alt="y_Re" style="zoom:33% "/>
    <img src="./image/x_Re.jpg" alt="x_Re" style="zoom:33% "/>
</center>


- 绕**$Z$轴偏航角**旋转角度为$\theta$，则旋转矩阵$R_z$为

$$
{R_z} = \left[ {\begin{array}{*{20}{c}}
{\cos (\theta )}&{ - \sin (\theta )}&0\\
{\sin (\theta )}&{\cos (\theta )}&0\\
0&0&1
\end{array}} \right]\\
$$

- 绕**$Y$轴横滚角**旋转角度为$\beta$，则旋转矩阵$R_y$为

$$
{R_y} = \left[ {\begin{array}{*{20}{c}}
{\cos (\beta )}&0&{\sin (\beta )}\\
0&1&0\\
{ - \sin (\beta )}&0&{\cos (\beta )}
\end{array}} \right]\\
$$

- 绕**$X$轴俯仰角**旋转角度为$\alpha$，则旋转矩阵$R_x$为

$$
{R_x} = \left[ {\begin{array}{*{20}{c}}
1&0&0\\
0&{\cos (\alpha )}&{ - \sin (\alpha )}\\
0&{\sin (\alpha )}&{\cos (\alpha )}
\end{array}} \right]
$$

##### 欧拉角

欧拉角是一种变化，而**不是对物体实际运动的描述**。对欧拉角的变换是顺序的，不同旋转顺序，最后得到的结果也不同

<img src="./image/rotation.gif" alt="2dimension" style="zoom:80%" style="float:left;">

##### 欧拉角->旋转矩阵

绕三轴旋转矩阵（绕**全局坐标系**下的X轴，再绕Y轴，最后绕Z轴），则三轴旋转矩阵为：
$$
M_{ZYX}=\left[ {\begin{array}{*{20}{c}}
{\cos (\theta )}&{ - \sin (\theta )}&0\\
{\sin (\theta )}&{\cos (\theta )}&0\\
0&0&1
\end{array}} \right] \left[ {\begin{array}{*{20}{c}}
{\cos (\beta )}&0&{\sin (\beta )}\\
0&1&0\\
{ - \sin (\beta )}&0&{\cos (\beta )}
\end{array}} \right] \left[ {\begin{array}{*{20}{c}}
1&0&0\\
0&{\cos (\alpha )}&{ - \sin (\alpha )}\\
0&{\sin (\alpha )}&{\cos (\alpha )}
\end{array}} \right]\\

M_{ZYX}=\begin{bmatrix} 
\cos(\theta) \cos(\beta) & \cos(\theta) \sin(\beta) \sin(\alpha) - \sin(\theta) \cos(\alpha) & \cos(\theta) \sin(\beta) \cos(\alpha) + \sin(\theta) \sin(\alpha) \\ 
\sin(\theta) \cos(\beta) & \sin(\theta) \sin(\beta) \sin(\alpha) + \cos(\theta) \cos(\alpha) & \sin(\theta) \sin(\beta) \cos(\alpha) - \cos(\theta) \sin(\alpha) \\ 
-\sin(\beta) & \cos(\beta) \sin(\alpha) & \cos(\beta) \cos(\alpha) 
\end{bmatrix}
$$
**注意：绕轴旋转顺序不一样，得到的三轴旋转矩阵也不一样，选择合适的旋转顺序避免万向锁问题**

##### 旋转矩阵->欧拉角

利用旋转矩阵元素的相乘、相除、反三角函数等操作解算欧拉角；

- $Yaw$偏航角角度，  $Z$轴
  $$
  \theta  = \arctan2(\sin \theta \cos \beta ,\cos \theta \cos \beta ) = \arctan2(m_{21},m_{11})
  $$

- $Roll$横滚角角度，   $Y$轴

  $$
  \beta  = \arcsin (\sin \beta ) = \arcsin (-m_{31})
  $$

- $Pitch$俯仰角角度，$X$轴

  $$
  \alpha  = \arctan2(\cos \beta \sin \alpha ,\cos \beta \cos \alpha ) = \arctan2(m_{32},m_{33})
  $$

**注意：$Pitch$角度，当$sin(\beta)=\pm1$即$cos(\beta) = 0$时，欧拉角提取表达式分子分母都为0，$\arctan2$就没有意义了**，此时看起来是本地坐标系$Y$轴与全局坐标系的反方向$X$轴重合，导致的万向锁死问题[^1]

#### 四元数

欧拉旋转是有万向节锁死的问题，而四元数这个数学工具可以避免这种情况。使用单位四元数 $q = {q_0} + {q_1}i + {q_2}j + {q_3}k$ 来表示旋转，

$\left\| q \right\| = {q_0}^2 + {q_1}^2 + {q_2}^2 + {q_3}^2 = 1$

${i^2} = {j^2} = {k^2} =  - 1$

$ij =  - ji = k$

$jk =  - kj = i$

$ki =  - ik = j$

四元数的表示方式：**复数式、矢量式、三角式、指数式、矩阵式**；

##### 四元数性质[^2]

###### 共轭四元数

一个四元数$q = {q_0} + {q_1}i + {q_2}j + {q_3}k = s + \nu $的共轭表示为$\bar q = {q_0} - {q_1}i - {q_2}j - {q_3}k = s - \nu$

一个四元数和它的共轭的乘积等于自身的点乘，也等于该四元数长度的平方。即

$q\bar q = \bar qq = q \cdot q = {\left\| q \right\|^2}$

###### 四元数的逆

一个非零四元数$q$的逆$q^{-1}=\frac{\bar q}{\left\| q \right\|^2}$。显然$qq^{-1}=q^{-1}q=1$。

###### 四元数乘法

- 假设有四元数$Q = {q_0} + {q_1}i + {q_2}j + {q_3}k$和$P = {p_0} + {p_1}i + {p_2}j + {p_3}k$

$$
\begin{array}{l}
Q \times P  &= &({q_0} + {q_1}i + {q_2}j + {q_3}k)\times({p_0} + {p_1}i + {p_2}j + {p_3}k)\\
 &= &({q_0}{p_0} - {q_1}{p_1} - {q_2}{p_2} - {q_3}{p_3}) + ({q_0}{p_1} + {q_1}{p_0} + {q_2}{p_3} - {q_3}{p_2})i + \\
 & &({q_0}{p_2} - {q_1}{p_3} + {q_2}{p_0} + {q_3}{p_1})j + ({q_0}{p_3} + {q_1}{p_2} - {q_2}{p_1} + {q_3}{p_0})k
\end{array}
$$

- **矩阵式**四元数乘法；表示为$Q = {\left[ {\begin{array}{*{20}{c}}
  {{q_0}}&{{q_1}}&{{q_2}}&{{q_3}}\end{array}} \right]^T}$和$P = {\left[ {\begin{array}{*{20}{c}}
  {{p_0}}&{{p_1}}&{{p_2}}&{{p_3}}\end{array}} \right]^T}$

$$
QP = \left[ {\begin{array}{*{20}{c}}
{{q_0}}&{ - {q_1}}&{ - {q_2}}&{ - {q_3}}\\
{{q_1}}&{{q_0}}&{ - {q_3}}&{{q_2}}\\
{{q_2}}&{{q_3}}&{{q_0}}&{ - {q_1}}\\
{{q_3}}&{ - {q_2}}&{{q_1}}&{{q_0}}
\end{array}} \right]\left[ {\begin{array}{*{20}{c}}
{{p_0}}\\
{{p_1}}\\
{{p_2}}\\
{{p_3}}
\end{array}} \right] = \left[ {\begin{array}{*{20}{c}}
{{p_0}}&{ - {p_1}}&{ - {p_2}}&{ - {p_3}}\\
{{p_1}}&{{p_0}}&{{p_2}}&{ - {p_3}}\\
{{p_2}}&{ - {p_3}}&{{p_0}}&{{p_1}}\\
{{p_3}}&{{p_2}}&{ - {p_1}}&{{p_0}}
\end{array}} \right]\left[ {\begin{array}{*{20}{c}}
{{q_0}}\\
{{q_1}}\\
{{q_2}}\\
{{q_3}}
\end{array}} \right]
$$

​       注意$QP \ne PQ$

###### 四元数表示向量旋转

定义${p_v} = {\left[ {\begin{array}{*{20}{c}}
{{p_x}}&{{p_y}}&{{p_z}}\end{array}} \right]^T}$为三维空间中的一点，将其转化为纯四元数形式即$p = {\left[ {\begin{array}{*{20}{c}}
0&{{p_v}}\end{array}} \right]^T}$，令
$$
q = {\left[ {\begin{array}{*{20}{c}}
{\cos (\frac{\theta }{2})}&{v\sin (\frac{\theta }{2})}
\end{array}} \right]^T}
$$
为单位四元数，则有
$$
p' = qp{q^{ - 1}} = {\left[ {\begin{array}{*{20}{c}}
0&{{{p'}_v}}
\end{array}} \right]^T}
$$
其中${p'}$表示绕旋转轴${v}$旋转$\theta$角度后得到的新向量在**原三维空间的坐标表示**

###### 四元数表示坐标系旋转

定义向量$v_0$在$oxyz$坐标系中的表示为$\left[ {\begin{array}{*{20}{c}}
{{v_x}}&{{v_y}}&{{v_z}}\end{array}} \right]^T$，令坐标系$oxyz$绕单位旋转轴${v}$旋转$\theta$角度，得到新坐标系$o'x'y'z'$，此时$v_0$在新坐标系中表示为$\left[ {\begin{array}{*{20}{c}}{{v'_x}}&{{v'_y}}&{{v'_z}}\end{array}} \right]^T$。转化关系为
$$
{\left[ {\begin{array}{*{20}{c}}
{\begin{array}{*{20}{c}}
0&{{{v'}_x}}
\end{array}}&{{{v'}_y}}&{{{v'}_z}}
\end{array}} \right]^T} = {q^{ - 1}}{\left[ {\begin{array}{*{20}{c}}
{\begin{array}{*{20}{c}}
0&{{v_x}}
\end{array}}&{{v_y}}&{{v_z}}
\end{array}} \right]^T}q
$$
其中$q = {\left[ {\begin{array}{*{20}{c}}{\cos (\frac{\theta }{2})}&{v\sin (\frac{\theta }{2})}\end{array}} \right]^T}$	

##### 四元数->旋转矩阵[^3]

那么给定一个单位四元数，可以构造旋转矩阵：
$$
R(q) = \left[ {\begin{array}{*{20}{c}}
{1 - 2({q_2}^2 + {q_3}^2)}&{2({q_1}{q_2} - {q_0}{q_3})}&{2({q_1}{q_3} + {q_0}{q_2})}\\
{2({q_1}{q_2} + {q_0}{q_3})}&{1 - 2({q_1}^2 + {q_3}^2)}&{2({q_2}{q_3} - {q_0}{q_1})}\\
{2({q_1}{q_3} - {q_0}{q_2})}&{2({q_2}{q_3} + {q_0}{q_1})}&{1 - 2({q_1}^2 + {q_2}^2)}
\end{array}} \right]
$$

##### 四元数->欧拉角[^4]

- $Yaw$偏航角角度，  $Z$轴
  $$
  \theta  = \arctan2({2({q_1}{q_2} + {q_0}{q_3})} ,{1 - 2({q_2}^2 + {q_3}^2)} )
  $$

- $Roll$横滚角角度，   $Y$轴

  $$
  \beta  = \arcsin (2({q_0}{q_2} - {q_1}{q_3}) )
  $$

- $Pitch$俯仰角角度，$X$轴

  $$
  \alpha  = \arctan2({2({q_2}{q_3} + {q_0}{q_1})} ,{1 - 2({q_1}^2 + {q_2}^2)} )
  $$

##### 欧拉角->四元数

- 绕$Z$轴旋转（ $Yaw$偏航角）对应的四元数为

$$
{q_z} = \left( {\cos (\frac{\theta }{2}),0,0,\sin (\frac{\theta }{2})} \right)
$$

- 绕$Y$轴旋转（  $Roll$偏航角）对应的四元数为

$$
{q_y} = \left( {\cos (\frac{\beta }{2}),0,\sin (\frac{\beta }{2}),0} \right)
$$

- 绕$X$轴旋转（ $Pitch$偏航角）对应的四元数为

$$
{q_x} = \left( {\cos (\frac{\alpha }{2}),\sin (\frac{\alpha }{2}),0,0} \right)
$$

- 最终四元数相乘得到，四元数的乘法是非交换的，以$ZYX$顺序进行相乘。

$$
q & = & {q_z} \times {q_y} \times {q_x}
  = \left[ {\begin{array}{*{20}{c}}
{\cos (\frac{\theta }{2})}\\
0\\
0\\
{\sin (\frac{\theta }{2})}
\end{array}} \right]\left[ {\begin{array}{*{20}{c}}
{\cos (\frac{\beta }{2})}\\
0\\
{\sin (\frac{\beta }{2})}\\
0
\end{array}} \right]\left[ {\begin{array}{*{20}{c}}
{\cos (\frac{\alpha }{2})}\\
{\sin (\frac{\alpha }{2})}\\
0\\
0
\end{array}} \right] \\
& = & \left[ {\begin{array}{*{20}{c}}
{\cos (\frac{\theta }{2})\cos (\frac{\beta }{2})\cos (\frac{\alpha }{2}) + \sin (\frac{\theta }{2})\sin (\frac{\beta }{2})\sin (\frac{\alpha }{2})}\\
{\cos (\frac{\theta }{2})\cos (\frac{\beta }{2})\sin (\frac{\alpha }{2}) - \sin (\frac{\theta }{2})\sin (\frac{\beta }{2})\cos (\frac{\alpha }{2})}\\
{\cos (\frac{\theta }{2})\sin (\frac{\beta }{2})\cos (\frac{\alpha }{2}) + \sin (\frac{\theta }{2})\cos (\frac{\beta }{2})\sin (\frac{\alpha }{2})}\\
{\sin (\frac{\theta }{2})\cos (\frac{\beta }{2})\cos (\frac{\alpha }{2}) - \cos (\frac{\theta }{2})\sin (\frac{\beta }{2})\sin (\frac{\alpha }{2})}
\end{array}} \right]
$$

##### 四元数微分方程

定义单位四元数$q(t)$来表示旋转关系，在$t + \Delta t$时刻，旋转可表示为$q\left( {t + \Delta t} \right)$；即在过程中，机体经过微小旋转，其瞬时旋转角速度为${\omega  = {{\left[ {\begin{array}{*{20}{c}}0&{{\omega _x}}&{{\omega _y}}&{{\omega _z}}\end{array}} \right]}^T}}$，单位化
$$
\hat \omega  = \frac{\omega }{{\left\| \omega  \right\|}}
$$
在$\Delta t$时刻转过的角度为
$$
\Delta \theta  = \Delta t\left\| \omega  \right\|
$$
则这次微小旋转可由如下形式的单位四元数表示
$$
\Delta q = \cos \frac{{\Delta \theta }}{2} + \sin \frac{{\Delta \theta }}{2}\hat \omega  = \cos \frac{{\left\| \omega  \right\|\Delta t}}{2} + \sin \frac{{\left\| \omega  \right\|\Delta t}}{2}\hat \omega 
$$
连续两次的旋转可以表示为$q\left( {t + \Delta t} \right) = \Delta q \cdot q$，则有
$$
\begin{array}{l}
q\left( {t + \Delta t} \right) - q\left( t \right)& = &\left( {\cos \frac{{\left\| \omega  \right\|\Delta t}}{2} + \sin \frac{{\left\| \omega  \right\|\Delta t}}{2}\hat \omega } \right)q\left( t \right) - q\left( t \right)\\
& = &\left( { - 2{{\sin }^2}\frac{{\left\| \omega  \right\|\Delta t}}{2} + \sin \frac{{\left\| \omega  \right\|\Delta t}}{2}\hat \omega } \right)q\left( t \right)
\end{array}
$$
省略高阶项可得
$$
&q\left( {t + \Delta t} \right) - q\left( t \right) = \hat \omega \sin \frac{{\left\| \omega  \right\|\Delta t}}{2}q\left( t \right)\\
&\begin{array}{l}
\dot q\left( t \right) &=& \mathop {\lim }\limits_{\Delta t \to \infty } \frac{{q\left( {t + \Delta t} \right) - q\left( t \right)}}{{\Delta t}} = \mathop {\lim }\limits_{\Delta t \to \infty } \frac{{\hat \omega \sin \frac{{\left\| \omega  \right\|\Delta t}}{2}}}{{\Delta t}}q\left( t \right)\\
 &=& \frac{1}{2}\hat \omega \left\| \omega  \right\|q\left( t \right)\\
 &=& \frac{1}{2}\omega q\left( t \right)
\end{array}
$$


### IMU姿态解算

#### 姿态解算-陀螺仪

##### 获取陀螺仪数据

测量当前的角速度${\omega  = {{\left[ {\begin{array}{*{20}{c}}0&{{\omega _x}}&{{\omega _y}}&{{\omega _z}}\end{array}} \right]}^T}}$单位弧度/秒，按照右手螺旋定则为正

##### 计算四元数导数

四元数的时间导数与角速度关系为，初始化${q = {{\left[ {\begin{array}{*{20}{c}}{q_0}&{{q_1}}&{{q_2}}&{{q_3}}\end{array}} \right]}^T} = {{\left[ {\begin{array}{*{20}{c}}1&{0}&{0}&{0}\end{array}} \right]}^T}}$
$$
\dot q = \frac{1}{2}\omega q
$$

##### 数值积分

利用如欧拉法、四阶龙格-库塔法(RK4)等数值积分方法，更新四元数
$$
q\left( {t + \Delta t} \right) = q\left( t \right) + \dot q\left( t \right)\Delta t
$$

##### 归一化四元数

由于数值积分会引入误差，需要对四元数进行归一化
$$
{q_{new}} = \frac{{q\left( {t + \Delta t} \right)}}{{\left\| {q\left( {t + \Delta t} \right)} \right\|}}
$$



#### 姿态解算-加速度计

加速度计测量的是其感受到的加速度，在静止时，其本身是没有加速运动的，感受到的加速度与重力加速度正好相反

<img src="./image/IMU-Z.png" alt="z_Re" style="zoom:80% "/>

##### 重力计算欧拉角

$$
\left[ {\begin{array}{*{20}{c}}
{{a_x}}\\
{{a_y}}\\
{{a_z}}
\end{array}} \right] = \left[ {\begin{array}{*{20}{c}}
{ - \sin \beta }\\
{\cos \beta \sin \alpha }\\
{\cos \beta \cos \alpha }
\end{array}} \right] \cdot g
$$

- $Roll$横滚角角度，   $Y$轴

$$
\beta  =  - \arctan (\frac{{{a_x}}}{{\sqrt {{a_y}^2 + {a_z}^2} }})
$$

- $Pitch$俯仰角角度，$X$轴


$$
\alpha  = \arctan (\frac{{{a_y}}}{{{a_z}}})
$$

由公式可知$Yaw$偏航角的角度无法解算；实际运动中因为一般绕$Z$轴旋转，所以重力分量不会改变。

#### 注意事项

- **静态情况下**：所测量的加速度主要来自于重力分量，能较准确地计算角度。主要关注传感器校准、漂移、噪声和温度补偿
- **动态情况下**：在动态运动中，测量的加速度还包括运动产生的惯性加速度，此时的角度计算结果会存在误差。主要关注高动态范围、震动影响以及与陀螺仪和磁力计和融合
- **环境干扰**：避免强电磁干扰环境
- **数据预处理**：对数据进行归一化和滤波处理
- **设备校正**：定期校正确保测量精度



- **陀螺仪**：零点漂移、白噪声、温漂、积分误差

















[^1]: https://www.zhihu.com/question/23005815/answer/33971127 " 形象理解四元数"
[^2]: https://www.zhihu.com/tardis/zm/art/78987582?source_id=1005 "四元数和旋转"
[^3]: https://zhuanlan.zhihu.com/p/45404840 "三维旋转：欧拉角、四元数、旋转矩阵、轴角之间的转换"
[^4]: https://www.bilibili.com/video/BV1Nr4y1j7kn  "无伤理解欧拉角中的“万向死锁”现象"
[^5]: https://www.bilibili.com/video/BV14t421h7M4/ "四元数如何控制物体旋转"

