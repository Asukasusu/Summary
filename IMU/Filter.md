# IMU

## MPU6050姿态解算

### 基本功能

- 3轴陀螺仪

  陀螺仪，测量绕**$XYZ$**轴转动的角速度，对角速度积分可以得到角度

- 3轴加速度计

  加速度计，测量的是**$XYZ$**轴收到的加速度。在静止时，测量的时重力加速度，因此物体倾斜时根据重力的分力可以粗略计算角度。在运动时，除了重力加速度，还叠加了运动产生的加速度。

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

##### 欧拉角->旋转矩阵

绕三轴旋转矩阵（先绕X轴，再绕Y轴，最后绕Z轴），则三轴旋转矩阵为：
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

**注意：$Pitch$角度，当$sin(\beta)=\pm1$即$cos(\beta) = 0$时，欧拉角提取表达式分子分母都为0，$\arctan2$就没有意义了**



#### 四元数

欧拉旋转是有万向节锁死的问题，而四元数这个数学工具可以避免这种情况。使用单位四元数 $q = {q_0} + {q_1}i + {q_2}j + {q_3}k$ 来表示旋转，满足符合右手螺旋定则

$\left\| q \right\| = {q_0}^2 + {q_1}^2 + {q_2}^2 + {q_3}^2 = 1$

${i^2} = {j^2} = {k^2} =  - 1$

$ij =  - ji = k$

$jk =  - kj = i$

$ki =  - ik = j$

四元数的表示方式：**复数式、矢量式、三角式、指数式、矩阵式**；

##### 四元数性质[^1]

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



##### 四元数->旋转矩阵[^2]

那么给定一个单位四元数，可以构造旋转矩阵：
$$
R(q) = \left[ {\begin{array}{*{20}{c}}
{1 - 2({q_2}^2 + {q_3}^2)}&{2({q_1}{q_2} - {q_0}{q_3})}&{2({q_1}{q_3} + {q_0}{q_2})}\\
{2({q_1}{q_2} + {q_0}{q_3})}&{1 - 2({q_1}^2 + {q_3}^2)}&{2({q_2}{q_3} - {q_0}{q_1})}\\
{2({q_1}{q_3} - {q_0}{q_2})}&{2({q_2}{q_3} + {q_0}{q_1})}&{1 - 2({q_1}^2 + {q_2}^2)}
\end{array}} \right]
$$

##### 四元数->欧拉角[^3]

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

### IMU姿态估计



[^1]: https://www.zhihu.com/question/23005815/answer/33971127 " 形象理解四元数"
[^2]: https://www.zhihu.com/tardis/zm/art/78987582?source_id=1005 "四元数和旋转"
[^3]: https://zhuanlan.zhihu.com/p/45404840 "三维旋转：欧拉角、四元数、旋转矩阵、轴角之间的转换"
[^4]: https://www.bilibili.com/video/BV14t421h7M4/ "四元数如何控制物体旋转"

