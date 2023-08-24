# EKF for IMU and GPS FUSION
![Static Badge](https://img.shields.io/badge/效果展示-blue)
![x](./config/x.png)
![y](./config/y.png)
![效果图](./config/y.gif)

![Static Badge](https://img.shields.io/badge/ekf流程-blue)  
![Static Badge](https://img.shields.io/badge/状态方程-red)
$$
\begin{aligned}
&x = f(x, u) + w \\
&w\in N(0,Q)
\end{aligned}
$$

![Static Badge](https://img.shields.io/badge/观测方程-red)
$$
\begin{aligned}
&z = h(x) + v \\
&v\in N(0,R)
\end{aligned}
$$

![Static Badge](https://img.shields.io/badge/预测方程-green)
$$
\begin{align}
&x_{k/k-1}=Ax_{k-1/k-1}+Bu_k\tag{1}\\
&P_{k/k-1}=AP_{k-1/k-1}A^T+Q\tag{2}\\
\end{align}
$$

![Static Badge](https://img.shields.io/badge/更新方程-green)
$$
\begin{align}
&K_k=\frac{P_{k/k-1}H^T}{HP_{k-1/k-1}H^T+R} \tag{3}\\
&x_{k/k}=x_{k/k-1}+K_k(z_k-Hx_{k/k-1}) \tag{4}\\
&P_{k/k}=(I-K_kH)P_{k/k-1} \tag{5}
\end{align}
$$

![Static Badge](https://img.shields.io/badge/收集数据-purple)   
[具体imu与gps文档](https://pan.baidu.com/s/1bigJYQUTd0TCfswACV3pUg?pwd=8919)
```bash
gps:
roslaunch roslaunch nmea_navsat_driver nmea_serial_driver.launch
imu:
roslaunch ekf rviz_and_imu.launch
```

![Static Badge](https://img.shields.io/badge/转换数据-yellow)
```bash
roslaunch ekf gps_to_xyz.launch
```

![Static Badge](https://img.shields.io/badge/融合数据-orange)
```bash
python3 ekf.py
```