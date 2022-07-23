## 一、简介
> CD/CS策略是最简单的EMS控制策略，这是基于规则的，最简单的控制策略，容易在实车上实现，通常在科研中用于与其他策略进行比较来体现其控制策略的优势。

## 二、优缺点
**优点：**

- **实现简单**
- **实时**
- **发动机可工作在高效区**

**缺点：**

- **适应力差**
- **电池频繁冲放电**
## 三、流程图
![](https://cdn.nlark.com/yuque/0/2022/jpeg/29247941/1658491504699-1b275325-501f-4fd8-8f87-c9f6f50eb1dd.jpeg)
## 四、原理
### 1、CD模式
需求功率只由电池提供，先推导状态变量SOC，

$S\dot OC=\frac{dSOC}{dt}=\frac{-I_{ess}}{C_{ess}}=\frac{\sqrt{U^2_{OCV}-4P_{ess}R_{ess}}-U_{OCV}}{2R_{ess}C_{ess}}$  

$SOC_{min} \leq SOC\leq_{max}$

> 推导SOC的导数（安时计量法）
> 
> $SOC_{k+1}=SOC_k-\int_{t_k}^{t_{k+1}}{\frac{\eta I{\rm d}t}{C_e}}$
> 
> 后边  $U_{OCV}I_{ess}-I_{ess}^2R_{ess}=P_{ess}$
> 求根公式


P_ess为动力电池功率，I_ess为动力电池电流，C_ess为动力电池额外容量，R_ess为动力电池内阻，U_ocv为动力电池开路电压
### 2、CS模式
CS模式是电量保持，模式，增程器控制采用定点控制策略，发动机在固定工作点工作。

- 需求功率由增程器提供，多余功率给电池充电
### 3、制动能量回收

- 20%回收
## 五、代码实现
### 1、流程逻辑

- 导入工况数据
- 车辆参数初始化
- 计算工况功率需求
- 工况循环
### 2、提示

- 未考虑效率问题
- 制动能量回收20%
- CD模式精度可以，可与powertrain的EV一个NEDC比较。
### 3、实现环境
1️⃣** matlab **

- 无脑建议2021b+，因为有代码提示和补全😁

2️⃣** python**

- **3.7**
- **numpy**
- **math**
- **scipy**
- **matplotlib**



## 五、仿真结果
### 1、matlab
![untitled.png](https://cdn.nlark.com/yuque/0/2022/png/29247941/1658578071597-3a409205-7c74-4792-9a64-ba8e21e76de1.png#clientId=udeb27149-9620-4&crop=0&crop=0&crop=1&crop=1&from=drop&id=ud0e49798&margin=%5Bobject%20Object%5D&name=untitled.png&originHeight=1313&originWidth=1750&originalType=binary&ratio=1&rotation=0&showTitle=false&size=55831&status=done&style=none&taskId=u95b70829-2f57-46cf-ae70-5b98f292490&title=)
![2.png](https://cdn.nlark.com/yuque/0/2022/png/29247941/1658578078270-4f799f64-5dd6-4ac0-95e7-2410ccf11405.png#clientId=udeb27149-9620-4&crop=0&crop=0&crop=1&crop=1&from=drop&id=u27de6e91&margin=%5Bobject%20Object%5D&name=2.png&originHeight=1313&originWidth=1750&originalType=binary&ratio=1&rotation=0&showTitle=false&size=54936&status=done&style=none&taskId=u40546385-963e-452c-8cf3-193d58bc6a5&title=)
### 2、python
![一个ndec_py.png](https://cdn.nlark.com/yuque/0/2022/png/29247941/1658562210451-fd7e8a04-2b65-4092-8c58-8b2d9a278129.png#clientId=u544f25ee-f912-4&crop=0&crop=0&crop=1&crop=1&from=drop&id=uf9aaa9cf&margin=%5Bobject%20Object%5D&name=%E4%B8%80%E4%B8%AAndec_py.png&originHeight=480&originWidth=640&originalType=binary&ratio=1&rotation=0&showTitle=false&size=17642&status=done&style=none&taskId=u3eb2a083-bd99-4cd6-9907-41c29be4b1a&title=)
![cdcs_py.png](https://cdn.nlark.com/yuque/0/2022/png/29247941/1658562312340-9178376f-a20f-43bf-a16d-66196b9bf5b8.png#clientId=u544f25ee-f912-4&crop=0&crop=0&crop=1&crop=1&from=drop&id=ued3e45c6&margin=%5Bobject%20Object%5D&name=cdcs_py.png&originHeight=480&originWidth=640&originalType=binary&ratio=1&rotation=0&showTitle=false&size=20480&status=done&style=none&taskId=u6391f7fd-36bd-4212-86cd-047555e015c&title=)
## 六、github链接
[https://github.com/suntong-1221/EMS_OS.git](https://github.com/suntong-1221/EMS_OS.git)
