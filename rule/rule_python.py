import numpy as np
import math
import scipy.io as scio
import matplotlib.pyplot as plt

# 工况
data_path = 'nedc.mat'
data = scio.loadmat(data_path)
cycle = data['sch_cycle']
t=cycle[:,0].astype(np.int16)
v=cycle[:,1]

k=np.arange(len(t)-1)

acc=np.diff(v)
acc =np.insert(acc,0,0)
# 车体参数
veh_gravity=9.81    # m/s^2重力加速度
veh_air_density=1.2258 # kg/m^3
veh_CD=0.55
veh_FA=1.9# (m^2)

veh_mass=1500

#  车轮及滚阻
wh_radius=0.301    # m
I_wh=10.4       # 车轮转动惯量 kg*m^2
wh_1st_rrc=0.0065
delta_avg=1.1
# 整车模型
Fr=veh_mass*veh_gravity*wh_1st_rrc
Fi=veh_mass*veh_gravity*0
Fa=0.5*veh_air_density*veh_FA*veh_CD*(v**2)
Fj=veh_mass*delta_avg*acc

Ft=Fr+Fi+Fa+Fj
#
Tt=Ft*wh_radius # 车轮处力矩需求  N
Ww=v/wh_radius*30/math.pi # 车轮处转速  rpm
Pw=Tt*Ww/9549 # 车轮处功率需求, 与发动机的功率需求之间存在效率的转换

# plt.plot(Pw)
# plt.show()
# SOC 上下限 及电池相关信息
SOC_min=0.3
SOC_max=0.9

BatC=45 # 电池容量 Ah
BatU=384 # 电池电压 V

Bat_eff=0.90 #电池平均效率
Bat_eff=0.95

Bat_Rohmic=0.0343 #Ohm 电池内阻

SOC_grid=np.zeros(len(v)*30)

SOC_grid[0]=SOC_max
EREV_MODE=0

CYCLE_STEP=len(t)-1 #一个循环多少秒 nedc1180
j=0
i=0
iter=0
while True:
    P_dem=Pw[i]*1000
    if EREV_MODE==0:
         if P_dem>=0:
            SOC_dot=(math.sqrt(BatU**2-4*P_dem*Bat_Rohmic)-BatU)/(2*Bat_Rohmic*BatC*3600)
         else:
            SOC_dot=(math.sqrt(BatU**2-4*P_dem*0.2*Bat_Rohmic)-BatU)/(2*Bat_Rohmic*BatC*3600)
         SOC_grid[j+1]=SOC_grid[j]+SOC_dot
         if SOC_grid[j+1]<0.25:
             EREV_MODE=1
    else:
        #开启增程器
        P_engine=30*1000
        if P_dem>=0:
            SOC_dot=(math.sqrt(BatU**2-4*(P_dem-P_engine)*0.5*Bat_Rohmic)-BatU)/(2*Bat_Rohmic*BatC*3600)
        else:
            SOC_dot=(math.sqrt(BatU**2-4*(P_dem*0.2-P_engine)*0.5*Bat_Rohmic)-BatU)/(2*Bat_Rohmic*BatC*3600)
        SOC_grid[j+1]=SOC_grid[j]+SOC_dot
        if SOC_grid[j+1]>0.35:
             EREV_MODE=0
    j=j+1
    i=i+1
    #当完成一个循环时重置
    if i==CYCLE_STEP:
        i=0
        iter=iter+1
    if iter==30: #完成30个工况循环之后结束
        break




plt.plot(SOC_grid,linestyle='-',color='r',label='SOC')
plt.legend(loc='best')
plt.xlim(0,35000)
# plt.ylim(0.8,0.9)
plt.show()

