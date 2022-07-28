clear;
close all;
clc;
%% 工况
load("nedc.mat");
t=sch_cycle(:,1);
ve=sch_cycle(:,2);
k=1:length(sch_cycle(:,1))-1;

acc=[0;(ve(k+1)-ve(k))./(t(k+1)-t(k))];%m/s/s 加速度
%% 车体参数
veh_gravity=9.81;    % m/s^2重力加速度
veh_air_density=1.2258; % kg/m^3
veh_CD=0.55; 
veh_FA=1.9;% (m^2) (ref. WVU test data)

veh_mass=1500;
delta_avg=1.1; 
%% 车轮及滚阻
wh_radius=0.301;    % (m) 

wh_1st_rrc=0.0065;
eff_fd=0.95;
%% 整车模型
Fr=veh_mass*veh_gravity*wh_1st_rrc;     % 滚动阻力 N
Fi=veh_mass*veh_gravity.*0;  % 坡道阻力 N
Fa=0.5*veh_air_density*veh_FA*veh_CD*(ve.^2);     % 空气阻力 N
Fj=veh_mass.*delta_avg.*acc;   %加速阻力  N

Ft=Fr+Fi+Fa+Fj;% 车轮处驱动力需求，N

Tt=Ft.*wh_radius;% 车轮处力矩需求  N
Ww=ve/wh_radius*30/pi;% 车轮处转速  rpm
Pw=Tt.*Ww/9549;% 车轮处功率需求, 与发动机的功率需求之间存在效率的转换


%% SOC 上下限 及电池相关信息
SOC_min=0.3;
SOC_max=0.9;


BatC=45;% 电池容量 Ah
BatU=384;% 电池电压 V

Bat_eff=0.90;% 电池平均效率

Bat_Rohmic=0.0343;%Ohm 电池内阻

SOC_grid=zeros(1,length(ve));
SOC_dot_show=zeros(1,length(ve));
SOC_grid(1)=SOC_max;


EREV_MODE=0;
j=1;
i=1;
iter=1;
while true
    P_dem=Pw(i)*1000;
    if EREV_MODE==0
         if P_dem>=0
            SOC_dot=(sqrt(BatU^2-4*P_dem*Bat_Rohmic)-BatU)/(2*Bat_Rohmic*BatC*3600);%A.h-->s
         else
            SOC_dot=(sqrt(BatU^2-4*P_dem*0.2*Bat_Rohmic)-BatU)/(2*Bat_Rohmic*BatC*3600);%A.h-->s
         end
         SOC_grid(j+1)=SOC_grid(j)+SOC_dot;
         if SOC_grid(j+1)<0.25
             EREV_MODE=1;
         end
    else
        %开启增程器
        P_engine=30*1000;
        if P_dem>=0
            SOC_dot=(sqrt(BatU^2-4*(P_dem-P_engine)*0.5*Bat_Rohmic)-BatU)/(2*Bat_Rohmic*BatC*3600);%A.h-->s
        else
            SOC_dot=(sqrt(BatU^2-4*(P_dem*0.2-P_engine)*0.5*Bat_Rohmic)-BatU)/(2*Bat_Rohmic*BatC*3600);%A.h-->s
        end
        SOC_grid(j+1)=SOC_grid(j)+SOC_dot;
        if SOC_grid(j+1)>0.35
             EREV_MODE=0;
        end
    end


    j=j+1;
    i=i+1;

     %当完成一个循环时重置
    if i==1181
        i=1;
        iter=iter+1;
    end

    if iter==30
        break;
    end
end

figure 
plot(SOC_grid,'LineWidth',2,'Color','r','LineStyle','-')
