clc;clear;close all
%% 建立机器人模型
%
d1 = 89.459/1000;
d2 = 0;
d3 = 0;
d4 = 109.15/1000;
d5 = 94.65/1000;
d6 = 82.3/1000;
a1 = 0;
a2 = -425/1000; 
a3 = -392.25/1000;
a4 = 0;
a5 = 0;
a6 = 0;
alpha1=pi/2;
alpha2=0;
alpha3=0;
alpha4=pi/2;
alpha5=-pi/2;
alpha6=0;

%          theta    d       a       alpha   R/L
L1 = Link([0 d1 a1 alpha1 0],'standard'); %定义连杆的D-H参数
L2 = Link([0 d2 a2 alpha2 0],'standard');
L3 = Link([0 d3 a3 alpha3 0],'standard');
L4 = Link([0 d4 a4 alpha4 0],'standard');
L5 = Link([0 d5 a5 alpha5 0],'standard');
L6 = Link([0 d6 a6 alpha6 0],'standard');

L1.qlim = [-360 360]*pi/180;
L2.qlim = [-360 360]*pi/180;
L3.qlim = [-360 360]*pi/180;
L4.qlim = [-360 360]*pi/180;
L5.qlim = [-360 360]*pi/180;
L6.qlim = [-360 360]*pi/180;

robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','UR5'); %连接连杆，机器人取名manman
q = deg2rad([0 -90 0 -90 0 0]);
robot.fkine(q)
robot.plot(q);
robot.teach
grid off

%%

%init model
demo_ur5
%init param
step=10;
init_ang = rand(1,6);
targ_ang = rand(1,6);
T0 = robot.fkine(init_ang);%forward solution
T1 = robot.fkine(targ_ang);%inverse solution
Tc = ctraj(T0,T1,step);    %get line matrix
tt = transl(Tc);           %get xyz position vector
q  = robot.ikine(Tc);      %inverse solution with Tc
tt1=[0 0 0];                

for i=1:length(q)
    T=robot.fkine(q(i,:)); %给定关节角,得到位姿4x4矩阵
    tt1_=transl(T);
    if i==1
        tt1=tt1_;
    else
        tt1=[tt1;tt1_];
    end
    
    robot.plot(q(i,:)); 
    plot2(T.t(1:3)','b.');hold on
    plot2(Tc(i).t(1:3)','r.');hold on
end

figure 
plot3(tt(:,1),tt(:,2),tt(:,1),'r.');hold on
plot3(tt1(:,1),tt1(:,2),tt1(:,1),'b.');
legend('desired','actual')

err=tt-tt1;

