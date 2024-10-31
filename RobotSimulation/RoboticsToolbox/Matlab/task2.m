clc;clear;close all
%%
robot = robot_build();
%%
T1 = transl(0.5,0.4,0.3);  % 根据给定起始点，得到起始点位姿
T2 = transl(-0.7,-0.6,0.1);% 根据给定终止点，得到终止点位姿
q1 = robot.ikine(T1);      % 根据起始点位姿，得到起始点关节角
q2 = robot.ikine(T2);      % 根据终止点位姿，得到终止点关节角
%%
length = 100;
QDO = 0.1*ones(1,6);
QD1 = 0.1*ones(1,6);
% 五次多项式轨迹，得到关节角度，角速度，角加速度
[q,dq,ddq] = jtraj(q1,q2,length,QDO,QD1); 
% 给定关节角,得到位姿4x4矩阵
T = robot.fkine(q); 
%%
figure('Color',[1 1 1])
view(3);
robot.display();
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));hold on;grid on
robot.plot(q);
%%
figure
subplot(3,1,1)
plot(q(:,1:6));
subplot(3,1,2)
plot(dq(:,1:6));
subplot(3,1,3)
plot(ddq(:,1:6));
