clc;clear;close all
%%
robot = robot_build();
%% task
% 根据给定起始点，得到起始点位姿
T1 = transl(0.5,0,0);
% 根据给定终止点，得到终止点位姿
T2 = transl(0,0.5,0);
% 根据起始点位姿，得到起始点关节角
q1 = robot.ikine(T1);
% 根据终止点位姿，得到终止点关节角
q2 = robot.ikine(T2);
% 五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
[q ,qd, qdd] = jtraj(q1,q2,50);
% 根据插值，得到末端执行器位姿
T = robot.fkine(q);
%%
figure('Color',[1 1 1])
view(3);
robot.display();
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));hold on;grid on
robot.plot(q);