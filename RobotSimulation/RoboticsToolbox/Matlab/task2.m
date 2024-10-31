clc;clear;close all
%%
robot = robot_build();
%%
T1 = transl(0.5,0.4,0.3);  % ���ݸ�����ʼ�㣬�õ���ʼ��λ��
T2 = transl(-0.7,-0.6,0.1);% ���ݸ�����ֹ�㣬�õ���ֹ��λ��
q1 = robot.ikine(T1);      % ������ʼ��λ�ˣ��õ���ʼ��ؽڽ�
q2 = robot.ikine(T2);      % ������ֹ��λ�ˣ��õ���ֹ��ؽڽ�
%%
length = 100;
QDO = 0.1*ones(1,6);
QD1 = 0.1*ones(1,6);
% ��ζ���ʽ�켣���õ��ؽڽǶȣ����ٶȣ��Ǽ��ٶ�
[q,dq,ddq] = jtraj(q1,q2,length,QDO,QD1); 
% �����ؽڽ�,�õ�λ��4x4����
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
