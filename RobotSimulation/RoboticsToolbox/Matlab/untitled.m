clc;clear;close all
% %%
% %% 建立机器人模型
% %       theta    d        a        alpha     R/L
% % 定义连杆的D-H参数
% L1=Link([0       0       0.0       -pi/2      0     ]); 
% L2=Link([0       0.45    0.075     0.0        0     ]);
% L3=Link([-pi/2   0       0.84      0          0     ]);
% L4=Link([0       0       0.195     -pi/2      0     ]);
% L5=Link([0       1.1     0         pi/2      0     ]);
% L6=Link([0       0.0     0         -pi/2       0     ]);
% %连接连杆，机器人取名manman
% robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','manman'); 
% 
% view(3)
% % figure
% robot.teach()
% 
% % L1 = Link([0 12.4  0  pi/2 0 -pi/2 ]);
% % L2 = Link([0 0     0 -pi/2         ]);
% % L3 = Link([0 15.43 0  pi/2         ]);
% % L4 = Link([0 0     0 -pi/2 0  0    ]);
% % L5 = Link([0 15.92 0  pi/2         ]);
% % L6  =Link([0 0     0 -pi/2         ]);
% % L7 = Link([0 15    0  0    0  pi/2 ]);
% % robot = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','robot');

% xml = xmlread('fanucm10id8l.urdf');
% links = xml.getElementsByTagName('link');
% joints = xml.getElementsByTagName('joint');
% 
% for i = 0:joints.getLength-1
%     jointNode = joints.item(i);
%     theta = str2double(jointNode.getAttribute('theta')); % 可能需要调整以匹配实际属性名或使用子节点方式获取值
%     d = str2double(jointNode.getAttribute('d')); % 同上
%     a = str2double(jointNode.getAttribute('a')); % 同上
%     alpha = str2double(jointNode.getAttribute('alpha')); % 同上
%     fprintf('Joint %d: theta=%f, d=%f, a=%f, alpha=%f\n', i, theta, d, a, alpha);
% end


robot = importrobot('fanucm10id8l.urdf');
showdetails(robot)  % 显示机械臂结构信息
show(robot)  % 可视化机器人

config = homeConfiguration(robot);  % 获取默认关节角度配置
% T = getTransform(robot, config, 'wrist3_Link');  % 获取末端到基座的变换矩阵
% disp(T)

% 
for i = 1:length(robot.Bodies)
    body = robot.Bodies{i};
    T = body.Joint.JointToParentTransform;  % 关节的变换矩阵
    disp(['Joint ', num2str(i), ' Transform Matrix:'])
    disp(T)
end

dh_table = extractDH(robot);


