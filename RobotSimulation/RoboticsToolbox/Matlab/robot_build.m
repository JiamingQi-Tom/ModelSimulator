function [robot] = robot_build()
%% 建立机器人模型
%       theta    d        a        alpha     R/L
% 定义连杆的D-H参数
L1=Link([0       0.4      0.025    pi/2      0     ]); 
L2=Link([pi/2    0        0.56     0         0     ]);
L3=Link([0       0        0.035    pi/2      0     ]);
L4=Link([0       0.515    0        pi/2      0     ]);
L5=Link([pi      0        0        pi/2      0     ]);
L6=Link([0       0.08     0        0         0     ]);
%连接连杆，机器人取名manman
robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','manman'); 

% L1 = Link([0 12.4  0  pi/2 0 -pi/2 ]);
% L2 = Link([0 0     0 -pi/2         ]);
% L3 = Link([0 15.43 0  pi/2         ]);
% L4 = Link([0 0     0 -pi/2 0  0    ]);
% L5 = Link([0 15.92 0  pi/2         ]);
% L6  =Link([0 0     0 -pi/2         ]);
% L7 = Link([0 15    0  0    0  pi/2 ]);
% robot = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','robot');
end


