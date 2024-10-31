clc;clear;close all
%%
mc = mobile_car(0.15,0.08,0.08,[0 0]);
N = 20;
theta = linspace(0,2*pi,100);
x1 = 0.1;
y1 = 0;
x2 = 0.8;
y2 = 0.0;
Theta1 = deg2rad(-320);
Theta2 = deg2rad(0);
para0 = zeros(2 * 4 +2, 1);
cable_length = 1;
for i=1:80
    if i>=1 && i< 20
        u1 = 0.002;
        u2 = 0.000;
    elseif i >= 20 && i< 40
        u1 = 0.01;
        u2 = 0.0;
    elseif i>= 40 && i < 60
        u1 = 0.002;
        u2 = 0.006;
    else
        u1 = 0.01;  
        u2 = 0.0;
    end
    
    mc.control([u1 u2]);
    x1 = mc.car_center(1);
    y1 = mc.car_center(2);
    Theta1 = mc.current_rotation_angle;
    [centerline,~,para1] = dlodynamics_2D(x1, y1, x2, y2,Theta1, Theta2, cable_length, para0, 1);
    para0 = para1;
    
    mc.plot();
    plot(centerline(:, 1), centerline(:, 2),'k','linewidth',3,'Markersize',3,'MarkerEdgeColor', 'k');
    hold off
    
    pause(0.1)
end
mc.plot();
%%