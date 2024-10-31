clc;clear;close all
%% Constraints
state0 = [-0.1 -0.1 0 0 0 0]; % positon and orientation of left point
lx = 0.4;
ly = 0.4;
lz = -0.3;
ax = deg2rad(0);
ay = deg2rad(180);
az = deg2rad(0);
state1 = [state0(1)+lx state0(2)+ly state0(3)+lz state0(4)+ax state0(5)+ay state0(6)+az];
param0 = zeros(4*6, 1);
Length = 2;
%%
tic
[p_dat, PHI_dat, T_dat, param1] = dlodynamics_3D(state0,state1,Length,[1 1 0 0],param0);
toc
%%
figure
plot3(p_dat(:,1),p_dat(:,2),p_dat(:,3),'k.','linewidth',3,'markersize',20);hold on
grid on
daspect([1 1 1])
title('Euler Rotation is XYZ')
T_world = eye(4);
T_base_ur5 = [];
T_end_ur5 = [];
T_base_shape = T_dat(1:4,(1*4-3):(1*4));
T_end_shape = T_dat(1:4,(size(p_dat,1)*4-3):(size(p_dat,1)*4));
DrawAllFrame(T_world,T_base_ur5,T_end_ur5,T_base_shape,T_end_shape,1,1)
axis([-0.8 1 -0.8 1 -0.8 1])