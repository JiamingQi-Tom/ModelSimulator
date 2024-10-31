clc;clear;close all
%%
state0 = [0.1 0 deg2rad(45)];
state1 = [0.5 0 deg2rad(-45)];
tic
[centerline,contour,~] = dlodynamics_2D(state0,state1,1,[],2);
toc
%%
figure
plot(centerline(:, 1), centerline(:, 2),'ob-','linewidth',2,'Markersize',3,'MarkerEdgeColor', 'k','MarkerFaceColor', [254, 67, 101]/255);
axis([-0.2 0.7 -0.2 0.7])
daspect([1 1 1])
%%
figure
fill(contour(:,1),contour(:,2),'r');hold on
axis([-0.2 0.7 -0.2 0.7])
daspect([1 1 1])
polyarea(contour(:,1),contour(:,2))
