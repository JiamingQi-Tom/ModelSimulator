clc;clear;close all
%%
cor = [1 1;
       2 1;
       2 2;
       1 2;
       1 1];

cor1 = cor * Rotate(60);
center = (cor(1,:) + cor(3,:)) / 2;
cor2 = (cor-center) * Rotate(60) + center;
cor3 = cor - center;
figure
plot(0,0,'r.','markersize',50);hold on
plot(cor(:,1),cor(:,2),'r-');hold on
plot(cor1(:,1),cor1(:,2),'b-');hold on
plot(cor2(:,1),cor2(:,2),'k-');hold on
plot(cor3(:,1),cor3(:,2),'k-');hold on

set(gca,'XLim',[-2 5])
set(gca,'YLim',[-2 5])
daspect([1 1 1])