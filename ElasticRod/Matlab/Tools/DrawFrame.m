function DrawFrame(T, linelength, linewidth)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw a 3D frame associated with the 
% homogeneous transformation T
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[m,n]=size(T);
if m ~= 4,
    disp('Invalid dimension of T');
    return;
end
if n ~= 4,
    disp ('Invalid dimension of T');
    return;
end


%s=50;
x(1) = T(1,4);
y(1) = T(2,4);
z(1) = T(3,4);
h1 = plot3(x,y,z);hold on
set(get(get(h1,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

k = 1;
x(2) = x(1) + linelength*T(1,k);
y(2) = y(1) + linelength*T(2,k);
z(2) = z(1) + linelength*T(3,k);
h2 = plot3(x, y, z, 'LineWidth', linewidth,'color','#FF0000');
% text(x(2),y(2),z(2),'x','fontsize',22)
set(get(get(h2,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

k = 2;
x(2) = x(1) + linelength*T(1,k);
y(2) = y(1) + linelength*T(2,k);
z(2) = z(1) + linelength*T(3,k);
h3 = plot3(x, y, z, 'LineWidth', linewidth,'Color','#008000');hold on
% text(x(2),y(2),z(2),'y','fontsize',22)
set(get(get(h3,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

k = 3;
x(2) = x(1) + linelength*T(1,k);
y(2) = y(1) + linelength*T(2,k);
z(2) = z(1) + linelength*T(3,k);
h4 = plot3(x, y, z, 'LineWidth', linewidth,'Color','#0000FF');hold on
% text(x(2),y(2),z(2),'z','fontsize',22)
set(get(get(h4,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
end