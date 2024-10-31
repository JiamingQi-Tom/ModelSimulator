function [centerline, contour, para1] = dlodynamics_2D(state0, state1, length, para0, format)
%% global
global L n N Lx Ly Theta1 Theta2
L = length;
n = 4;
N = 49;
Lx = state1(1) - state0(1);       
Ly = state1(2) - state0(2);
Theta1 = state0(3);
Theta2 = state1(3);

if isempty(para0)
    para0 = zeros(2 * 4 + 2, 1);
end

% format = 1 rod simulation 
% format = 2 area simulation
if isempty(format)
    format = 1;
end

A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
options = optimset('LargeScale','off','display','none');
[para1, fval] = fmincon(@costfcn,para0,A,b,Aeq,beq,lb,ub,@nlinconfun,options);
px = state0(1);
py = state0(2);
DLO = [px,py];  % This gives the position left end
DLOangle = 0;   % This gives the left angle
numOfData = N;
for k = 1 : numOfData
    phi = para1(1) + para1(2) * L * k / numOfData;
    for i = 1 : n
        phi = phi + para1(2 * i + 1) * sin(2 * pi * i * k / numOfData) + para1(2 * i + 2) * cos(2 * pi * i * k / numOfData);
    end
    px = px + cos(phi) * L / numOfData;
    py = py + sin(phi) * L / numOfData;
    DLO = [DLO;px, py];
    DLOangle = [DLOangle; phi];
end

if format == 1
    centerline = DLO;
    contour = [];
else
    radius = 0.03;
    for i=2:N
        if DLOangle(i) > 0
            theta = -(pi/2 - DLOangle(i));
            upper(i,1) = DLO(i,1) - radius * cos(theta);
            upper(i,2) = DLO(i,2) - radius * sin(theta);
        
            lower(i,1) = DLO(i,1) + radius * cos(theta);
            lower(i,2) = DLO(i,2) + radius * sin(theta);
        else
            theta = pi/2 + DLOangle(i);
            upper(i,1) = DLO(i,1) + radius * cos(theta);
            upper(i,2) = DLO(i,2) + radius * sin(theta);
            
            lower(i,1) = DLO(i,1) - radius * cos(theta);
            lower(i,2) = DLO(i,2) - radius * sin(theta);
        end
    end
    lower = flipud(lower);
    centerline = DLO;
    contour = [DLO(1,:);upper(2:N,:);DLO(N + 1,:);lower(1:(N - 1),:)];
end
end