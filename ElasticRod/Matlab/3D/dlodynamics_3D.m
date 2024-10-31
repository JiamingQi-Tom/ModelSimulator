function [p_dat, PHI_dat, T_dat, para1] = dlodynamics_3D(STATE0, STATE1, Length, PHYSICAL, para0)
%% Definition of the global frame:
global Rf Rt Re D L
global n s0 s1 ds lx ly lz state0 state1
L = Length;
Rf = PHYSICAL(1);     % Flexural coefficient
Rt = PHYSICAL(2);     % Torsional coefficient
Re = PHYSICAL(3);     % extension coefficient
D = PHYSICAL(4);      % weight par m
N = 39;
s0 = 0;
s1 = L;    
ds = (s1 - s0) / N;
kmax = 2;             % Use 2nd order approximation
n = 2 * kmax + 2;     % number of parameters per varaible
%% left and right constraints
state0 = STATE0;
state1 = STATE1;
lx = state1(1) - state0(1);
ly = state1(2) - state0(2);
lz = state1(3) - state0(3);
ax = state1(4) - state0(4);
ay = state1(5) - state0(5);
az = state1(6) - state0(6);
%%
switch nargin
    case 4
        para0 = zeros(4*n, 1);
end
%% Computation
options = optimset('display','None');
para1 = fmincon(@costfun,para0,[],[],[],[],[],[],@nonlinc,options);
[p_dat, PHI_dat, T_dat] = plotDLO(para1);
end