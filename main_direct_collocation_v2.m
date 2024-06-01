clc;clear;
addpath('C:\Users\PC_4236\Desktop\old_pc\downloads\casadi-3.6.3-windows64-matlab2018b')
import casadi.*

scenario = 1;
lat_lim = [35 60];
lon_lim = [-5 35];
h_lim = [0.5e4 1.5e4];

switch scenario

    case 1
        %scenerio initial and final conditions
        N = 400;
        lla0 = [40 5 8000];
        V0 = 210;
        h0 = lla0(3);
        psi0 = 0;
        m0 = 68e3;
        lla_final = [40 32 8000];

    case 2
        N = 100;
        lla0 = [55 30 7000];
        V0 = 220;
        h0 = lla0(3);
        psi0 = deg2rad(40);
        m0 = 67e3;
        lla_final = [40 15 9000];
    case 3
        N = 150;
        lla0 = [45 32 8000];
        V0 = 210;
        h0 = lla0(3);
        psi0 = deg2rad(180);
        m0 = 65e3;
        lla_final = [45 5 7000];
end

% Degree of interpolating polynomial
d = 3;

% Get collocation points
tau = collocation_points(d, 'legendre');

% Collocation linear maps
[C,D,B] = collocation_coeff(tau);

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(6,N+1); % state trajectory
pos_x = X(1,:);
pos_y = X(2,:);
pos_h = X(3,:);
V     = X(4,:);
psi   = X(5,:);
m     = X(6,:);

U = opti.variable(3,N);   % control trajectory
path_ang = U(1,:);
bank_ang = U(2,:);
thrt     = U(3,:);

T = opti.variable();      % final time

% ---- objective          ---------
L = 0.05*T + (m0-m(end));
opti.minimize(L); % race in minimal time

%Xc = {};
t = linspace(0,2.3e6/V0,N+1);

h = T/N; % length of a control interval
for k=1:N % loop over control intervals

    % Xc{k} = opti.variable(6, d);
    % 
    % % for i=1:d
    % % 
    % %     Xck_dot(:,i) = f(Xc{k}, U(:,k),lla0);
    % % 
    % % end
    % 
    % Xck_dot1 = f(Xc{k}(:,1), U(:,k),lla0);
    % Xck_dot2 = f(Xc{k}(:,2), U(:,k),lla0);
    % Xck_dot3 = f(Xc{k}(:,3), U(:,k),lla0);
    % 
    % 
    % Z = [X(:,k) Xc{k}];
    % 
    % Pidot = Z*C;
    % 
    % opti.subject_to(Pidot(:,1) == h*Xck_dot1);    
    % opti.subject_to(Pidot(:,2) == h*Xck_dot2);
    % opti.subject_to(Pidot(:,3) == h*Xck_dot3);
    % 
    % Xk_end = Z*D;
    % 
    % opti.subject_to(Xk_end==X(:,k+1));

    Xc = opti.variable(6,d);

    opti.set_initial(Xc, repmat([0;240*t(k);12000;240;deg2rad(90);m0],1,d));
    
    for dd=1:d
    lla_xcdk = lla2ned2([Xc(1,d) Xc(2,d)],lla0);
    opti.subject_to(lat_lim(1) < lla_xcdk(1) < lat_lim(2));
    opti.subject_to(lon_lim(1) < lla_xcdk(2) < lon_lim(2)); 
    opti.subject_to(h_lim(1) < Xc(3,d) < h_lim(2));
    opti.subject_to(100 <= Xc(4,d) <= 400);
    opti.subject_to(Xc(6,d) >= 1e4);
    end
    Xck_dot1 = f(Xc(:,1), U(:,k),lla0);
    Xck_dot2 = f(Xc(:,2), U(:,k),lla0);
    Xck_dot3 = f(Xc(:,3), U(:,k),lla0);

    Z = [X(:,k) Xc];

    Pidot = Z*C;

    opti.subject_to(Pidot(:,1) == h*Xck_dot1);    
    opti.subject_to(Pidot(:,2) == h*Xck_dot2);
    opti.subject_to(Pidot(:,3) == h*Xck_dot3);

    Xk_end = Z*D;

    opti.subject_to(Xk_end==X(:,k+1));
end

% ---- state constraints -----------

lla = [];
for i=1:N+1
    lla_i = lla2ned2([pos_x(i) pos_y(i)],lla0);
    lla = [lla ; lla_i(1) lla_i(2)];
end
h_lim = [0.5e4 1.5e4];
opti.subject_to(lat_lim(1) < lla(:,1) < lat_lim(2));
opti.subject_to(lon_lim(1) < lla(:,2) < lon_lim(2)); 
opti.subject_to(h_lim(1) < pos_h < h_lim(2));
opti.subject_to(100 <= V <= 400);
opti.subject_to(m >= 1e4);

% ---- input constraints -----------
path_ang_max = deg2rad(1.5);
bank_ang_max = deg2rad(30);
thrt_lim = [0.2 1];
opti.subject_to(-path_ang_max <= path_ang <= path_ang_max);
opti.subject_to(-bank_ang_max <= bank_ang <= bank_ang_max);
opti.subject_to(thrt_lim(1) <= thrt <= thrt_lim(2));

% ---- boundary conditions --------
opti.subject_to(lla(1,1)==lla0(1));   
opti.subject_to(lla(N+1,1)==lla_final(1));   
opti.subject_to(lla(1,2)==lla0(2));   
opti.subject_to(lla(N+1,2)==lla_final(2)); 
opti.subject_to(pos_h(1)==h0);   
opti.subject_to(pos_h(N+1)==h0);   
opti.subject_to(V(1)==V0);   
opti.subject_to(psi(1)==psi0);   
opti.subject_to(m(1)==m0);   

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

%---- initial values for solver ---
tf_guess = 2.3e6/V0;  %s
t = linspace(0,tf_guess,N+1);
opti.set_initial(pos_x, 0);
opti.set_initial(pos_y, 240*t);
opti.set_initial(pos_h, 12000);
opti.set_initial(V, 240);
opti.set_initial(psi, deg2rad(90));
opti.set_initial(m, m0);

opti.set_initial(path_ang, 0);
opti.set_initial(bank_ang, 0);
opti.set_initial(thrt, 1);

% % setting initial guess for x and input with optimal solution
% load('opti_guess_sc1_800N.mat');
% opti.set_initial(pos_x, opti_X(1,:));
% opti.set_initial(pos_y, opti_X(2,:));
% opti.set_initial(pos_h, opti_X(3,:));
% opti.set_initial(V,     opti_X(4,:));
% opti.set_initial(psi,   opti_X(5,:));
% opti.set_initial(m,     opti_X(6,:));

% opti.set_initial(path_ang, opti_inputs(1,:));
% opti.set_initial(bank_ang, opti_inputs(2,:));
% opti.set_initial(thrt,     opti_inputs(3,:));

tf_guess = 11e3;
opti.set_initial(T, tf_guess);

% ---- solve NLP              ------
p_opts = struct('expand',true);
s_opts = struct('max_iter',10000);
opti.solver('ipopt',p_opts,s_opts); % set numerical backend
sol = opti.solve();   % actual solve







