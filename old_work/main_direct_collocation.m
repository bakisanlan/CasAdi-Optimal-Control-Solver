clc;clear;
addpath('C:\Users\user\Downloads\casadi-3.6.5-windows64-matlab2018b')

scenario = 1;

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
        N = 3200;
        lla0 = [55 30 7000];
        V0 = 220;
        h0 = lla0(3);
        psi0 = deg2rad(40);
        m0 = 67e3;
        lla_final = [40 15 9000];
    case 3
        N = 3200;
        lla0 = [45 32 8000];
        V0 = 210;
        h0 = lla0(3);
        psi0 = deg2rad(180);
        m0 = 65e3;
        lla_final = [45 5 7000];
end


% xyzNED_final = lla2ned(lla_final,lla0,"flat");
% pos_x_final = xyzNED_final(1);
% pos_y_final = xyzNED_final(2);
% pos_h_final = lla0(3);

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

h = T/N; % length of a control interval
for k=1:N-1 % loop over control intervals
   % % Runge-Kutta 4 integration
   % k1 = f(X(:,k),         U(:,k),lla0);
   % k2 = f(X(:,k)+dt/2*k1, U(:,k),lla0);
   % k3 = f(X(:,k)+dt/2*k2, U(:,k),lla0);
   % k4 = f(X(:,k)+dt*k3,   U(:,k),lla0);
   % x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   % 
   % %euler integration
   % %x_next = X(:,k) + dt*f(X(:,k), U(:,k),lla0);
   % opti.subject_to(X(:,k+1)==x_next); % close the gaps

   Xc = 0.5*( X(:,k) + X(:,k+1) ) + (h/8)*( f(X(:,k),U(:,k),lla0) - f(X(:,k+1),U(:,k+1),lla0) );
   Xc_dot = (-3/(2*h))*( X(:,k) - X(:,k+1) ) -0.25*( f(X(:,k),U(:,k),lla0) + f(X(:,k+1),U(:,k+1),lla0) );
   Xc_dyn = f(Xc, 0.5*(U(:,k) + U(:,k+1)),lla0);
   opti.subject_to(Xc_dyn==Xc_dot); % close the gaps

end

% ---- state constraints -----------
lat_lim = [35 60];
lon_lim = [-5 35];
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
% tf_guess = 2.3e6/V0;  %s
% t = linspace(0,tf_guess,N+1);
% opti.set_initial(pos_x, 0);
% opti.set_initial(pos_y, 240*t);
% opti.set_initial(pos_h, 12000);
% opti.set_initial(V, 240);
% opti.set_initial(psi, deg2rad(90));
% opti.set_initial(m, m0);
% 
opti.set_initial(path_ang, 0);
opti.set_initial(bank_ang, 0);
opti.set_initial(thrt, 1);

% % % % setting initial guess for x and input with optimal solution
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







