clc;clear;
addpath('C:\Users\user\Downloads\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

scenario = 1;

switch scenario
    case 1
        N = 100;
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

% Declare model variables
X = SX.sym('x', 6);
pos_x = X(1);
pos_y = X(2);
pos_h = X(3);
V = X(4);
psi = X(5);
m = X(6);

Xk = [0; 0; h0; V0; psi0; m0];

lat_lim = [35 60];
lon_lim = [-5 35];
lla = [];
h_lim = [0.5e4 1.5e4];
V_lim = [100 400];
m_lim = 1e4;

lbx = [35; -5; h_lim(1); V_lim(1); -inf; m_lim];
ubx = [60; 35; h_lim(2); V_lim(2); inf; m0];

T = MX.sym('T');

path_ang_max = deg2rad(1.5);
bank_ang_max = deg2rad(30);
thrt_lim = [0.2 1];
ubu = [path_ang_max; bank_ang_max; thrt_lim(2)];
lbu = [-path_ang_max; -bank_ang_max; thrt_lim(1)];
u0 = [0; 0; 1];

w = {};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g = {};
lbg = [];
ubg = [];

% Formulate the NLP
dt = T/N;

for k = 1:N
    Uk = MX.sym(['U_' num2str(k)], 3);
    w = {w{:}, Uk};
    lbw = [lbw; lbu];
    ubw = [ubw; ubu];
    w0 = [w0; u0];

    k1 = f(Xk, Uk(:), lla0);
    k2 = f(Xk + dt/2*k1, Uk(:), lla0);
    k3 = f(Xk + dt/2*k2, Uk(:), lla0);
    k4 = f(Xk + dt*k3, Uk(:), lla0);
    Xk = Xk + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    lla_i = lla2ned2([Xk(1) Xk(2)], lla0);
    lla = [lla; lla_i(1) lla_i(2)];
    Xkg = [lla_i(1); lla_i(2); Xk(3:end)];
    g = {g{:}, Xkg};

    if k ~= N
        lbg = [lbg; lbx];
        ubg = [ubg; ubx];
    else
        lbg = [lbg; [lla_final.'; -inf; -inf; -inf]];
        ubg = [ubg; [lla_final.'; inf; inf; inf]];
    end
end

w = vertcat(w{:});
g = vertcat(g{:});

tf_guess = 11e3;
w = [w; T];
lbw = [lbw; 0];
ubw = [ubw; inf];
w0 = [w0; tf_guess];

L = 0.05*T + (m0 - Xk(end));

options = struct;
options.ipopt.max_iter = 10000;
options.ipopt.print_level = 5;
options.ipopt.tol = 1e-6;
options.ipopt.constr_viol_tol = 1e-4;

prob = struct('f', L, 'x', w, 'g', g);
solver = nlpsol('solver', 'ipopt', prob, options);

sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);


%%

% Plot the solution
u_opt = w_opt(1:end-1);
t_opt = w_opt(end);
dt = t_opt/N;
Xk = [0 ;0 ;h0 ;V0 ;psi0 ;m0];
x_opt = Xk;
for k=1:N
    % Integrate till the end of the interval
    k1 = f(x_opt,         u_opt((k-1)*3 + 1:k*3),lla0);
    k2 = f(x_opt+dt/2*k1, u_opt((k-1)*3 + 1:k*3),lla0);
    k3 = f(x_opt+dt/2*k2, u_opt((k-1)*3 + 1:k*3),lla0);
    k4 = f(x_opt+dt*k3,   u_opt((k-1)*3 + 1:k*3),lla0);
    x_opt = [x_opt x_opt(:,end) + dt/6*(k1+2*k2+2*k3+k4)];
end

%%
%clc;close all
%addpath('C:\Users\PC_4236\Desktop\old_pc\downloads\casadi-3.6.3-windows64-matlab2018b')

%% plotting path of aicraft 3D
switch scenario

    case 1
        %scenerio initial and final conditions
        lla0 = [40 5 8000];
        lla_final = [40 32 8000];
        m0 = 68e3;

    case 2
        lla0 = [55 30 7000];
        lla_final = [40 15 9000];
        m0 = 67e3;

    case 3
        lla0 = [45 32 8000];
        lla_final = [45 5 7000];
        m0 = 65e3;
end

opti_pos_x   = x_opt(1,:);
opti_pos_y   = x_opt(2,:);
opti_pos_h   = x_opt(3,:);
opti_V       = x_opt(4,:);
opti_psi     = x_opt(5,:);
opti_m       = x_opt(6,:);

u_opt = reshape(u_opt,[3 100]);

opti_path_ang = u_opt(1,:);
opti_bank_ang = u_opt(2,:);
opti_thrt     = u_opt(3,:);

opti_T = t_opt;

lla = [];
for i=1:length(opti_pos_x)
    lla_i = lla2ned2([opti_pos_x(i) opti_pos_y(i)],lla0);
    lla = [lla ; lla_i(1) lla_i(2)];
end
%%

figure(1)
plot3(lla(:,1),lla(:,2),opti_pos_h,'b','LineWidth',2);
hold on 
plot3(lla0(1),lla0(2),lla0(3),'ro','MarkerSize',10,'LineWidth',1);
plot3(lla_final(1),lla_final(2),lla_final(3),'go','MarkerSize',10,'LineWidth',1);

grid minor 

xlabel('lat(deg)','FontSize',15)
ylabel('lon(deg)','FontSize',15)
zlabel('h(m)','FontSize',15)
xlim([35 60])
ylim([-5 35]) 
title('Path of Aircraft 3D','FontSize',20)
legend('Aircraft Path','Start','Goal')
axis ij

%% plotting path of aicraft 2D

figure(2)

subplot(1,3,1)
plot(lla(:,2),lla(:,1),'b')
xlabel('lon(deg)')
ylabel('lat(deg)')
title('lat vs lon')
grid minor

subplot(1,3,2)
plot(lla(:,2),opti_pos_h,'b')
xlabel('lon(deg)')
ylabel('h(m)')
title('lon vs h')
grid minor

subplot(1,3,3)
plot(lla(:,1),opti_pos_h,'b')
xlabel('lat(deg)')
ylabel('h(m)')
title('lat vs h')
grid minor


%% Turning initial phase
figure(3)
plot3(lla(1:N/100,1),lla(1:N/100,2),opti_pos_h(1:N/100),'b');
xlabel('lat(deg)')
ylabel('lon(deg)')
zlabel('h(m)')
title('Turning maneuver at the beginning',FontSize=20)
axis ij
grid minor

%% Plotting Control Inputs 
% figure(4)
% title('mean')
% t = linspace(1,opti.debug.value(T),N);
% opti_inputs = [opti.debug.value(path_ang) ; opti.debug.value(bank_ang) ; opti.debug.value(thrt)];
% 
% subplot(3,1,1)
% for i=1:length(opti_inputs(1,:))
%     mean_path_angs(i) = mean(opti_inputs(1,1:i)); 
% end
% stairs(t,rad2deg(mean_path_angs),'LineWidth',0.02);
% title('Mean Flight Path Angle(deg) vs t')
% 
% subplot(3,1,2)
% for i=1:length(opti_inputs(2,:))
%     mean_bank_angs(i) = mean(opti_inputs(2,1:i)); 
% end
% stairs(t,rad2deg(mean_bank_angs));
% title('Mean Bank Angle vs t')
% 
% subplot(3,1,3)
% for i=1:length(opti_inputs(3,:))
%     mean_thrt(i) = mean(opti_inputs(3,1:i)); 
% end
% stairs(t,mean_thrt);
% title('Mean Throttle lever vs t')
% grid minor


figure(4)
title('actual')
t = linspace(1,opti_T,N);
opti_inputs = [opti_path_ang ; opti_bank_ang ; opti_thrt];

subplot(3,1,1)
stairs(t,rad2deg(opti_inputs(1,:)),'LineWidth',2);
title('Flight Path Angle vs t')
grid minor
xlabel('t(s)')
ylabel('\gamma(deg)',FontSize=10,Rotation=0)

subplot(3,1,2)
stairs(t,rad2deg(opti_inputs(2,:)),'LineWidth',2);
title('Bank Angle vs t')
grid minor
xlabel('t(s)')
ylabel('\mu(deg)',FontSize=10,Rotation=0)


subplot(3,1,3)
stairs(t,opti_inputs(3,:),'LineWidth',2);
title('Thrust lever pos. vs t')
grid minor
xlabel('t(s)')
ylabel('\delta',FontSize=10,Rotation=0,HorizontalAlignment='right',VerticalAlignment='bottom')

%% Plotting V, heading and mass of aircraft

figure(5)

t = linspace(1,opti_T,N+1);
subplot(3,1,1)
plot(t,opti_V,'LineWidth',2);
title('V vs t')
grid minor
ylabel('V(m/s)')
xlabel('t(s)')

subplot(3,1,2)
plot(t,rad2deg(opti_psi),'LineWidth',2);
title('heading vs t')
grid minor
ylabel('heading(deg)')
xlabel('t(s)')

subplot(3,1,3)
plot(t,opti_m,'LineWidth',2);
title('Mass vs t')
grid minor
ylabel('Mass(kg)')
xlabel('t(s)')

%% Printing necessery value
L = 0.05*opti_T + (m0-opti_m(end));
fprintf(' Cost function value = %f \n ',L)
fprintf('T final = %f s \n ',opti_T)
fprintf('Total fuel consumpion = %f \n ',(m0 - opti_m(end)))

%% Saving optimal output as inital guess
opti_pos_x = opti.debug.value(pos_x);
opti_pos_y =  opti.debug.value(pos_y);
opti_pos_h =  opti.debug.value(pos_h);
opti_V =  opti.debug.value(V);
opti_psi =  opti.debug.value(psi);
opti_m =  opti.debug.value(m);

opti_inputs = [opti.debug.value(path_ang) ; opti.debug.value(bank_ang) ; opti.debug.value(thrt)];
opti_U = opti_inputs;
opti_X = [opti_pos_x ; opti_pos_y ; opti_pos_h ; opti_V ; opti_psi ; opti_m];

opt_sol.opti_U = opti_U;
opt_sol.opti_X = opti_X;
opt_sol.T = opti.debug.value(T);



