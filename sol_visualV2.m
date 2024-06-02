clc;close all
%addpath('C:\Users\PC_4236\Desktop\old_pc\downloads\casadi-3.6.3-windows64-matlab2018b')

%% plotting path of aicraft 3D
N = length(opt_sol.opti_U(1,:));
scenario = 2;
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

opti_pos_x   = opt_sol.opti_X(1,:);
opti_pos_y   = opt_sol.opti_X(2,:);
opti_pos_h   = opt_sol.opti_X(3,:);
opti_V       = opt_sol.opti_X(4,:);
opti_psi = opt_sol.opti_X(5,:);
opti_m       = opt_sol.opti_X(6,:);

opti_path_ang = opt_sol.opti_U(1,:);
opti_bank_ang = opt_sol.opti_U(2,:);
opti_thrt =     opt_sol.opti_U(3,:);

opti_T = opt_sol.T;


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

