clc;
%addpath('C:\Users\PC_4236\Desktop\old_pc\downloads\casadi-3.6.3-windows64-matlab2018b')

%% plotting path of aicraft 3D
figure(1)
        lla0 = [40 5 8000];

opti_pos_x = opti.debug.value(pos_x);
opti_pos_y = opti.debug.value(pos_y);
lla = [];
for i=1:length(opti.debug.value(pos_x))
    lla_i = lla2ned2([opti_pos_x(i) opti_pos_y(i)],lla0);
    lla = [lla ; lla_i(1) lla_i(2)];
end

plot3(lla(:,1),lla(:,2),opti.debug.value(pos_h(:)));
grid minor

xlabel('lat(deg)','FontSize',15)
ylabel('lon(deg)','FontSize',15)
zlabel('h(m)','FontSize',15)
xlim([35 60])
ylim([-5 35]) 
title('Path of Aircraft 3D','FontSize',20 )
axis ij

%% plotting path of aicraft 2D

figure(2)

subplot(1,3,1)
plot(lla(:,2),lla(:,1))
xlabel('lon(deg)')
ylabel('lat(deg)')
title('lat vs lon')
grid minor

subplot(1,3,2)
plot(lla(:,2),opti.debug.value(pos_h(:)))
xlabel('lon(deg)')
ylabel('h(m)')
title('lon vs h')
grid minor

subplot(1,3,3)
plot(lla(:,1),opti.debug.value(pos_h(:)))
xlabel('lat(deg)')
ylabel('h(m)')
title('lat vs h')
grid minor



%% Turning initial phase
figure(3)
plot3(lla(1:N/100,1),lla(1:N/100,2),opti.debug.value(pos_h(1:N/100)));
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
t = linspace(1,opti.debug.value(T),N);
opti_inputs = [opti.debug.value(path_ang) ; opti.debug.value(bank_ang) ; opti.debug.value(thrt)];

subplot(3,1,1)
stairs(t,rad2deg(opti_inputs(1,:)),'LineWidth',0.02);
title('Flight Path Angle vs t')
grid minor
xlabel('t(s)')
ylabel('\gamma(deg)',FontSize=10,Rotation=0)

subplot(3,1,2)
stairs(t,rad2deg(opti_inputs(2,:)));
title('Bank Angle vs t')
grid minor
xlabel('t(s)')
ylabel('\mu(deg)',FontSize=10,Rotation=0)


subplot(3,1,3)
stairs(t,opti_inputs(3,:));
title('Thrust lever pos. vs t')
grid minor
xlabel('t(s)')
ylabel('\delta',FontSize=10,Rotation=0,HorizontalAlignment='right',VerticalAlignment='bottom')

%% Plotting V, heading and mass of aircraft

figure(5)

t = linspace(1,opti.debug.value(T),N+1);
subplot(3,1,1)
plot(t,opti.debug.value(V))
title('V vs t')
grid minor
ylabel('V(m/s)')
xlabel('t(s)')

subplot(3,1,2)
plot(t,rad2deg(opti.debug.value(psi)))
title('heading vs t')
grid minor
ylabel('heading(deg)')
xlabel('t(s)')

subplot(3,1,3)
plot(t,opti.debug.value(m))
title('Mass vs t')
grid minor
ylabel('Mass(kg)')
xlabel('t(s)')

%% Printing necessery value

fprintf(' Cost function value = %f \n ',opti.debug.value(L))
fprintf('T final = %f s \n ',opti.debug.value(T))
fprintf('Total fuel consumpion = %f \n ',(m0 - opti.debug.value(m(end))))

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

