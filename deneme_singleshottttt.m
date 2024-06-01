clc;clear;
addpath('C:\Users\user\Downloads\casadi-3.6.5-windows64-matlab2018b')
import casadi.*

scenario = 1;

switch scenario

    case 1
        %scenerio initial and final conditions
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
%%
% Declare model variables
% x1 = SX.sym('x1');
% x2 = SX.sym('x2');
% x = [x1; x2];
% u = SX.sym('u');
X = SX.sym('x',6);

pos_x = X(1);
pos_y = X(2);
pos_h = X(3);
V     = X(4);
psi   = X(5);
m     = X(6);

Xk = [0 ;0 ;h0 ;V0 ;psi0 ;m0];

lat_lim = [35 60];
lon_lim = [-5 35];
lla = [];
h_lim = [0.5e4 1.5e4];
V_lim = [100 400];
m_lim = 1e4;

lbx = [35 ; -5 ; h_lim(1) ; V_lim(1); -inf ; m_lim];
ubx = [60 ; 35 ; h_lim(2) ; V_lim(2);  inf ; m0  ];


T = MX.sym('T');

% Start with an empty NLP
path_ang_max = deg2rad(1.5);
bank_ang_max = deg2rad(30);
thrt_lim = [0.2 1];
ubu = [path_ang_max ; bank_ang_max ; thrt_lim(2)];
lbu = [-path_ang_max ; -bank_ang_max ; thrt_lim(1)];
u0 = [0 ; 0 ; 1];


w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

% Formulate the NLP
dt = T/N; % length of a control interval

for k=1:N
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)],3);
    w = {w{:} Uk};
    lbw = [lbw ; lbu];
    ubw = [ubw ;  ubu];
    w0 = [w0 ; u0];

    % Integrate till the end of the interval
    k1 = f(Xk,         Uk(:),lla0);
    k2 = f(Xk+dt/2*k1, Uk(:),lla0);
    k3 = f(Xk+dt/2*k2, Uk(:),lla0);
    k4 = f(Xk+dt*k3,   Uk(:),lla0);
    Xk = Xk + dt/6*(k1+2*k2+2*k3+k4);

    % Add inequality constraint
    lla_i = lla2ned2([Xk(1) Xk(2)],lla0);
    lla = [lla ; lla_i(1) lla_i(2)];
    Xkg = [lla_i(1) ; lla_i(2) ; Xk(3:end)];
    g = {g{:} Xkg};

    if k ~= N
        lbg = [lbg; lbx];
        ubg = [ubg; ubx];
    else
        % boundary condition
        lbg = [lbg; [lla_final.' ; -inf ; -inf ; -inf]];
        ubg = [ubg; [lla_final.' ;  inf ;  inf ;  inf]];
    end


end

% Concatenate decision variables and constraint terms
w = vertcat(w{:});
g = vertcat(g{:});

% Add T to decision variablew
tf_guess = 11e3;
w = [w{:} ; T];
lbw = [lbw ; 0];
ubw = [ubw ; inf];
w0 = [w0 ; tf_guess];

% defining objective function
L = 0.05*T + (m0-Xk(end));

% Create an NLP solver
prob = struct('f', L, 'x', w, 'g', g);
solver = nlpsol('solver', 'ipopt', prob);

% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, ...
             'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);

%%

% Plot the solution
u_opt = w_opt(1:end-1);
t_opt = w_opt(end);
dt = t_opt/100;
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
x1_opt = x_opt(1,:);
x2_opt = x_opt(2,:);
tgrid = linspace(0, T, N+1);
clf;
hold on
plot(tgrid, x1_opt, '--')
plot(tgrid, x2_opt, '-')
stairs(tgrid, [u_opt; nan], '-.')
xlabel('t')
legend('x1','x2','u')