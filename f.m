function xdot = f(x,u,lla0)

%constants
g= 9.81;       % m/s^2
rho0 = 1.225;  % kg/m^3
%aerodynamics constants
cd0 = 0.025452;
k = 0.035815;
S = 124.65;    % m^2
%fuel consumption constants
Cf_cr=0.92958;Cf_1=0.70057;Cf_2=1068.1;
%engine thrust constants
CT_cr = 0.95;CTc_1 = 146590;CTc_2=53872;CTc_3 = 3.0453e-11;

%sub variables for dynamics
rho = rho0*(1-(2.2257e-5)*x(3))^4.2586;
CL = (2*x(6)*g)/(rho*S*(x(4)^2)*cos(u(2)));
CD = cd0 + k*CL^2;
Thr_max = CT_cr*CTc_1*(1 - 3.28*x(3)/CTc_2 + CTc_3*(3.28*x(3))^2);
eta = (Cf_1/60000)*(1 + 1.943*x(4)/Cf_2);
frac = u(3)*Thr_max*eta*Cf_cr;

%wind coefs
%coef of cx
cx0=-21.151;cx1=10.0039;cx2=1.1081;cx3=-0.5239;
cx4=-0.1297;cx5=-0.006;cx6=0.0073;cx7= 0.0066;cx8=-0.0001;
%coef of cy
cy0=-65.3035;cy1=17.6148;cy2=1.0855;cy3=-0.7001;
cy4=-0.5508;cy5=-0.003;cy6=0.0241;cy7=0.0064;cy8=-0.000227;

%converting xyzned to lla for wind
xyzNED = [x(1) x(2) x(3)];

lla = lla2ned2(xyzNED,lla0);
lat = lla(1);
lon = lla(2);

Wx = cx0 + cx1*lon + cx2*lat + cx3*lon*lat +...
     cx4*lon^2 + cx5*lat^2 +cx6*lon^2*lat +...
     cx7*lon*lat^2 + cx8*lon^2*lat^2;

Wy = cy0 + cy1*lon + cy2*lat + cy3*lon*lat +...
     cy4*lon^2 + cy5*lat^2 + cy6*lon^2*lat +...
     cy7*lon*lat^2 + cy8*lon^2*lat^2;

%dynamics
xdot = [    
        x(4)*cos(x(5))*cos(u(1)) + Wx;
        x(4)*sin(x(5))*cos(u(1)) + Wy;
        x(4)*sin(u(1));
        u(3)*Thr_max/x(6) - g*sin(u(1)) - (CD*S*rho*x(4)^2)/(2*x(6));
        (CL*S*rho*x(4)*sin(u(2)))/(2*x(6)*cos(u(1)));
        -frac
        ];

end