%%%% Lateral Vehicle Dynamics
clc
clear all
%the property of vehicle
m = 2650; % mass(kg)
I = 3000; %Inertia(kg*m^2)
a = 1.70; % Distance from center of mass to front axle (m)
b = 1.71; % Distance from center of mass to  rear axle (m)
Caf = -82600; %Lateral stiffness of the front wheels (N/rad)
Car = -71900; %Lateral stiffness of the rear wheels (N/rad)
Vx = 25; %Vx is the longitudinal velocity (m/s)
Wr = 2; %rotation velocity (rad/s)

%% Initial Element
% y is the vehicle lateral postion, measured along the vehicle
% lateral axis to the point O which is the center of rotation
% Vx is the longitudinal velocity;
% yaw is the yaw angle;
% define x = [y; dot(y); yaw; dot(yaw)];  change to error state
As = [ 0, 1, 0, 0;
      0, 2*(Caf+Car)/(m*Vx), -2*(Caf+Car)/m, 2*(Caf*a-Car*b)/(m*Vx);
      0, 0, 0, 1;
      0, 2*(Caf*a-Car*b)/(I*Vx) ,-2*(Caf*a-Car*b)/I, 2*(Caf*b^2+Car*a^2)/(I*Vx)];
% A =round(A,2);
Bs = [0; -2*Caf/m; 0; -2*a*Caf/I];
Es = [0; 2*(Caf*a-Car*b)/(m*Vx)-Vx;0;2*(Caf*a^2+Car*b^2)/(I*Vx)];
E = Es * Wr;
% B = round(B,2);
Cs = [0 0 1 0];
Ds = zeros(size(Cs,1), size(Bs,2));
% initial state
x0 = [0.8;0.2;-0.6;0];
%% turn the continuous part into discrete and judge the stability of the initial state
Ts = 0.25; %sample frequency(s)
sys = ss(As,Bs,Cs,Ds);
G = tf(sys);
p = pole(G)
[GM, PM, Wcg, Wcp] = margin(G);
sys_d  = c2d(sys,Ts,'zoh');
% get discrete A,B,C
A = sys_d.A;
B = sys_d.B;
C = sys_d.C;
D = sys_d.D;
[numerator, denominator] = tfdata(sys, 'v');
G_d = tf(sys_d)
p_d = pole(G_d)
[GM, PM, Wcg, Wcp] = margin(G_d)
figure(1)
bode(sys_d);
grid on

% figure(2)
% step(sys_d)
% grid on
% figure(2)
% nyquist(sys_d)
% 
% % lsim
% figure(2)
% initial(sys_d,x0)
% figure(4)
% t = 1 : 0.5 : 30;
% u = max(0,min(t-1,1));
% [u,t] = gensig("square",2,20);
% lsim(sys_d,u,t,x0)

%% Use PID to make the system more stable

% suppose PM = 55 a steady error is 0.5
% pid choose ww = 1
% x = 2.5;
% result_num = polyval(numerator,x*i);
% result_de = polyval(denominator,x*i);
% p = result_num/result_de;
% magnitude = abs(p);
% phase = angle(p);
% phase = rad2deg(phase);
% theta = 180 + 55 - phase;
% % Kp = cosd(theta)/magnitude
% %let Ki = 0.5
Kp = 0.4;
Ki = 0.06;
Kd = 0.1;
% Kd = (sind(theta)/magnitude + Ki/x)/x
% Kd = 0;
% Tf = 1;
% PID = pid(Kp,Ki,Kd,Tf,Ts);
% 
% 
% sys_control = feedback(sys_d*PID,1);
% [numerator_c, denominator_c] = tfdata(sys_control, 'v');
% Gz = tf(sys_control)
% % % initial(sys_control,x0)
% figure
% % step(sys_control)
% figure(4)
% grid on
% bode(sys_control)
% figure(5)
% nyquist(sys_control)
% p_Gz = pole(Gz)
% [GM, PM, Wcg, Wcp] = margin(Gz)

%% pole placement
p0 = [0.7,0.4,-0.7,-0.3];
K = place(A,B,p0);


