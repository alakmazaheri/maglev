%% Simulink values
X0 = 0.01;      % meters
L0 = 0.01;       % henry (kg m2 / s2 A2)
C = L0*X0/2;    % kg m3 / s2 A2
m = 0.068;        % kg
g = 9.8;        % m/s2

% length = 0.2;           % m
% d = 0.016;              % m
% N = length/(pi*d);      % no. coils
% mu_air = 1.257 * 10^-6;                % henry/m
% L = mu_air*N^2*area/length;           % inductance of the coil
% L = N^2/R;
L = 0.483;              % H
R = 28;                 % ohms

I0 = sqrt(m*g*X0^2/C);

%% Open loop transfer function
s = tf('s');
G_OL = (2*C*I0*X0 / (-X0^3*m*s^2 + 2*C*I0^2)) * (1/ (R + L*s));
pzmap(G_OL)

%% Closed loop w/PD
kd = 1; kp = 30;
k = 100000;
C = k*(kp + kd*s);
G_CL = C*G_OL / (1 + C*G_OL);
G_CL_simp = minreal(G_CL);
[ps, z] = pzmap(G_CL_simp)

G_CL_true = (s-z)/( (s-ps(1)) * (s-ps(2)) * (s-ps(3))  );
% figure; rlocus(G_CL_true)
% title('PD Controller')
% xlim([-450 100])
% ylim([-400 400])
figure; pzmap(G_CL_simp) % or G_CL_true ... doesn't work either way

%% Closed loop w/lead lag
k = 2*10^6;
z = -30; p = -400;
C_leadlag = k*((s-z)/(s-p));
G_CL = C_leadlag*G_OL / (1 + C_leadlag*G_OL);
G_CL_simp = minreal(G_CL);
[ps, z] = pzmap(G_CL_simp);

G_CL_true = (s-z)/( (s-ps(1)) * (s-ps(2)) * (s-ps(3)) * (s-ps(4)) );
figure; pzmap(G_CL_true)
