%
%   Simulation of dc_motor position with ultimat gain
%       Author: Keitaro Naruse
%       Date: 2023-04-24
%

% s is symbol for transfer function
s = tf('s');

% Motor parameres
% R = 2.61;
% L = 0.02 * 1E-3;
% Jm = 0.014 * 1E-7;
% Ki = 15400/60*2*pi;
% Ke = 1/Ki;
% Kt = 0.657 * 1E-3;
% Tm = ( R * Jm ) / ( Ke * Kt ) ;
% Te = L/R;

R = 37.5;
L = 0.29 * 1E-3;
Jm = 0.015 * 1E-7;
Ki = 3840/60*2*pi;
Ke = 1/Ki;
Kt = 2.63 * 1E-3;
Tm = ( R * Jm ) / ( Ke * Kt ) ;
Te = L/R;


% Set a plant of DC motor position as transfer function
P = tf([1], [Ke*Tm*Te, Ke*Tm, Ke, 0]);

% Simulation setting
t = [0 : 0.00001 : 1];
% t = [0 : 0.00001 : 1];


% Set a P-controller
Kc = 321.575;  %% when Kc gain is this balue, output is pure oscillation
C1 = Kc;
Tc = 0.00162; %% this is the period

% Set a reference as 1
X1 = 1 * ones(size(t));

% Whole system
G1 = feedback(C1*P, 1);

% Simulation
Y_G1_X1 = lsim(G1, X1, t);

% Plot for x = 1
figure(1);
plot(t, Y_G1_X1, 'r-');
xlim([0, 1.0]);
ylim([0, 2.1]);
title('DC motor angular position for finding unlimate gain');

% Designed P, PI, PD, PID controllers
% Designed P controller
Kp = 0.5 * Kc; 
C2 = Kp;
% Whole system
G2 = feedback(C2*P, 1);
% Simulation
Y_G2_X1 = lsim(G2, X1, t);

% Designed PI controller
Kp = 0.45 * Kc; 
% C3 = Kp * (1 + 1/(0.83 * Tc * s));
C3 = Kp * (1 + 1/(100 * Tc * s));  %% I modify it
% Whole system
G3 = feedback(C3*P, 1);
% Simulation
Y_G3_X1 = lsim(G3, X1, t);

% % Designed PD controller
% Kp = 0.8 * Kc; 
% C4 = Kp * (1 + 0.125 * Tc * s);
% % Whole system
% G4 = feedback(C4*P, 1);
% % Simulation
% Y_G4_X1 = lsim(G4, X1, t);


% Designed PID controller
Kp = 0.6 * Kc; 
C5 = Kp * (1 + 1/(0.5 * Tc * s) + 0.125*Tc*s);
% Whole system
G5 = feedback(C5*P, 1);
% Simulation
Y_G5_X1 = lsim(G5, X1, t);

% % Plot
% figure(2);
% plot(t, Y_G2_X1, 'r-', t, Y_G4_X1, 'b-', t, Y_G5_X1, 'k-');
% title('DC motor angular speed by ultimate gain method');
% legend('P', 'PD', 'PID');
% figure(3);
% plot(t, Y_G2_X1, 'r-', t, Y_G3_X1, 'g-', t, Y_G4_X1, 'b-', t, Y_G5_X1, 'k-');

figure;
plot(t, Y_G2_X1, 'r-', ...
     t, Y_G3_X1, 'g-', ...
     t, Y_G5_X1, 'k-');
legend('P', 'PI', 'PID');
title('PID tuning by ultimate gain method');