%
%   Simulation of dc_motor position with process reaction
%       Author: Keitaro Naruse
%       Date: 2023-04-24
%
s = tf('s');
% Motor parameres
R = 2.61;
L = 0.02 * 1E-3;
Jm = 0.014 * 1E-7;
Ki = 15400/60*2*pi;
Ke = 1/Ki;
Kt = 0.657 * 1E-3;
Tm = ( R * Jm ) / ( Ke * Kt ) ;
Te = L/R;

% Set a plant of DC motor position as transfer function
P = tf([1], [Ke*Tm*Te, Ke*Tm, Ke, 0]);

% Simulation setting
t = [0 : 0.00001 : 1];

% Set a reference as 1
X1 = 1 * ones(size(t));

% Simulation
Y_P_X1 = lsim(P, X1, t);

% Plot for x = 1
figure(1);
plot(t, Y_P_X1, 'r-');
xlim([0, 0.1]); ylim([0, 100]);
title('DC motor angular position for process reaction');

% Read a slope R and a corssing point L from a plot
R = 1538;
L = 0.005;

% Designed P, PI, PID controllers
% Designed P controller
Kp = 1/(R*L); 
C2 = Kp;
% Whole system
G2 = feedback(C2*P, 1);
% Simulation
Y_G2_X1 = lsim(G2, X1, t);

% Designed PI controller
Kp = 0.9/(R*L); 
C3 = Kp * (1 + 1/(3.33 * L * s));
% Whole system
G3 = feedback(C3*P, 1);
% Simulation
Y_G3_X1 = lsim(G3, X1, t);

% Designed PID controller
Kp = 1.2/(R*L); 
C4 = Kp * (1 + 1/(2*L*s) + 0.5*L*s);
% Whole system
G4 = feedback(C4*P, 1);
% Simulation
Y_G4_X1 = lsim(G4, X1, t);

% Plot
figure(2);
plot(t, Y_G2_X1, 'r-', t, Y_G3_X1, 'g-', t, Y_G4_X1, 'b-');
title('DC motor angular speed by reaction process method');
legend('P', 'PI', 'PID');
