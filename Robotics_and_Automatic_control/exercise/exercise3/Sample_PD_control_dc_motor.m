%
%   Simulation of dc_motor PD-control
%     Author: Keitaro Naruse
%     Date: 2023-04-10
%     You should update motor parameteres and equations
%     MIT License
%

% Motor parameters
R = 2.61;
L = 0.02 * 1E-3;
Jm = 0.014 * 1E-7;
Ki = 15400/60*2*pi;
Ke = 1/Ki;
Kt = 0.657 * 1E-3;
Tm = ( R * Jm ) / ( Ke * Kt ) ;
Te = L/R;

% Set a plant of DC motor speed as transfer function
P = tf([1], [Ke*Tm*Te, Ke*Tm, Ke]);

% Simulation setting
t = [0 : 0.001 : 1];

% Set an PD-controller
C1 = tf([1 0.01], [1]);
C2 = tf([1 0.1], [1]);
C3 = tf([1 1], [1]);

% Set a reference 300, 600, 900 [rad/s]
X3 = 300 * ones(size(t));
X6 = 600 * ones(size(t));
X9 = 900 * ones(size(t));

% Whole system
G1 = feedback(C1*P, 1);
G2 = feedback(C2*P, 1);
G3 = feedback(C3*P, 1);

% Simulation
Y_G1_X3 = lsim(G1, X3, t);
Y_G1_X6 = lsim(G1, X6, t);
Y_G1_X9 = lsim(G1, X9, t);

Y_G2_X3 = lsim(G2, X3, t);
Y_G2_X6 = lsim(G2, X6, t);
Y_G2_X9 = lsim(G2, X9, t);

Y_G3_X3 = lsim(G3, X3, t);
Y_G3_X6 = lsim(G3, X6, t);
Y_G3_X9 = lsim(G3, X9, t);

% Plot for kp = 1, kd = 0.01
figure(1);
plot(t, Y_G1_X3, 'r-',  t, X3, 'k-', t, ...
    Y_G1_X6, 'g-',  t, X6, 'k-', ...
    t, Y_G1_X9, 'b-',  t, X9, 'k-');
xlim([0 1]); ylim([0 1200]);
title('DC motor angular speed by I-control: k = 0.1');
legend('x = 300', 'x = 600', 'x = 900');

% Plot for kp = 1, kd = 0.1
figure(2);
plot(t, Y_G2_X3, 'r-', t, X3, 'k-',...
    t, Y_G2_X6, 'g-', t, X6, 'k-', t, Y_G2_X9, 'b-',t,X9,'k-');
xlim([0 1]); ylim([0 1200]);
title('DC motor angular speed by P-control: k = 0.01');
legend('x = 300', 'x = 600', 'x = 900');

% Plot for kp = 1, kd = 1
figure(3);
plot(t, Y_G3_X3, 'r-', t, X3, 'k-', ...
    t, Y_G3_X6, 'g-', t, X6, 'k-' ,t, Y_G3_X9, 'b-', t, X9, 'k-');
xlim([0 1]); ylim([0 1200]);
title('DC motor angular speed by P-control: k = 0.001');
legend('x = 300', 'x = 600', 'x = 900');

% Plot for x = 900
figure(4);
plot(t, Y_G1_X9, 'r-', t, Y_G2_X9, 'g-', t, Y_G3_X9, 'b-', t, X9, 'k-');
xlim([0 1]); ylim([0 1200]);
title('DC motor angular speed by I-control for x = 900');
legend('k = 0.1', 'k = 0.01', 'k = 0.001');

% Simulation setting
t = [0 : 0.000001 : 0.001];
% Simulation
Y_G1_X3 = lsim(G1, X3, t);
Y_G1_X6 = lsim(G1, X6, t);
Y_G1_X9 = lsim(G1, X9, t);

Y_G2_X3 = lsim(G2, X3, t);
Y_G2_X6 = lsim(G2, X6, t);
Y_G2_X9 = lsim(G2, X9, t);

Y_G3_X3 = lsim(G3, X3, t);
Y_G3_X6 = lsim(G3, X6, t);
Y_G3_X9 = lsim(G3, X9, t);

% Plot for kp = 1, kd = 0.01
figure(7);
plot(t, Y_G1_X3, 'r-',  t, X3, 'k-', t, ...
    Y_G1_X6, 'g-',  t, X6, 'k-', ...
    t, Y_G1_X9, 'b-',  t, X9, 'k-');
xlim([0 0.001]); ylim([0 1200]);
title('DC motor angular speed by I-control: k = 0.1');
legend('x = 300', 'x = 600', 'x = 900');

% Plot for kp = 1, kd = 0.1
figure(8);
plot(t, Y_G2_X3, 'r-', t, X3, 'k-',...
    t, Y_G2_X6, 'g-', t, X6, 'k-', t, Y_G2_X9, 'b-',t,X9,'k-');
xlim([0 0.001]); ylim([0 1200]);
title('DC motor angular speed by P-control: k = 0.01');
legend('x = 300', 'x = 600', 'x = 900');

% Plot for kp = 1, kd = 1
figure(9);
plot(t, Y_G3_X3, 'r-', t, X3, 'k-', ...
    t, Y_G3_X6, 'g-', t, X6, 'k-' ,t, Y_G3_X9, 'b-', t, X9, 'k-');
xlim([0 0.001]); ylim([0 1200]);
title('DC motor angular speed by P-control: k = 0.001');
legend('x = 300', 'x = 600', 'x = 900');

% Plot for x = 900
figure(10);
plot(t, Y_G1_X9, 'r-', t, Y_G2_X9, 'g-', t, Y_G3_X9, 'b-', t, X9, 'k-');
xlim([0 0.0001]); ylim([0 1200]);
title('DC motor angular speed by I-control for x = 900');
legend('k = 0.1', 'k = 0.01', 'k = 0.001');

