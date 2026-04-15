%
%   Simulation of dc_motor I-control
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

% % Set an I-controller
% C1 = 0.1 * tf([1], [1, 0]);
% C2 = 0.01 * tf([1], [1, 0]);
% C3 = 0.001 * tf([1], [1, 0]);
% P, I, PI, and PD control
C1 = 0.1;                      % P
C2 = tf([0.2], [1, 0]);        % I
C3 = tf([0.1, 0.2], [1, 0]);   % PI
C4 = tf([0.2, 0.1], [1]);      % PD

% Set a reference 300, 600, 900 [rad/s] 1000
% X3 = 300 * ones(size(t));
% X6 = 600 * ones(size(t));
% X9 = 900 * ones(size(t));
X10 = 1000 * ones(size(t));

% Whole system
G1 = feedback(C1*P, 1);
G2 = feedback(C2*P, 1);
G3 = feedback(C3*P, 1);
G4 = feedback(C4*P, 1);

% Simulation
Y_G1_X10 = lsim(G1, X10, t);
Y_G2_X10 = lsim(G2, X10, t);
Y_G3_X10 = lsim(G3, X10, t);
Y_G4_X10 = lsim(G4, X10, t);

% Plot for c = 0.1
figure(1);
plot(t, Y_G1_X10, 'r-',  t, ...
    Y_G2_X10, 'g-',   ...
    t, Y_G3_X10, 'b-',   ...
    t, Y_G4_X10, 'm-', t, X10, 'k');


xlim([0 1]); ylim([0 1200]);
title('DC motor angular speed by P, I, PI, PD-control');
legend('P-control', 'I-control', 'PI-control', 'PD-control', 'reference');


% 
% % Plot for c = 0.01
% figure(2);
% plot(t, Y_G2_X3, 'r-', t, X3, 'k-',...
%     t, Y_G2_X6, 'g-', t, X6, 'k-', t, Y_G2_X9, 'b-',t,X9,'k-');
% xlim([0 1]); ylim([0 1200]);
% title('DC motor angular speed by P-control: k = 0.01');
% legend('x = 300', 'x = 600', 'x = 900');
% 
% % Plot for c = 0.001
% figure(3);
% plot(t, Y_G3_X3, 'r-', t, X3, 'k-', ...
%     t, Y_G3_X6, 'g-', t, X6, 'k-' ,t, Y_G3_X9, 'b-', t, X9, 'k-');
% xlim([0 1]); ylim([0 1200]);
% title('DC motor angular speed by P-control: k = 0.001');
% legend('x = 300', 'x = 600', 'x = 900');
% 
% % Plot for x = 900
% figure(6);
% plot(t, Y_G1_X9, 'r-', t, Y_G2_X9, 'g-', t, Y_G3_X9, 'b-', t, X9, 'k-');
% xlim([0 1]); ylim([0 1200]);
% title('DC motor angular speed by I-control for x = 900');
% legend('k = 0.1', 'k = 0.01', 'k = 0.001');
% 
