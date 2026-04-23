%
%   Simulation of dc_motor angle with P-controllers for moving reference
%     Author: Keitaro Naruse
%     Date: 2023-04-27
%

% Symbol
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

% Define a transfer function of DC motor angle
P = tf( [ 1 ] , [ Ke*Tm*Te, Ke*Tm, Ke, 0 ] );

% Define a time period and a reference
t = [ 0:0.0001:1 ];
R1 = 300 * sin( 1 * t );
R2 = 300 * sin( 10 * t );
R3 = 300 * sin( 100 * t );
R4 = 300 * sin( 1000 * t );

% P-controllers with defferent gains
Cp1 = 0.01;
Cp2 = 0.1;
Cp3 = 1.0 + 1/s + 1*s;
Gp1 = feedback( Cp1 * P, 1 );
Gp2 = feedback( Cp2 * P, 1 );
Gp3 = feedback( Cp3 * P, 1 );

% Actual output of P-controllers for the reference R1
[ Y_Gp1_R1, t ] = lsim( Gp1, R1, t ); 
[ Y_Gp2_R1, t ] = lsim( Gp2, R1, t ); 
[ Y_Gp3_R1, t ] = lsim( Gp3, R1, t ); 
figure( 1 );
plot( t, R1, 'k-', t, Y_Gp1_R1, 'r-', t, Y_Gp2_R1, 'g-', t, Y_Gp3_R1, 'b-' );
title('Moving reference R1 = 300*sin(t) and output by P-controllers with different gains');
legend('Referece', 'C = 0.01', 'C = 0.1', 'C = 1.0');

% Actual output of P-controllers for the reference R2
[ Y_Gp1_R2, t ] = lsim( Gp1, R2, t ); 
[ Y_Gp2_R2, t ] = lsim( Gp2, R2, t ); 
[ Y_Gp3_R2, t ] = lsim( Gp3, R2, t ); 
figure( 2 );
plot( t, R2, 'k-', t, Y_Gp1_R2, 'r-', t, Y_Gp2_R2, 'g-', t, Y_Gp3_R2, 'b-' );
title('Moving reference R2 = 300*sin(10t) and output by P-controllers with different gains');
legend('Referece', 'C = 0.01', 'C = 0.1', 'C = 1.0');

% Actual output of P-controllers for the reference R3
[ Y_Gp1_R3, t ] = lsim( Gp1, R3, t ); 
[ Y_Gp2_R3, t ] = lsim( Gp2, R3, t ); 
[ Y_Gp3_R3, t ] = lsim( Gp3, R3, t ); 
figure( 3 );
plot( t, R3, 'k-', t, Y_Gp1_R3, 'r-', t, Y_Gp2_R3, 'g-', t, Y_Gp3_R3, 'b-' );
title('Moving reference R3 = 300*sin(100t) and output by P-controllers with different gains');
legend('Referece', 'C = 0.01', 'C = 0.1', 'C = 1.0');

% Actual output of P-controllers for the reference R4
[ Y_Gp1_R4, t ] = lsim( Gp1, R4, t ); 
[ Y_Gp2_R4, t ] = lsim( Gp2, R4, t ); 
[ Y_Gp3_R4, t ] = lsim( Gp3, R4, t ); 
figure( 4 );
plot( t, R4, 'k-', t, Y_Gp1_R4, 'r-', t, Y_Gp2_R4, 'g-', t, Y_Gp3_R4, 'b-' );
title('Moving reference R4 = 300*sin(1000t) and output by P-controllers with different gains');
legend('Referece', 'C = 0.01', 'C = 0.1', 'C = 1.0');

% Bode plots of Gp1, Gp2, and Gp3
figure( 5 );
bode( Gp1, 'r-', Gp2, 'g-', Gp3, 'b-', { 0.1, 10000 } );
title('Bode plots of P-controllers');
legend('C = 0.01', 'C = 0.1', 'C = 1.0');
grid on;