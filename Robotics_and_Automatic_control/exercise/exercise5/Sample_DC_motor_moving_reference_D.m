%
%   Simulation of dc_motor angle with D-controllers for moving reference
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

% D-controllers with defferent gains
Cd1 = 0.01 * s;
Cd2 = 0.1 * s;
Cd3 = 1.0 * s;
Gd1 = feedback( Cd1 * P, 1 );
Gd2 = feedback( Cd2 * P, 1 );
Gd3 = feedback( Cd3 * P, 1 );

% Actual output of D-controllers for the reference R1
[ Y_Gd1_R1, t ] = lsim( Gd1, R1, t ); 
[ Y_Gd2_R1, t ] = lsim( Gd2, R1, t ); 
[ Y_Gd3_R1, t ] = lsim( Gd3, R1, t ); 
figure( 11 );
plot( t, R1, 'k-', t, Y_Gd1_R1, 'r-', t, Y_Gd2_R1, 'g-', t, Y_Gd3_R1, 'b-' );
title('Moving reference R1 = 300*sin(t) and output by D-controllers with different gains');
legend('Referece', 'C = 0.01*s', 'C = 0.1*s', 'C = 1.0*s');

% Actual output of I-controllers for the reference R2
[ Y_Gd1_R2, t ] = lsim( Gd1, R2, t ); 
[ Y_Gd2_R2, t ] = lsim( Gd2, R2, t ); 
[ Y_Gd3_R2, t ] = lsim( Gd3, R2, t ); 
figure( 12 );
plot( t, R2, 'k-', t, Y_Gd1_R2, 'r-', t, Y_Gd2_R2, 'g-', t, Y_Gd3_R2, 'b-' );
title('Moving reference R2 = 300*sin(10t) and output by D-controllers with different gains');
legend('Referece', 'C = 0.01*s', 'C = 0.1*s', 'C = 1.0*s');

% Actual output of I-controllers for the reference R3
[ Y_Gd1_R3, t ] = lsim( Gd1, R3, t ); 
[ Y_Gd2_R3, t ] = lsim( Gd2, R3, t ); 
[ Y_Gd3_R3, t ] = lsim( Gd3, R3, t ); 
figure( 13 );
plot( t, R3, 'k-', t, Y_Gd1_R3, 'r-', t, Y_Gd2_R3, 'g-', t, Y_Gd3_R3, 'b-' );
title('Moving reference R3 = 300*sin(100t) and output by D-controllers with different gains');
legend('Referece', 'C = 0.01*s', 'C = 0.1*s', 'C = 1.0*s');

% Actual output of I-controllers for the reference R4
[ Y_Gd1_R4, t ] = lsim( Gd1, R4, t ); 
[ Y_Gd2_R4, t ] = lsim( Gd2, R4, t ); 
[ Y_Gd3_R4, t ] = lsim( Gd3, R4, t ); 
figure( 14 );
plot( t, R4, 'k-', t, Y_Gd1_R4, 'r-', t, Y_Gd2_R4, 'g-', t, Y_Gd3_R4, 'b-' );
title('Moving reference R4 = 300*sin(1000t) and output by D-controllers with different gains');
legend('Referece', 'C = 0.01*s', 'C = 0.1*s', 'C = 1.0*s');

% Bode plots of Gi1, Gi2, and Gi3
figure( 15 );
bode( Gd1, 'r-', Gd2, 'g-', Gd3, 'b-', { 0.1, 10000 } );
title('Bode plots of D-controllers');
legend( 'C = 0.01*s', 'C = 0.1*s', 'C = 1.0*s' );
grid on;
