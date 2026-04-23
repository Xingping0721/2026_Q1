%
%   Simulation of dc_motor angle with I-controllers for moving reference
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

% I-controllers with defferent gains
Ci1 = 0.01 / s;
Ci2 = 0.1 / s;
Ci3 = 1.0 / s;
Gi1 = feedback( Ci1 * P, 1 );
Gi2 = feedback( Ci2 * P, 1 );
Gi3 = feedback( Ci3 * P, 1 );

% Actual output of I-controllers for the reference R1
[ Y_Gi1_R1, t ] = lsim( Gi1, R1, t ); 
[ Y_Gi2_R1, t ] = lsim( Gi2, R1, t ); 
[ Y_Gi3_R1, t ] = lsim( Gi3, R1, t ); 
figure( 6 );
plot( t, R1, 'k-', t, Y_Gi1_R1, 'r-', t, Y_Gi2_R1, 'g-', t, Y_Gi3_R1, 'b-' );
title('Moving reference R1 = 300*sin(t) and output by I-controllers with different gains');
legend('Referece', 'C = 0.01/s', 'C = 0.1/s', 'C = 1.0/s');

% Actual output of I-controllers for the reference R2
[ Y_Gi1_R2, t ] = lsim( Gi1, R2, t ); 
[ Y_Gi2_R2, t ] = lsim( Gi2, R2, t ); 
[ Y_Gi3_R2, t ] = lsim( Gi3, R2, t ); 
figure( 7 );
plot( t, R2, 'k-', t, Y_Gi1_R2, 'r-', t, Y_Gi2_R2, 'g-', t, Y_Gi3_R2, 'b-' );
title('Moving reference R2 = 300*sin(10t) and output by P-controllers with different gains');
legend('Referece', 'C = 0.01/s', 'C = 0.1/s', 'C = 1.0/s');

% Actual output of I-controllers for the reference R3
[ Y_Gi1_R3, t ] = lsim( Gi1, R3, t ); 
[ Y_Gi2_R3, t ] = lsim( Gi2, R3, t ); 
[ Y_Gi3_R3, t ] = lsim( Gi3, R3, t ); 
figure( 8 );
plot( t, R3, 'k-', t, Y_Gi1_R3, 'r-', t, Y_Gi2_R3, 'g-', t, Y_Gi3_R3, 'b-' );
title('Moving reference R3 = 300*sin(100t) and output by I-controllers with different gains');
legend('Referece', 'C = 0.01/s', 'C = 0.1/s', 'C = 1.0/s');

% Actual output of I-controllers for the reference R4
[ Y_Gi1_R4, t ] = lsim( Gi1, R4, t ); 
[ Y_Gi2_R4, t ] = lsim( Gi2, R4, t ); 
[ Y_Gi3_R4, t ] = lsim( Gi3, R4, t ); 
figure( 9 );
plot( t, R4, 'k-', t, Y_Gi1_R4, 'r-', t, Y_Gi2_R4, 'g-', t, Y_Gi3_R4, 'b-' );
title('Moving reference R4 = 300*sin(1000t) and output by I-controllers with different gains');
legend('Referece', 'C = 0.01/s', 'C = 0.1/s', 'C = 1.0/s');

% Bode plots of Gi1, Gi2, and Gi3
figure( 10 );
bode( Gi1, 'r-', Gi2, 'g-', Gi3, 'b-', { 0.1, 10000 } );
title('Bode plots of I-controllers');
legend('C = 0.01/s', 'C = 0.1/s', 'C = 1.0/s');
grid on;