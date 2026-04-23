%
%   Simulation of dc_motor position with process reaction
%     Author: Keitaro Naruse
%     Date: 2025-04-24
%

% Symbol
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

% Define a transfer function of DC motor angle
P = tf( [ 1 ] , [ Ke*Tm*Te, Ke*Tm, Ke, 0 ] );

% Define a time period and a reference
t = [ 0:0.001:1 ];
r = 300 * sin( 500000000*t );
C = 1 + 1/s + 1*s;
G = feedback( C*P, 1 );
Y = lsim( G, r, t );
plot( t, r, 'k-', t, Y, 'r-' );

figure( 2 );
opts = bodeoptions();
% opts.XLimMode = 'manual';
opts.XLim = {[0.1,1000]};
opts.YLim = {[-60 10],[-180 180]};
bode( G, opts );

% pidTuner( P, 'pid' );