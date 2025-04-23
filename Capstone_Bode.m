close all; clear; clc;

% Parameters
Vin = 12;
Vout = 22;
R = 1.47;
L = 7e-6;
C = 380e-6;
Rc = 0.03;         % ESR
Vm = 3.8;          % PWM ramp (assumed)
Fsw = 100e3;
ws = 2*pi*Fsw;
D = 1 - Vin/Vout;

% Boost small-signal output transfer function approximations
Q = R * sqrt(C/L);
wo = sqrt(1/(L*C));           % Natural frequency
wesr = 1/(C*Rc);              % ESR zero
fz = 0.8*wo;
fz2 = 2*wo;
fp2 = 0.4*ws;
wc = 0.2*ws;

% Calculate gain constant for controller
Kv = Vm * fz * fz2 * wc / Vin / (wo^2);

% Frequencies for inspection
fo = wo / (2*pi);
fz1 = fz / (2*pi);
fz2 = fz2 / (2*pi);
fp1 = wesr / (2*pi);

fprintf("fo  = %.1f Hz\nfz1 = %.1f Hz\nfz2 = %.1f Hz\nfp1 = %.1f Hz\n", fo, fz1, fz2, fp1);

% Transfer Functions
s = tf('s');

% Power stage transfer function (Control-to-Output)
Gvd = Vout/Vin * (1 + s/wesr) / (1 + s/(Q*wo) + (s/wo)^2);

% Type II Compensator
Fv = Kv/s * (1 + s/fz) * (1 + s/fz2) / (1 + s/wesr) / (1 + s/fp2);
Fm = 1/Vm;                            % Modulator gain
Tm = Gvd * Fv * Fm;                   % Loop gain

% Plot Tm Bode plot (loop gain)
figure(1);
h = bodeplot(Tm);
p = getoptions(h);
p.FreqUnits = 'Hz';
setoptions(h, p);
grid on;
title('Loop Gain T_m');
margin(Tm);

% Control-to-output transfer function Gvs
Gvs = D * (1 + s/wesr) / (1 + s/(Q*wo) + (s/wo)^2);
Au2 = Gvs / (1 + Tm); % Small-signal output impedance
figure(2);
bode(Au2);
title('Closed-Loop Output Impedance A_u2');
grid on;

% Asymptotic gain
Kt = Vin * Kv / Vm;
Ka = D / Kt;
Kdc = 20*log10(Kv);
fprintf("Controller DC Gain (Kv) = %.2f dB\n", Kdc);

% Asymptotic magnitude plot
Au = Ka * s * (1 + s/wesr) / (1 + s/fz) / (1 + s/fz2) / (1 + s/wc);
figure(3);
bode(Au);
title('Asymptotic Gain A_u');
grid on;

% Output Impedance
Kz = Rc * fz * wesr / fz / fz2 / wc;
Zo = Kz * s * (1 + s/fz) * (1 + s/wesr) / (1 + s/fz) / (1 + s/fz2) / (1 + s/wc);
figure(4);
bode(Zo);
title('Output Impedance Z_o');
grid on;
