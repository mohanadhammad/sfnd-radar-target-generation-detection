clc;
clear;

% Speed of light
c_mps = 3*10^8;

% The range resolution
dres_m = 1;

% The radar maximum range
Rmax_m = 300;

% TODO : Find the Bsweep of chirp for 1 m resolution
Bsweep = c_mps / (2 * dres_m);

% TODO : Calculate the chirp time based on the Radar's Max Range
Tc = 5.5 * 2 * Rmax_m / c_mps;

% TODO : define the frequency shifts 
fb_Hz = [0, 1.1, 13, 24] * 1e6;

calculated_range = (c_mps * Tc * fb_Hz) / (2 * Bsweep);

% Display the calculated range
disp(calculated_range);
