clc; clear; close all;

M = readmatrix("C:\Users\lynth\Desktop\map_output.csv");

world   = M ;

worldColor = [1 0 1];
scanColor = 'green';

% Plot
% cfig(1); clf; 
set(gca, 'color', [0,0,0]);
hold on; grid on; axis equal;

plot(world(:,1), world(:,2), '+', 'MarkerSize', 1, 'color', worldColor);


