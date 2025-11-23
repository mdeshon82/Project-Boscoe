% File: orbit_sim.m
mu = 3.986004418e14;  % m^3/s^2
R_earth = 6371e3;     % m
h = 400e3;            % 400 km altitude
a = R_earth + h;
T = 2*pi*sqrt(a^3/mu)/60;  % Period in minutes
fprintf('Orbital Period: %.2f minutes\n', T);

% Eclipse fraction (sun-synchronous approximation)
beta = 0;  % Assume polar orbit
eclipse_fraction = acosd(sqrt(1 - (R_earth/a)^2))/180;
sunlit_fraction = 1 - eclipse_fraction;
fprintf('Sunlit: %.1f%%, Eclipse: %.1f%%\n', sunlit_fraction*100, eclipse_fraction*100);