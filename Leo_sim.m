% Simple LEO Orbit Simulation
mu = 3.986e14; % Earth's gravitational parameter (m^3/s^2)
Re = 6371e3; % Earth radius (m)
h = 400e3; % Altitude (m)
a = Re + h; % Semi-major axis
n = sqrt(mu / a^3); % Mean motion (rad/s)
t = 0:60:3600*90; % 90-min orbit times
theta = n * t; % True anomaly
x = a * cos(theta); y = a * sin(theta);
plot(x/1e3, y/1e3); xlabel('X (km)'); ylabel('Y (km)'); title('Project Boscoe LEO Orbit');