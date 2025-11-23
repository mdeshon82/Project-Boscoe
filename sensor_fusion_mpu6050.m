% Load data
load('mpu6050_data.mat');
% Madgwick AHRS
ahrs = madgwickAHRS('SampleRate', 100);
quaternion = ahrs.updateIMU(accel, gyro);
eul = eulerd(quaternion, 'ZYX');
plot(eul);