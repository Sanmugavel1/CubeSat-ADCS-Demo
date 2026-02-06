%clear; clc; close all;

% ---------------- Time ----------------
dt = 0.01;
T  = 20;
t  = 0:dt:T;

% ---------------- CubeSat ----------------
I = 0.02;
theta = deg2rad(30);
omega = 0;

theta_ref = 0;

% ---------------- PID ----------------
Kp = 8; Ki = 1.5; Kd = 2;
int_err = 0;
prev_err = 0;

% ---------------- IMU ----------------
gyro_noise = deg2rad(0.5);
acc_noise  = deg2rad(1);
alpha = 0.98;
theta_est = theta;

% ---------------- Disturbance ----------
disturb = @(x) (x > 5 && x < 7) * 0.02;

% ---------------- Logs ----------------
theta_true = zeros(1,length(t));
theta_estm = zeros(1,length(t));
control    = zeros(1,length(t));

% ================= LOOP =================
for k = 1:length(t)

    gyro = omega + gyro_noise*randn;
    acc  = theta + acc_noise*randn;

    theta_est = alpha*(theta_est + gyro*dt) + (1-alpha)*acc;

    err = theta_ref - theta_est;
    int_err = int_err + err*dt;
    der = (err - prev_err)/dt;

    u = Kp*err + Ki*int_err + Kd*der;
    prev_err = err;

    omega = omega + (u + disturb(t(k)))/I * dt;
    theta = theta + omega*dt;

    theta_true(k) = theta;
    theta_estm(k) = theta_est;
    control(k)    = u;
end

% ================= PLOTS =================
t = 0:dt:T;   % 🔒 LOCKED FIX (NO ERROR POSSIBLE)

figure
subplot(2,1,1)
plot(t, rad2deg(theta_true),'b','LineWidth',1.8); hold on
plot(t, rad2deg(theta_estm),'r--','LineWidth',1.5)
yline(0,'k:')
grid on
xlabel('Time (s)')
ylabel('Angle (deg)')
title('CubeSat Attitude (Ground Station)')
legend('True','Estimated','Reference')

subplot(2,1,2)
plot(t, control,'m','LineWidth',1.6)
grid on
xlabel('Time (s)')
ylabel('Control Torque (Nm)')
title('PID Control Output')

%clear; clc; close all;

figure('Color','w');
axis equal;
axis([-1 1 -1 1 -1 1]);
grid on;
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('CubeSat 3D Visualization');

% Define cube vertices
vertices = [
    -0.5 -0.5 -0.5;
     0.5 -0.5 -0.5;
     0.5  0.5 -0.5;
    -0.5  0.5 -0.5;
    -0.5 -0.5  0.5;
     0.5 -0.5  0.5;
     0.5  0.5  0.5;
    -0.5  0.5  0.5
];

faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];

cube = patch('Vertices',vertices,'Faces',faces,...
             'FaceColor',[0.2 0.6 1],'FaceAlpha',0.8);
theta_log = theta_true;   % from your simulation

for k = 1:length(theta_log)

    theta = 20*theta_log(k);

    R = [1 0 0;
         0 cos(theta) -sin(theta);
         0 sin(theta)  cos(theta)];

    rotated_vertices = (R * vertices')';
    set(cube,'Vertices',rotated_vertices);

    drawnow;
    pause(0.01);
end
