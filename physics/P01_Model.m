close all; clear; clc;
%% Motion (Kinematics & Dynamics): theta dtheta ddtheta

% change path depending on your file location
data_ik_slower01 = importdata(['/Users/ikhlas_mac/Desktop/P01/'...
    'RawData/Level Ground/LG_Walk_Slow/LG_Walk_Slower01_ik.mot']); 
headers1 = data_ik_slower01.colheaders;

% Extract time vector
time = data_ik_slower01.data(:, strcmp(headers1,...
    'time'));

% Extract ankle angle (right leg)
theta = data_ik_slower01.data(:, strcmp(headers1,...
    'ankle_angle_r'));
theta = deg2rad(theta);  % Convert degrees to radians


% Time step
dt = mean(diff(time));

% angular velocity
dtheta = gradient(theta, dt);

% angular acceleration
ddtheta = gradient(dtheta, dt);


%% Inverse Dynamics:
% Load the .mot file (ID)
data_id_slower01 = importdata(['/Users/ikhlas_mac/Desktop/P01/RawData/' ...
    'Level Ground/LG_Walk_Slow/LG_Walk_Slower01_id.sto']);

headers2 = data_id_slower01.colheaders;

% Extract time vector & torque
time_tau = data_id_slower01.data(:, strcmp(headers2,...
    'time'));
tau = data_id_slower01.data(:, strcmp(headers2,...
    'ankle_angle_r_moment'));

% Extract Hip and Knee Torques from OpenSim Inverse Dynamics output
tau_hip  = data_id_slower01.data(:, strcmp(headers2, 'hip_flexion_r_moment'));
tau_knee = data_id_slower01.data(:, strcmp(headers2, 'knee_angle_r_moment'));
tau_ankle = data_id_slower01.data(:, strcmp(headers2, 'ankle_angle_r_moment'));

tau_meas = [tau_hip, tau_knee, tau_ankle];  % Nx3 measure torques

%% checking theta and tau plots:
% figure;
% subplot(4,1,1); plot(time, theta,LineWidth=1.5);
% ylabel('\theta (rad)'); title('Ankle Angle');
% subplot(4,1,2); plot(time, dtheta,LineWidth=1.5);
% ylabel('d\theta/dt (rad/s)'); title('Angular Velocity');
% subplot(4,1,3); plot(time, ddtheta,LineWidth=1.5);
% ylabel('d^2\theta/dt^2 (rad/s^2)'); title('Angular Acceleration');
% subplot(4,1,4);plot(time, tau,LineWidth=1.5);
% ylabel('Torque (Nm)');title('Right Ankle Torque from Inverse Dynamics');
% xlabel('Time (s)');
% grid on

%% Participant Data:
P01_data = load('/Users/ikhlas_mac/Downloads/P01/P01.mat');
P01_data = P01_data.P01;
P01_info = P01_data.Participant_Information;

% mass (kg)
m_tot = P01_info.Weight;  % kg
m_tot = str2double(erase(m_tot, 'kg'));

% height (m)
h = P01_info.Height;
h = (str2double(erase(h, 'cm')))/100;

% Segment Lengths
L1 = 0.245 * h; % thigh
L2 = 0.246 * h; % Shank ("leg" in DA Winter book)
L3 = 0.152 * h; % foot

% Masses
m1 = 0.10   * m_tot; 
m2 = 0.0465 * m_tot; 
m3 = 0.0145 * m_tot; 

% CoM distances
l1 = 0.433 * L1;
l2 = 0.433 * L2;
l3 = 0.442 * L3;

% Radius of gyration (Winter 2009)
r1 = 0.322;
r2 = 0.303;
r3 = 0.475;

% Inertias (distributed model)
I1 = m1 * (r1 * L1)^2;
I2 = m2 * (r2 * L2)^2;
I3 = m3 * (r3 * L3)^2;

% Convert scalar inertia to full 3D form (only Iyy is nonzero)
Ivec = @(Iyy) [0 Iyy 0 0 0 0];  % [Ixx Iyy Izz Ixy Iyz Ixz]

%% Construct Skeletal Model 

% [ pelvis ] —(hip)—> [ thigh ] —(knee)—> [ shank ] —(ankle)—> [ foot ]

% Link angle (joint) vectors:
hip_r   = deg2rad(data_ik_slower01.data(:, strcmp(headers1, 'hip_flexion_r'))); % use flexion for sagittal plane
knee_r  = deg2rad(data_ik_slower01.data(:, strcmp(headers1, 'knee_angle_r')));
ankle_r = deg2rad(data_ik_slower01.data(:, strcmp(headers1, 'ankle_angle_r')));


% Define links with mass and dynamics
L1 = Revolute('a', L1, 'alpha', 0, 'm', m1, 'r', [-l1 0 0], 'I', Ivec(I1));
L2 = Revolute('a', L2, 'alpha', 0, 'm', m2, 'r', [-l2 0 0], 'I', Ivec(I2));

% zero is at 90 degrees wrt shank for the foot (need offset)
L3 = Revolute('a', L3,  'alpha', 0, 'm', m3, 'r', [-l3 0 0], 'I', Ivec(I3), 'offset', pi/2);


% Define Linkage
right_leg = SerialLink([L1 L2 L3], 'name', 'RightLeg');

% re-position & re-orient
right_leg.base = trotx(pi/2)*trotz(-pi/2);

% Joint vectors 
Q = [hip_r, knee_r, ankle_r];  % Nx3

% plot 0 position 
right_leg.plot([0 0 0]); 

% plot first time step
right_leg.plot(Q(1,:)); 

% Animate across all time steps
%right_leg.plot(Q);  

dt = mean(diff(time));

dq1 = gradient(Q(:,1), dt);  % hip velocity
dq2 = gradient(Q(:,2), dt);  % knee velocity
dq3 = gradient(Q(:,3), dt);  % ankle velocity
dQ = [dq1, dq2, dq3];  % reassemble


ddq1 = gradient(dq1, dt);
ddq2 = gradient(dq2, dt);
ddq3 = gradient(dq3, dt);
ddQ = [ddq1, ddq2, ddq3];


n = size(Q, 1);
M_all = zeros(3, 3, n);
C_all = zeros(3, 3, n);
G_all = zeros(n, 3);

for i = 1:n
    q_i = Q(i, :);
    dq_i = dQ(i, :);

    M_all(:, :, i) = right_leg.inertia(q_i);           % M(q)
    C_all(:, :, i) = right_leg.coriolis(q_i, dq_i);     % C(q,dq)
    G_all(i, :)     = right_leg.gravload(q_i);          % G(q)
end
