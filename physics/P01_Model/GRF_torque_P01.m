%% Computes Torque due to Ground Reaction Force (F_y) on P01 Ankle
% We look at the right leg only, so only plate 1 is analyzed

% Load the .mot data for external loads               
path_delim = "/"; % use "\" on windows PC and "/" on mac
mot = importdata('data_files_trial_01'+ path_delim +...
    'LG_Walk_Slower01_filtered.mot');
mot_data = mot.data;
mot_headers = mot.colheaders;

% Extract time vector
time = mot_data(:, strcmp(mot_headers,...
    'time'));

% load OpenSim torque from .sto 
data_id_slower01 = importdata('data_files_trial_01'+ path_delim + ...
    'LG_Walk_Slower01_id.sto');
headers2 = data_id_slower01.colheaders;
% Extract time vector & torque
time_tau = data_id_slower01.data(:, strcmp(headers2,...
    'time'));
tau = data_id_slower01.data(:, strcmp(headers2,...
    'ankle_angle_r_moment'));


% Get GRF and COP columns to create the vectors

% F_y
vy1_idx = find(strcmp(mot_headers, 'ground_force1_vy'));

% Pressure centers
px1_idx = find(strcmp(mot_headers, 'ground_force1_px'));
py1_idx = find(strcmp(mot_headers, 'ground_force1_py'));
pz1_idx = find(strcmp(mot_headers, 'ground_force1_pz'));

% Force and COP vectors
F1 = mot_data(:,vy1_idx);  % N
COP1 = mot_data(:, [px1_idx, py1_idx, pz1_idx]);  % m


%% Plotting foot step and force plate 

% force plate #1


x1 = COP1(:,1);       % Medial-Lateral (sideways)
y1 = COP1(:,2);       % Vertical 
z1 = COP1(:,3);       % Anterior-Posterior (forward/back)


% Center of the plate (X, Y, Z) in OpenSim global coordinates
p = [-0.13, 0, 0.305];  

% Plate dimensions
width_x = 0.4;   % along X 
length_z = 0.6;  % along Z 

% Corners of the rectangle (centered at p, lying in the X-Z plane)
x_corners = p(1) + [-0.5,  0.5,  0.5, -0.5]*width_x;  
z_corners = p(3) + [-0.5, -0.5,  0.5,  0.5]*length_z;  
y_corners = zeros(1, 4);               

% Plot the plate
figure;
fill3(x_corners, y_corners, z_corners, [0.8 0.8 0.8]);
hold on;

% Plot center of the plate (COP when plate is idle)
plot3(p(1), p(2), p(3), 'ro', 'MarkerFaceColor', 'r');
text(p(1), p(2), p(3), '  C', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');

% labeled to match Opensim 
xlabel('X');
ylabel('Y');
zlabel('Z');
zlim([z_corners(1) z_corners(3)]);
xlim(x_corners(1:2));

title('COP Trajectory');
axis equal;
grid on;
view(3);

% first contact
idx1 = find(F1(:), 1, 'first'); % first contact
% last contact
idx2 = find(F1(:), 1, 'last'); 

% Plot the COPs during stepping.
% plotted during contact with plate (between t = 7.32s to t = 8.94s) 
for t = idx1:idx2
    if t == idx1
        plot3(x1(t), y1(t), z1(t), 'o','Color','g','MarkerSize',15,'LineWidth',2); hold on;
    elseif t == idx2
        plot3(x1(t), y1(t), z1(t), 'x','Color','r','MarkerSize',15,'LineWidth',2); hold on;
    else
        plot3(x1(t), y1(t), z1(t), '+','Color','k')
    end
    title('COP Trajectory');
    grid on;
    axis equal;
    hold on
end
view(3);


%% Approximate Ankle Position:

% Estimated Ankle position (First point of contact with the plate):
Ankle_pos = COP1(idx1,:);

% Compute arm (r) and torque values:
r_ankle = zeros(size(COP1,1),1);
tau_GRF = zeros(size(COP1,1),1);

for i = 1: size(COP1,1)
    r_ankle(i) = abs(COP1(i,3)- Ankle_pos(3));
    tau_GRF(i) = r_ankle(i)*F1(i);
end

% Keep every 10th sample (downsampling to match theta)
tau_GRF_down = downsample(tau_GRF, 10);  
tau_GRF_all = [zeros(size(tau_GRF_down,1),1) zeros(size(tau_GRF_down,1),1) tau_GRF_down];


%% Plot:

figure
plot(time,F1,'LineWidth',2)
xlabel('Time (s)');
ylabel('F_y (N)');
xlim([5 9.5]);
title('Vertical Ground Reaction Force on Ankle')

figure
plot(time,-tau_GRF,'--','LineWidth',2)
xlabel('Time (s)');
ylabel('\tau_{GRF} (Nm)');
xlim([5 9.5]);
title('Vertical Ground Reaction Force on Ankle')

hold on;
plot(time_tau,tau,'LineWidth',2)
xlabel('Time (s)');
ylabel('Torque (Nm)');
xlim([5 9.5]);
title('GRF Torque and OpenSim Torque')
legend('GRF torque','OpenSim torque')