%% Extract Forces and COPs: (Torque due to Ground Reaction Force (F_y))
% We look at the right leg only, so only plate 1 is analyzed

% Load the .mot data
mot = importdata('/Users/ikhlas_mac/Desktop/P01/RawData/Level Ground/LG_Walk_Slow/LG_Walk_Slower01_filtered.mot');
mot_data = mot.data;
mot_headers = mot.colheaders;

% Extract time vector
time = mot_data(:, strcmp(mot_headers,...
    'time'));

% Get indices for GRF and COP columns to create the vectors
% plate 1

% Forces
vx1_idx = find(strcmp(mot_headers, 'ground_force1_vx'));
vy1_idx = find(strcmp(mot_headers, 'ground_force1_vy'));
vz1_idx = find(strcmp(mot_headers, 'ground_force1_vz'));

% Pressure centers
px1_idx = find(strcmp(mot_headers, 'ground_force1_px'));
py1_idx = find(strcmp(mot_headers, 'ground_force1_py'));
pz1_idx = find(strcmp(mot_headers, 'ground_force1_pz'));


% Get force and COP vectors for both plates
F1 = mot_data(:, [vx1_idx, vy1_idx, vz1_idx]);  % N
COP1 = mot_data(:, [px1_idx, py1_idx, pz1_idx]);  % m


%% Steps Visulaization: 

% OpenSim coordinate convention
x1 = COP1(:,1);       % Medial-Lateral (sideways)
y1 = COP1(:,2);       % Vertical 
z1 = COP1(:,3);       % Anterior-Posterior (forward/back)

% show the plate
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

xlabel('X (Medial-Lateral)');
ylabel('Y (Vertical)');
zlabel('Z (Anterior-Posterior)');
title('Force Plate and COP Trajectory in OpenSim Global Coordinates');
axis equal;
grid on;
view(3);


% Plot the COPs during stepping.
% plotted during contact with plate (between t = 7.32s to t = 8.94s) 
for t = 2415:4040
    if t == 2420
        plot3(x1(t), y1(t), z1(t), 'o','Color','g','MarkerSize',15); hold on;
    elseif t == 4040
        plot3(x1(t), y1(t), z1(t), 'x','Color','r','MarkerSize',15); hold on;
    else
        plot3(x1(t), y1(t), z1(t), '+')
    end
    xlabel('X (Medial-Lateral)');
    ylabel('Y');
    zlabel('Z');
    title('COP Trajectory in OpenSim Global Coordinates');
    grid on;
    axis equal;
    hold on
end
zlim([min(z1) 0.7])
view(3);


%% Approximate Ankle Position:

% Find index of first non-zero F_y instance
idx = find(F1(:,2), 1, 'first');  % returns 4

% Estimated Ankle position (First point of contact with the plate):
Ankle_pos = COP1(idx,:);

% Compute arm (r) and torque values:
r_ankle = zeros(size(COP1,1),1);
tau_GRF_all = zeros(size(COP1,1),1);

for i = 1: size(COP1,1)
    r_ankle(i) = COP1(i,3)- Ankle_pos(3);
    tau_GRF_all(i) = r_ankle(i)*F1(i,2);
end


% Keep every 10th sample (downsampling to match theta)
tau_GRF_down = downsample(tau_GRF_all, 10);  
tau_GRF_down_all = [zeros(481,1) zeros(481,1) tau_GRF_down];