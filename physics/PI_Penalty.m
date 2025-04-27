
n = size(Q, 1);  % Number of time steps
tau_model = zeros(n, 3);  % Estimated torque for each joint

for i = 1:n
    M = M_all(:, :, i);
    C = C_all(:, :, i);
    G = G_all(i, :).';  
    tau_GRF = tau_GRF_down_all(i,:);

    ddq_i = ddQ(i, :).'; 
    dq_i  = dQ(i, :).';   

    % Estimate joint torques using dynamics equation
    tau_model(i, :) = (M * ddq_i + C * dq_i + G - tau_GRF').';
end

% Measured OpenSim torques 
tau_hip  = data_id_slower01.data(:, strcmp(headers2, 'hip_flexion_r_moment'));
tau_knee = data_id_slower01.data(:, strcmp(headers2, 'knee_angle_r_moment'));
tau_ankle = data_id_slower01.data(:, strcmp(headers2, 'ankle_angle_r_moment'));

tau_meas = [tau_hip, tau_knee, tau_ankle];  % Nx3

tau_hip_model = tau_model(:,1);  % Only 1st joint
tau_knee_model = tau_model(:,2);  % Only 2nd joint
tau_ankle_model = tau_model(:,3);  % Only 3rd joint

% Compute residuals
residual_all = tau_meas - tau_model;  % Nx3

residual_hip = tau_hip - tau_hip_model;
residual_knee = tau_knee - tau_knee_model;
residual_ankle = tau_ankle_meas - tau_ankle_model;

loss_mse = mean(residual_ankle.^2);

% Plot: All Measured vs. Estimated Torque
figure;
plot(time, tau_meas, 'k', 'LineWidth', 1.5); hold on;
plot(time, tau_model, '--r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Torque (Nm)');
legend('Measured \tau', 'Estimated \tau', 'Location', 'Best');
title('Measured vs Estimated Torques');
grid on;

% Plot: Measured vs. Estimated Ankle Torque
figure;
plot(time2, tau_ankle, 'k', 'LineWidth', 1.5); hold on;
plot(time2, tau_ankle_model, '--r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Ankle Torque (Nm)');
legend('Measured \tau_{ankle}', 'Estimated \tau_{ankle}', 'Location', 'Best');
title('Measured vs Estimated Right Ankle Torque');
grid on;


% Plot: Residual
figure;
plot(time2, residual_ankle, 'b');
xlabel('Time (s)');
ylabel('Torque Error (Nm)');
title('Residual: Measured - Estimated Torque at Ankle');
grid on;

% Report
fprintf('\nPhysics-Based Model Evaluation for P01 \n');
fprintf('Physics-informed MSE loss (ankle joint): %.4f Nm²\n', loss_mse);