% updated to work for walk only trials (by manally inputting which trials
% to use force plate 2 and updating  path to be more flexible.
% TODO: Get working for sit to stand trials, remembering to chop first x
% seconds to avoid standing part of trials.

function GRF_torque = GRF_compute(iTrialType, iGait, iSpeed, iTrial, trial_data)
% takes input from the preprocess file:
% iTrialType: Walk, Stand-to-walk or Sit-to-stand-to-walk?
% iGait: ght foot or left foot?
% iSpeed = (slow, med, fast?), (rightfoot?), (rightfoot, leftfoot?)
% iTrial = trial 1, 2 or 3?

path_delim = "\"; % use "\" on PC and "/" on mac
path_prefix = "data" + path_delim + "P01" + path_delim + "RawData" + ...
    path_delim + "Level Ground" + path_delim;

use_plate2 = false;


if iTrialType == 1 % walking
    if iSpeed == 1 % slow
        type_name = "LG_Walk_Slow";
        trial_label = ["Slower01", "Slower02", "Slower03"];
        path_main =  path_prefix + type_name + path_delim + 'LG_Walk_'+ trial_label(iTrial);
        path1 = path_main +'_filtered.mot';
        path2 = path_main +'_ik.mot';
        path3 = path_main +'_id.sto';

        mot_file = importdata(path1);
        IK_file = importdata(path2);
        ID_file = importdata(path3);

        if iTrial == 2
            use_plate2 = true;
        end


    elseif iSpeed == 2 % self selected
        type_name = "LG_Walk_SelfSelected";
        trial_label = ["Normal09", "Normal10", "Normal12"];
        path_main = path_prefix + type_name + path_delim + 'LG_Walk_'+ trial_label(iTrial);
        path1 = path_main +'_filtered.mot';
        path2 = path_main +'_ik.mot';
        path3 = path_main +'_id.sto';

        mot_file = importdata(path1);
        IK_file = importdata(path2);
        ID_file = importdata(path3);

        if iTrial == 2 || iTrial == 3
            use_plate2 = true;
        end

    elseif iSpeed == 3 % fast
        type_name = "LG_Walk_Fast";
        trial_label = ["Faster01", "Faster02", "Faster04"];
        path_main = path_prefix + type_name + path_delim + 'LG_Walk_'+ trial_label(iTrial);
        path1 = path_main +'_filtered.mot';
        path2 = path_main +'_ik.mot';
        path3 = path_main +'_id.sto';

        mot_file = importdata(path1);
        IK_file = importdata(path2);
        ID_file = importdata(path3);

        if iTrial == 2 || iTrial == 3
            use_plate2 = true;
        end
    end

elseif iTrialType == 2 % stand to walk
    type_name = "LG_Stand_to_Walk";
    foot_start = "RightFootStart";
    trial_label = ["Walk01", "Walk02", "Walk04"];
    path_main = path_prefix + type_name +path_delim+foot_start(iSpeed)+path_delim+'LG_Stand_to_'+trial_label(iTrial);
    path1 = path_main +'_filtered.mot';
    path2 = path_main +'_ik.mot';
    path3 = path_main +'_id.sto';

    mot_file = importdata(path1);
    IK_file = importdata(path2);
    ID_file = importdata(path3);

elseif iTrialType == 3 % sit to stand to walk\
    type_name = "LG_Sit_Stand_Walk";
    if iSpeed == 1
        foot_start = "RightFootStart";
        trial_label = ["Walk01", "Walk02", "Walk03"];
        path_main = path_prefix + type_name +path_delim+foot_start+'/LG_Sit_Stand_'+trial_label(iTrial);
        path1 = path_main +'_filtered.mot';
        path2 = path_main +'_ik.mot';
        path3 = path_main +'_id.sto';


        mot_file = importdata(path1);
        IK_file = importdata(path2);
        ID_file = importdata(path3);

    elseif iSpeed == 2
        foot_start = "LeftFootStart";
        trial_label = ["Walk05", "Walk07", "Walk08"];
        path_main = path_prefix + type_name +path_delim+foot_start+'/LG_Sit_Stand_'+trial_label(iTrial);
        path1 = path_main +'_filtered.mot';
        path2 = path_main +'_ik.mot';
        path3 = path_main +'_id.sto';

        mot_file = importdata(path1);
        IK_file = importdata(path2);
        ID_file = importdata(path3);
    end
end

% after loading the .mot file, extract the needed data:
mot_data = mot_file.data;
mot_headers = mot_file.colheaders;

IK_data = IK_file.data;
IK_headers = IK_file.colheaders;

ID_data = ID_file.data;
ID_headers = ID_file.colheaders;

% Extract time vector
time = mot_data(:, strcmp(mot_headers,...
    'time'));

% Get indices for GRF and COP columns to create the vectors

if use_plate2 == false % force plate 1 for right foot
    % Get indices for GRF and COP columns from plate 1
    % vertical force
    vy1_idx = find(strcmp(mot_headers, 'ground_force1_vy'));
    % pressure centers
    px1_idx = find(strcmp(mot_headers, 'ground_force1_px'));
    py1_idx = find(strcmp(mot_headers, 'ground_force1_py'));
    pz1_idx = find(strcmp(mot_headers, 'ground_force1_pz'));

    % create force and COP vectors
    F1 = mot_data(:,vy1_idx);  % N
    COP1 = mot_data(:, [px1_idx, py1_idx, pz1_idx]);  

% Measured OpenSim torques
    tau_hip  = ID_data(:, strcmp(ID_headers, 'hip_flexion_r_moment'));
    tau_knee = ID_data(:, strcmp(ID_headers, 'knee_angle_r_moment'));
    tau_ankle = ID_data(:, strcmp(ID_headers, 'ankle_angle_r_moment'));

    % Extract ankle angle (right leg)
    hip_r   = deg2rad(IK_data(:, strcmp(IK_headers, 'hip_flexion_r')));
    knee_r  = deg2rad(IK_data(:, strcmp(IK_headers, 'knee_angle_r')));
    ankle_r = deg2rad(IK_data(:, strcmp(IK_headers, 'ankle_angle_r')));

    Q = [hip_r, knee_r, ankle_r];  % Nx3

    dt = mean(diff(time));

    dq1 = gradient(Q(:,1), dt);  % hip velocity
    dq2 = gradient(Q(:,2), dt);  % knee velocity
    dq3 = gradient(Q(:,3), dt);  % ankle velocity
    dQ = [dq1, dq2, dq3];  % reassemble

    ddq1 = gradient(dq1, dt);
    ddq2 = gradient(dq2, dt);
    ddq3 = gradient(dq3, dt);
    ddQ = [ddq1, ddq2, ddq3];

    filt_cut = 10;
    dQ = lowpass(dQ,filt_cut,1/dt);
    ddQ = lowpass(ddQ,filt_cut,1/dt);

    % approximate Ankle Position:
    % find index of first non-zero F_y instance
    idx = find(F1(:), 1, 'first');  
    %idx_2 = find(F1(:), 1, 'last');  % returns 4

    % estimate Ankle position (First point of contact with the plate):
    Ankle_pos = COP1(idx,:);

    % compute arm (r) and torque values:
    r_ankle = zeros(size(COP1,1),1);
    tau_GRF_all = zeros(size(COP1,1),1);
    for i = 1: size(COP1,1)
        r_ankle(i) = abs(COP1(i,3)- Ankle_pos(3));

        tau_GRF_all(i) = r_ankle(i)*F1(i);
    end

else % force plate 2 for left foot
    % vertical force
    vy2_idx = find(strcmp(mot_headers, 'ground_force2_vy'));
    % pressure centers
    px2_idx = find(strcmp(mot_headers, 'ground_force2_px'));
    py2_idx = find(strcmp(mot_headers, 'ground_force2_py'));
    pz2_idx = find(strcmp(mot_headers, 'ground_force2_pz'));

    % get force and COP vectors for both plates
    F2 = mot_data(:, vy2_idx);  % N
    COP2 = mot_data(:, [px2_idx, py2_idx, pz2_idx]);  % m


    % approximate Ankle Position:
    % find index of first non-zero F_y instance
    idx = find(F2(:), 1, 'first');
    %idx2 = find(F2(:), 1, 'last');

    % estimate Ankle position (First point of contact with the plate):
    Ankle_pos = COP2(idx,:);

    % compute arm (r) and torque values:
    r_ankle = zeros(size(COP2,1),1);
    tau_GRF_all = zeros(size(COP2,1),1);
    for i = 1: size(COP2,1)
        r_ankle(i) = COP2(i,3)- Ankle_pos(3);

        tau_GRF_all(i) = abs(r_ankle(i)*F2(i));
    end
end

    

% Keep every 10th sample (downsampling to match theta)
%tau_GRF_down = downsample(tau_GRF_all, 10);
% tau_GRF_down_all = [zeros(height(tau_GRF_down),1) zeros(height(tau_GRF_down),1) tau_GRF_down];
% 
% tau_GRF_down2 = downsample(tau_GRF_all, 10);
% tau_GRF_down_all2 = [zeros(height(tau_GRF_down2),1) zeros(height(tau_GRF_down),1) tau_GRF_down];

%% Check dynamics:

% Extract time vector
% time2 = ID_data(:, strcmp(ID_headers,...
%     'time'));
% 
% n = size(Q, 1);  % Number of time steps
% tau_model = zeros(n, 3);  % Estimated torque for each joint

% [M_all, C_all, G_all] = getMCG(Q,dQ);
% 
% for i = 1:n
%     M = M_all(:, :, i);
%     C = C_all(:, :, i);
%     G = G_all(i, :).';
%     tau_GRF = tau_GRF_down_all(i,:);
% 
%     ddq_i = ddQ(i, :).';
%     dq_i  = dQ(i, :).';
% 
%     % Estimate joint torques using dynamics equation
%     tau_model(i, :) = (M * ddq_i + C * dq_i + G - tau_GRF').';
% end

% tau_meas = [tau_hip, tau_knee, tau_ankle];  % Nx3
% % tau_ankle_model = tau_model(:,3);  % Only 3rd joint
% 
% % Plot: Measured vs. Estimated Ankle Torque
% figure;
% plot(time2, tau_ankle, 'k', 'LineWidth', 1.5); hold on;
% %plot(time2, tau_ankle_model, '--r', 'LineWidth', 1.5);
% hold on; plot(time,tau_GRF_all)
% xlabel('Time (s)');
% ylabel('Ankle Torque (Nm)');
% legend('Measured \tau_{ankle}', 'External torque');
% title('Measured vs Estimated Right Ankle Torque plate 1');
% grid on;
% 
% figure; 
% plot(time2, tau_ankle, 'k', 'LineWidth', 1.5); hold on;
% hold on; plot(time,tau_GRF_all2)
% %plot(time2, tau_ankle_model2, '--r', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Ankle Torque (Nm)');
% legend('Measured \tau_{ankle}', 'External torque');
% title('Measured vs Estimated Right Ankle Torque plate 2');
% grid on;


time_start = find(time == trial_data.IK.time(1));
time_end = find(time == trial_data.IK.time(end));



% downsampling to match joint value vector size
GRF_torque = tau_GRF_all(time_start:time_end);
GRF_torque = downsample(GRF_torque, 10);
%plot(trial_data.IK.time,GRF_torque); hold on;
%plot(time,F1);
end