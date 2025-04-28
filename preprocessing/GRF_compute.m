function GRF_torque = GRF_compute(iTrialType, iGait, iSpeed, iTrial, trial_data)
% takes input from the preprocess file:
% iTrialType: Walk, Stand-to-walk or Sit-to-stand-to-walk?
% iGait: Right foot or left foot?
% iSpeed = (slow, med, fast?), (rightfoot?), (rightfoot, leftfoot?)
% iTrial = trial 1, 2 or 3?

if iTrialType == 1 % walking
    if iSpeed == 1 % slow
        type_name = "LG_Walk_Slow";
        trial_label = ["Slower01", "Slower02", "Slower03"];
        path = '/Users/ikhlas_mac/Desktop/P01/RawData/Level Ground/'+ type_name + '/LG_Walk_'+ trial_label(iTrial) +'_filtered.mot';
        mot_file = importdata(path);
    elseif iSpeed == 2 % self selected
        type_name = "LG_Walk_SelfSelected";
        trial_label = ["Normal09", "Normal10", "Normal12"];
        path = '/Users/ikhlas_mac/Desktop/P01/RawData/Level Ground/'+ type_name + '/LG_Walk_'+ trial_label(iTrial) +'_filtered.mot';
        mot_file = importdata(path);
    elseif iSpeed == 3 % fast
        type_name = "LG_Walk_Fast";
        trial_label = ["Faster01", "Faster02", "Faster04"];
        path = '/Users/ikhlas_mac/Desktop/P01/RawData/Level Ground/'+ type_name + '/LG_Walk_'+ trial_label(iTrial) +'_filtered.mot';
        mot_file = importdata(path);
    end

elseif iTrialType == 2 % stand to walk
    type_name = "LG_Stand_to_Walk";
    foot_start = "RightFootStart";
    trial_label = ["Walk01", "Walk02", "Walk04"];
    path = '/Users/ikhlas_mac/Desktop/P01/RawData/Level Ground/'+ type_name +'/'+foot_start(iSpeed)+'/LG_Stand_to_'+trial_label(iTrial) +'_filtered.mot';
    mot_file = importdata(path);

elseif iTrialType == 3 % sit to stand to walk\
    type_name = "LG_Sit_Stand_Walk";
    if iSpeed == 1
        foot_start = "RightFootStart";
        trial_label = ["Walk01", "Walk02", "Walk04"];
        path = '/Users/ikhlas_mac/Desktop/P01/RawData/Level Ground/'+ type_name +'/'+foot_start+'/LG_Sit_Stand_'+trial_label(iTrial) +'_filtered.mot';
        mot_file = importdata(path);

    elseif iSpeed == 2
        foot_start = "LeftFootStart";
        trial_label = ["Walk05", "Walk07", "Walk08"];
        path = '/Users/ikhlas_mac/Desktop/P01/RawData/Level Ground/'+ type_name +'/'+foot_start+'/LG_Sit_Stand_'+trial_label(iTrial) +'_filtered.mot';
        mot_file = importdata(path);
    end
end

% after loading the .mot file, extract the needed data:
mot_data = mot_file.data;
mot_headers = mot_file.colheaders;

% Extract time vector
time = mot_data(:, strcmp(mot_headers,...
    'time'));

% Get indices for GRF and COP columns to create the vectors

if iGait == 1 % force plate 1 for right foot
    % Get indices for GRF and COP columns
    % vertical force
    vy1_idx = find(strcmp(mot_headers, 'ground_force1_vy'));
    % pressure centers
    px1_idx = find(strcmp(mot_headers, 'ground_force1_px'));
    py1_idx = find(strcmp(mot_headers, 'ground_force1_py'));
    pz1_idx = find(strcmp(mot_headers, 'ground_force1_pz'));

    % create force and COP vectors
    F1 = mot_data(:,vy1_idx);  % N
    COP1 = mot_data(:, [px1_idx, py1_idx, pz1_idx]);  


    %%
    % approximate Ankle Position:
    % find index of first non-zero F_y instance
    idx_1 = find(F1(:), 1, 'first');  
    idx_2 = find(F1(:), 1, 'last');  % returns 4

    % estimate Ankle position (First point of contact with the plate):
    Ankle_pos_1 = COP1(idx_1,:);
    Ankle_pos_2 = COP1(idx_2,:);

    % compute arm (r) and torque values:
    r_ankle = zeros(size(COP1,1),1);
    tau_GRF_all = zeros(size(COP1,1),1);
    for i = 1: size(COP1,1)
        %r_ankle(i) = COP1(i,3)- Ankle_pos_1(3);
        r_ankle(i) = COP1(i,3)- Ankle_pos_2(3);

        tau_GRF_all(i) = r_ankle(i)*F1(i);
    end

elseif iGait == 2 % force plate 2 for left foot
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
    idx2 = find(F2(:), 1, 'last');

    % estimate Ankle position (First point of contact with the plate):
    Ankle_pos = COP2(idx,:);

    % compute arm (r) and torque values:
    r_ankle = zeros(size(COP2,1),1);
    tau_GRF_all = zeros(size(COP2,1),1);
    for i = 1: size(COP2,1)
        r_ankle(i) = COP2(i,3)- Ankle_pos(3);
        r_ankle(i) = COP2(i,3)- Ankle_pos(3);

        tau_GRF_all(i) = r_ankle(i)*F2(i);
    end
end

plot(time,F1)
hold on; plot(time,tau_GRF_all)

time_start = find(time == trial_data.IK.time(1));
time_end = find(time == trial_data.IK.time(end));

% downsampling to match joint value vector size
GRF_torque = tau_GRF_all(time_start:time_end);
GRF_torque = downsample(GRF_torque, 10);
plot(trial_data.IK.time,GRF_torque); hold on;
plot(time,F1);
end