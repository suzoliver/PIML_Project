
gait_list = ["RightFoot", "LeftFoot"];
gait_file_save_name = ["R", "L"];

trial_type_list = ["Walk", "StandWalk","SitStandWalk"];
iTrialType = 2; % set the type of trial to use

if iTrialType == 1
    % Options for Walk Only Trials
    type_list = ["Slow_Speed", "Self_Selected_Speed", "Fast_Speed"];
    type_file_save_name = ["Slow", "Med", "Fast"];
    type_name = "Walking";
elseif iTrialType == 2
    % Options for Sit to Walk Trials
    type_list = ["Right_Foot_Start"];
    type_file_save_name = ["RStart"];
    type_name = "Stand_to_Walk";
elseif iTrialType == 3
    % Options for Sit to Stand to Walk Trials
    type_list = ["Right_Foot_Start"];
    type_file_save_name = ["RStart_SitStand"];
    type_name = "Sit_to_Stand_to_Walk";
end

% for each trial, load emg data, filter and save as CSV
% then get right ankle data (angle, torque) and save to separate csv 
% use same naming structure for both

for iGait = 1:length(gait_list)
    for iSpeed = 1:length(type_list)
        for iTrial = 1:3

            trial_data = P01.(gait_list(iGait) + "_GaitCycle_Data").Level_Ground.(type_name).(type_list(iSpeed))(iTrial);
            emg_data = trial_data.RightLeg_EMG;

            % Process EMG Data
            fs_EMG = 2000;
            nMuscles = 64;
            
            emg_processed = zeros(size(emg_data)); 
            
            % Butterworth filter between 20 and 500 hz
            filtMax = 500;
            filtMin = 20;
            
            L = length(emg_data);
            f = fs_EMG*(0:(L/2))/L;
            bp_lower = filtMin / (max(f));
            bp_upper = filtMax / (max(f));
            
            [bp_b, bp_a] = butter(4, [bp_lower bp_upper], "bandpass");
            
            for iMuscle = 1:nMuscles
                % Apply bandstop filter at multiples of 60 Hz
                % This accounts for powerline noise
                EMG_filtered = filter(bp_b, bp_a, emg_data(:,iMuscle));
                bs = 60;
                while bs < filtMax
                    bs_upper = (bs+2)/ (max(f));
                    bs_lower = (bs-2)/ (max(f));
                    [bs_b, bs_a] = butter(4, [bs_lower bs_upper], "stop");
                    EMG_filtered = filter(bs_b, bs_a, EMG_filtered);
                    bs = bs + 60;
                end
                emg_processed(:,iMuscle) = EMG_filtered;
            end

            file_name_conv = "_P01_" + trial_type_list(iTrialType) + "_" +...
                gait_file_save_name(iGait) + "_" + ...
                type_file_save_name(iSpeed) + "_T" + iTrial + ".csv";
            writematrix(emg_processed,"EMG_Proc" + file_name_conv)

            % get hip angle + moment and save to csv
            hip_data = zeros(height(trial_data.IK),2);
            hip_data(:,1) = trial_data.IK.hip_flexion_r;
            hip_data(:,2) = trial_data.ID.hip_flexion_r_moment;

            writematrix(hip_data,"Hip" + file_name_conv)

            % get knee angle + moment and save to csv
            knee_data = zeros(height(trial_data.IK),2);
            knee_data(:,1) = trial_data.IK.knee_angle_r;
            knee_data(:,2) = trial_data.ID.knee_r_moment;

            writematrix(knee_data,"Knee" + file_name_conv)

            % now get ankle angle + moment and save to csv
            ankle_data = zeros(height(trial_data.IK),2);
            ankle_data(:,1) = trial_data.IK.ankle_angle_r;
            ankle_data(:,2) = trial_data.ID.ankle_r_moment;

            writematrix(ankle_data,"Ankle" + file_name_conv)
        end
    end
end
