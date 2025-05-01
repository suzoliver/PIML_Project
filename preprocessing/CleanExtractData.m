% Note to run this file, you must have the subject mat file (i.e. P01.mat)
% loaded and the motion files in the path specified by the GRF_compute
% function

iSubj = 1; % change for diff subject

% get path to save files
path_delim = "\"; % use "\" on PC and "/" on mac
save_path = "data" + path_delim + "P0" + string(iSubj) + path_delim + "CSVs" + path_delim;

% get subject data
sbj = eval(sprintf('P0%d', iSubj));

% Loop through trials starting with right foot and left foot
gait_list = ["RightFoot", "LeftFoot"];
gait_file_save_name = ["R", "L"];
 
% three types of trials, all with different naming conventions
% specify all trials types to process
trial_type_list = ["Walk", "StandWalk","SitStandWalk"];
% set trial type index (1-3) to use on this run
iTrialType = 1;

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
    type_list = ["Right_Foot_Start","Left_Foot_Start"];
    type_file_save_name = ["RStart","LStart"];
    type_name = "Sit_to_Stand_to_Walk";
end

% for each trial, load emg data, filter and save as CSV
% then get right leg joint data (angle, torque) and save to separate csv 
% use same naming structure for both

for iGait = 1:length(gait_list)
    for iSpeed = 1:length(type_list)
        for iTrial = 1:3 

            % get data for this trial
            trial_data = sbj.(gait_list(iGait) + "_GaitCycle_Data").Level_Ground.(type_name).(type_list(iSpeed))(iTrial);
            emg_data = trial_data.RightLeg_EMG;

            % EMG data info
            fs_EMG = 2000;
            nMuscles = 64;
            
            emg_processed = zeros(size(emg_data)); 
            
            % build filter for EMG
            % Butterworth filter between 20 and 500 hz
            filtMax = 500;
            filtMin = 20;
            
            L = length(emg_data);
            f = fs_EMG*(0:(L/2))/L;
            bp_lower = filtMin / (max(f));
            bp_upper = filtMax / (max(f));
            
            [bp_b, bp_a] = butter(4, [bp_lower bp_upper], "bandpass");
            
            % iterate through each channel to apply filter
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

            % save out EMG data to csv at specified path
            file_name_conv = "_P0" + string(iSubj) + "_" + trial_type_list(iTrialType) + "_" +...
                gait_file_save_name(iGait) + "_" + ...
                type_file_save_name(iSpeed) + "_T" + iTrial + ".csv";
            writematrix(emg_processed,save_path + "EMG_Proc" + file_name_conv)


            % Now get true joint kinematic data. Save all angles, then all
            % velocities, then all accelerations. Always in order [hip,
            % knee, ankle]. Then col 10 is the true ankle moment and col 11
            % is the external moment on the ankle.

            % time step size for joint kinemetics/dynamics
            dt = 0.01;

            % initalize array to write to csv
            joint_values = zeros(height(trial_data.IK), 11);

            % get joint angles
            joint_values(:,1) = trial_data.IK.('hip_flexion_r');
            joint_values(:,2) = trial_data.IK.('knee_angle_r');
            joint_values(:,3) = trial_data.IK.('ankle_angle_r');
            joint_values(:,1:3) = deg2rad(joint_values(:,1:3));

            % differentiate to get velocities
            joint_values(:,4:6) = [diff(joint_values(:,1:3));[0,0,0]] /dt;

            % differentiate again to get accelerations
            joint_values(:,7:9) = [diff(joint_values(:,4:6));[0,0,0]] /dt;

            % velocity and accelerations are noisy - do low pass filter to
            % get smoother results
            % cut off of 10Hz seems to work fine
            filt_cut = 10;
            joint_values(:,4:9) = lowpass(joint_values(:,4:9),filt_cut,1/dt);

            % now add true joint moment for the ankle only (others not
            % needed since we are only predicting ankle)
            joint_values(:,10) = trial_data.ID.ankle_r_moment;

            % Get external moment on ankle from
            % GRF_torques and put in col 11
            joint_values(:,11) = GRF_compute(iTrialType, iGait, iSpeed, iTrial, trial_data,iSubj);

            writematrix(joint_values,save_path+ "Joints" + file_name_conv)
        end
    end
end
