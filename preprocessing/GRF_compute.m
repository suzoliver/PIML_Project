% TODO: rename files to fit code convention

function GRF_torque = GRF_compute(iTrialType, iGait, iSpeed, iTrial, trial_data, sbj_num)
% takes input from the preprocess file:
% iTrialType: Walk, Stand-to-walk or Sit-to-stand-to-walk?
% iGait: ght foot or left foot?
% iSpeed = (slow, med, fast?), (rightfoot?), (rightfoot, leftfoot?)
% iTrial = trial 1, 2 or 3?

path_delim = "/"; % use "\" on PC and "/" on mac
path_prefix = "data" + path_delim + "P0' + string(sbj_num)" + path_delim + "RawData" + ...
    path_delim + "Level Ground" + path_delim;

%% Select file:

if iTrialType == 1 % walking
    if iSpeed == 1 % slow
        type_name = "LG_Walk_Slow";
        trial_label = ["Slower01", "Slower02", "Slower03"];
        path_main =  path_prefix + type_name + path_delim + 'LG_Walk_'+ trial_label(iTrial);

    elseif iSpeed == 2 % self selected
        type_name = "LG_Walk_SelfSelected";
        trial_label = ["Normal01", "Normal02", "Normal03"];
        path_main = path_prefix + type_name + path_delim + 'LG_Walk_'+ trial_label(iTrial);

    elseif iSpeed == 3 % fast
        type_name = "LG_Walk_Fast";
        trial_label = ["Faster01", "Faster02", "Faster03"];
        path_main = path_prefix + type_name + path_delim + 'LG_Walk_'+ trial_label(iTrial);
    end

elseif iTrialType == 2 % stand to walk
    type_name = "LG_Stand_to_Walk";
    foot_start = "RightFootStart";
    trial_label = ["Walk01", "Walk02", "Walk03"];
    path_main = path_prefix + type_name +path_delim+foot_start(iSpeed)+path_delim+'LG_Stand_'+trial_label(iTrial);


elseif iTrialType == 3 % sit to stand to walk
    type_name = "LG_Sit_Stand_Walk";
    if iSpeed == 1
        foot_start = "RightFootStart";
        trial_label = ["Walk01", "Walk02", "Walk03"];
        path_main = path_prefix + type_name +path_delim+foot_start+'/LG_Sit_Stand_'+trial_label(iTrial);

    elseif iSpeed == 2
        foot_start = "LeftFootStart";
        trial_label = ["Walk01", "Walk02", "Walk03"];

        path_main = path_prefix + type_name +path_delim+foot_start+'/LG_Sit_Stand_'+trial_label(iTrial);
    end
end

path1 = path_main +'_filtered.mot'; % forces
path2 = path_main +'_external_loads.xml'; % plate number

mot_file = importdata(path1);

%% extract needed data:
mot_data = mot_file.data;
mot_headers = mot_file.colheaders;

% Extract time vector
time = mot_data(:, strcmp(mot_headers,...
    'time'));

% check if right foot is on plate 2
% Load XML
xmlFile = path2;
doc = xmlread(xmlFile);

externalForces = doc.getElementsByTagName('ExternalForce');

% Initialize
use_plate2 = NaN;

% Loop through ExternalForce entries
for k = 0:externalForces.getLength-1
    forceNode = externalForces.item(k);

    % Get the force_identifier tag value
    force_id = char(forceNode.getElementsByTagName('force_identifier').item(0).getTextContent);

    % Check if it's ground_force2_v
    if strcmp(force_id, 'ground_force2_v')
        applied_to_body = char(forceNode.getElementsByTagName('applied_to_body').item(0).getTextContent);

        % Set flag depending on body
        if strcmp(applied_to_body, 'calcn_r')
            use_plate2 = true;
        elseif strcmp(applied_to_body, 'calcn_l')
            use_plate2 = false;
        end
        break;  % No need to continue loop
    end
end


if use_plate2 == false % foot is on force plate 1

    % Fy
    vy1_idx = find(strcmp(mot_headers, 'ground_force1_vy'));
    % pressure centers
    px1_idx = find(strcmp(mot_headers, 'ground_force1_px'));
    py1_idx = find(strcmp(mot_headers, 'ground_force1_py'));
    pz1_idx = find(strcmp(mot_headers, 'ground_force1_pz'));

    % create force and COP vectors
    F = mot_data(:,vy1_idx);  % N
    COP = mot_data(:, [px1_idx, py1_idx, pz1_idx]);

else % foot is on force plate 2
    % Fy
    vy2_idx = find(strcmp(mot_headers, 'ground_force2_vy'));
    % pressure centers
    px2_idx = find(strcmp(mot_headers, 'ground_force2_px'));
    py2_idx = find(strcmp(mot_headers, 'ground_force2_py'));
    pz2_idx = find(strcmp(mot_headers, 'ground_force2_pz'));

    % get force and COP vectors for both plates
    F = mot_data(:, vy2_idx);  % N
    COP = mot_data(:, [px2_idx, py2_idx, pz2_idx]);  % m

end

%% approximate Ankle Position:

% find index of first non-zero Fy instance
idx = find(F(:), 1, 'first');

% estimate Ankle position (First point of contact):
Ankle_pos = COP(idx,:);

% compute arm (r) and torque values:
r_ankle = zeros(size(COP,1),1);
tau_GRF_all = zeros(size(COP,1),1);
for i = 1: size(COP,1)
    r_ankle(i) = abs(COP(i,3)- Ankle_pos(3));
    tau_GRF_all(i) = r_ankle(i)*F(i);
end


time_start = find(time == trial_data.IK.time(1));
time_end = find(time == trial_data.IK.time(end));


% downsampling to match joint value vector size
GRF_torque = tau_GRF_all(time_start:time_end);
GRF_torque = downsample(GRF_torque, 10);

end