close all; clear; clc;

sbj_num = 2; % change for diff subject

% load P0sbj_num in your environment.
eval(sprintf("load('P0%d.mat')", sbj_num));
sbj_data = eval(sprintf('P0%d.Participant_Information', sbj_num));

% mass (kg)
m_tot = sbj_data.Weight;  % kg
m_tot = str2double(erase(m_tot, 'kg'));

% height (m)
h = sbj_data.Height;
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
% I1 = m1 * (r1 * L1)^2;
% I2 = m2 * (r2 * L2)^2;
% I3 = m3 * (r3 * L3)^2;

% Convert scalar inertia to full 3D form (only Iyy is nonzero)
%Ivec = @(Iyy) [0 Iyy 0 0 0 0];  % [Ixx Iyy Izz Ixy Iyz Ixz]

