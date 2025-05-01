Each folder contains data for a specific subject.
Within these folders are two subfolders: CSVs and RawData.

CSVs contains the output of the CleanExtractData.m file for that subject, including: 
- EMG files: clean EMG data from each trial
- Joint Files: joint kinematics for all joints, true ankle moment, and external torque
For all EMG files, there is a matching Joint file of the same trial.

The columns of the EMG files represent the 64 channels of EMG data
The row of the EMG files represent different time steps, sampled at 2000 Hz

The columns of the joint files are organized as:
[Hip Angle, Knee Angle, Ankle Angle, Hip Velocity, Knee Velocity, Ankle Velocity, Hip Acceleration, Knee Acceleration, Ankle Acceleration, Ankle Torque, External Torque]
The rows of the Joint files represent different time steps, sampled at 100 Hz
