The CNN_physics file can be run in google with an GPU enabled (T4 used during testing).

The first section clones this github, which gives access to all necessary data files.

To run for a specific subject:
  1. Update the dataDir (file path) in the first Prep Data block 
  2. Update iSubj parameter in custom_loss function 

To update run with or without physics informed term, update the w_phys parameters in the custom_loss function.
0.25 is used to run with physics and 0 is used to run without physics.
