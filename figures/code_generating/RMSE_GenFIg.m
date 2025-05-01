%% Generate bar plot

%order: angle no phys, angle w phys, moment no phys, moment with phys
rmse_values = table2array(readtable("PIML_RMSE.csv"));

rmse_angle = rmse_values(:,1:2);
rmse_moment = rmse_values(:,3:4);

subj_nums = [1,3,4,7,8];

figure('Position', [200,200,500,650])
tiledlayout(2,1)

nexttile
hold on
bar(rmse_angle)
%xlabel("Subjects")
xticks(1:5);
xticklabels(subj_nums);
ylabel("Angle RMSE (°)")
legend("No Physics Term", "With Physics Term")
set(gca, 'FontWeight', 'bold', 'FontSize', 14, 'FontName', 'Arial')



nexttile
hold on
bar(rmse_moment)
xlabel("Subjects")
xticks(1:5);
xticklabels(subj_nums);
ylabel("Torque RMSE (Nm)")
set(gca, 'FontWeight', 'bold', 'FontSize', 14, 'FontName', 'Arial')

%% Get stats for report

mean(rmse_values)
std(rmse_values)
ttest(rmse_angle(:,1), rmse_angle(:,2))
ttest(rmse_moment(:,1), rmse_moment(:,2))