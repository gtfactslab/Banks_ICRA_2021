function plot_pose_and_orientation()
fig1=figure();
% ax = gca;
% ax.TickLength=[0.01, 0.15];
T = load('times.mat');
S = load('prediction_ekf_interaction_9_yaw.mat');
Y = load(' measured_interaction_9_yaw.mat');
P = load('p_all_arr_interaction_9_yaw.mat');
A = load('ang_thr_interaction_9_yaw.mat');

subplot(2, 3, 1);

stateForcesXpredBuf = S.stateForcesXpredBuf;
times = T.times;
yMeas = Y.yMeas;
angles_n_thrust = A.angles_n_thrust;
p_all = P.p_all;
times;
plot(times,stateForcesXpredBuf(1,:), 'LineWidth', 5);hold on;
plot(times,yMeas(1,:), 'LineWidth', 5);
plot(times(260:end), p_all(1, 260:end), 'LineWidth', 5);
plot(ones(1,20)*(5),linspace(-4, 4,20),'k--', 'LineWidth', 5);
xlabel('Time(s)', 'FontSize', 25);
ylabel('X-position (m)', 'FontSize', 25);
title('Positions', 'FontSize', 25)
yticks([-4, -3.5, -3, -2.5, -2, -1.5, -1,-0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4])
legend('Filter','Measured', 'Desired', 'FontSize', 25)

subplot(2, 3, 2);
plot(times,stateForcesXpredBuf(2,:), 'LineWidth', 5);hold on
plot(times,yMeas(2,:), 'LineWidth', 5);
plot(times(260:end), p_all(2, 260:end), 'LineWidth', 5);
plot(ones(1,10)*(5),linspace(-2,1,10),'k--', 'LineWidth', 5);
xlabel('Time(s)', 'FontSize', 25);
ylabel('Y-position (m)', 'FontSize', 25);
title('Positions', 'FontSize', 25);
yticks([-2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2])
legend('Filter','Measured', 'Desired', 'FontSize', 25);


subplot(2, 3, 3);

plot(times,stateForcesXpredBuf(3,:), 'LineWidth', 5); hold on
plot(times,yMeas(3,:), 'LineWidth', 5);
plot(times(270:end), p_all(3, 270:end), 'LineWidth', 5);
plot(ones(1,20)*(5),linspace(0,2,20),'k--', 'LineWidth', 5);
xlabel('Time(s)', 'FontSize', 25);
ylabel('Z-position (m)', 'FontSize', 25);
title('Positions', 'FontSize', 25)
yticks([-1, -0.5, 0, 0.5, 1, 1.5, 2])
legend('Filter','Measured', 'Desired', 'FontSize', 25)


%angles in degrees
subplot(2, 3, 4);
plot(times,rad2deg(stateForcesXpredBuf(7,:)), 'LineWidth',5);hold on;
plot(times,rad2deg(yMeas(4,:)), 'LineWidth', 5);
plot(times(250:end), rad2deg(angles_n_thrust(1, 250:end)), 'LineWidth', 5);
plot(ones(1,10)*(5),linspace(-10,5,10),'k--', 'LineWidth', 5)
title('Angles', 'FontSize', 25);
ylabel('Roll (degrees)', 'FontSize', 25);
yticks([-10, -5, 0, 5, 10])
xlabel('Time (s)', 'FontSize', 25);



subplot(2, 3, 5);
plot(times,rad2deg(stateForcesXpredBuf(8,:)), 'LineWidth', 5);hold on;
plot(times,rad2deg(yMeas(5,:)), 'LineWidth', 5);
plot(times(250:end), rad2deg(angles_n_thrust(2, 250:end)), 'LineWidth', 5);
plot(ones(1,10)*(5),linspace(-30,30,10),'k--', 'LineWidth', 5)
title('Angles', 'FontSize', 25);
ylabel('Pitch (degrees)', 'FontSize', 25);
xlabel('Time (s)', 'FontSize', 25);

subplot(2, 3, 6);
plot(times,rad2deg(stateForcesXpredBuf(9,:)), 'LineWidth', 5);hold on;
plot(times,rad2deg(yMeas(6,:)), 'LineWidth', 5);
plot(times(250:end), rad2deg(angles_n_thrust(3, 250:end)), 'LineWidth', 5);
plot(ones(1,10)*(5),linspace(-10,10,10),'k--', 'LineWidth', 5)
title('Angles', 'FontSize', 25);
ylabel('Yaw (degrees)', 'FontSize', 25);
yticks([-10, -5, 0, 5, 10])
xlabel('Time (s)', 'FontSize', 25);
end