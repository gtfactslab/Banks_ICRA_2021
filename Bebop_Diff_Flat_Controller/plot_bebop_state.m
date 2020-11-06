function plot_bebop_state(bebop_class)

size(bebop_class.t)
size(bebop_class.s)
% plot pose
hold on
plot(bebop_class.t, bebop_class.s(1:3, :))
title("Bebop Pose")
legend('x', 'y', 'z')
hold off

%plot velocity
hold on
figure
plot(bebop_class.t, bebop_class.s(4:6, :))
title("Bebop Velocity")
legend('v_x', 'v_y', 'v_z')
hold off
%plot orientation
hold on
figure
plot(bebop_class.t, bebop_class.s(7:9, :))
title("Bebop Orientation")
legend('phi', 'theta', 'psi')
hold off


end