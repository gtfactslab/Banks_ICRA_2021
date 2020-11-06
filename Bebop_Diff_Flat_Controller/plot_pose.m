function plot_pose(params_plot, i, j)
    figure(i)
    t= 1:j;
%     subplot(2, 1, 1);
    hold on
    plot(t, params_plot.robo_pose(t, 1));
    plot(t, params_plot.robo_pose(t, 2));
    plot(t, params_plot.robo_pose(t, 3));    
    title("Odom XYZ coords")
    legend("X", "Y", "Z");
    hold off
    
%     subplot(2, 1, 2);
%     hold on
%     plot(t, params_plot.world_robo_pose(t, 1));
%     plot(t, params_plot.world_robo_pose(t, 2));
%     plot(t, params_plot.world_robo_pose(t, 3));    
%     title("World XYZ coords")
%     legend("X", "Y", "Z");
%     hold off
   
    
    figure(i+2)
%     subplot(2, 1, 1);
    hold on
    plot(t, params_plot.robo_pose(t, 4));
    plot(t, params_plot.robo_pose(t, 5));
    plot(t, params_plot.robo_pose(t, 6));
    title("Odom Angles")
    legend("Roll", "Pitch", "Yaw");
    hold off
    
%     subplot(2, 1, 2);
%     hold on
%     plot(t, params_plot.world_robo_pose(t, 4));
%     plot(t, params_plot.world_robo_pose(t, 5));
%     plot(t, params_plot.world_robo_pose(t, 6));
%     title("World Angles")
%     legend("Roll", "Pitch", "Yaw");
%     hold off
    
end