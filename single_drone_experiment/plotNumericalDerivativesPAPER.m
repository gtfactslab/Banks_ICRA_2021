clear; close all;

testAdCtrlComplete;
%%
time=adCtr.getTimes();
d0=adCtr.getPositions();
d1=adCtr.getDer1();
d2=adCtr.getDer2();
d2f=adCtr.getDer2Filtered();
d3f=adCtr.getDer3Filtered();

figure
stairs(time,d1(1,:),'b','LineWidth',1.5); hold on; grid on;

stairs(time,d2(1,:),'g','LineWidth',1.5)
stairs(time,d2f(1,:),'r','LineWidth',1.5)
stairs(time,d3f(1,:),'k','LineWidth',1.5)
xlabel('Time [s]');
xaxis([0 12])

legend('first derivative','second derivative',...
    'second filtered derivative','third filtered derivative','Location','southeast')
title('x-axis position discrete derivatives')
