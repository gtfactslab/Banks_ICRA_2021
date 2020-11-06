clear; close all;

% control system sampling and simulation time
Ts = 0.02;
times = 0:Ts:25;
simulationSteps = length(times);

%bebop model
initialState = [0 0 1 0 0 0 0 0 0 0 0 0]';
bebop = Bebop2(initialState);
ueq=bebop.omegaHover*ones(4,1);
forcesEq = [bebop.m*bebop.g 0 0 0]';
seq=[0 0 1 0 0 0 0 0 0 0 0 0]';
%seq=initialState;

% 2 CONTROLLERS
% controller for taking off and landing
ctr = SubSysController(bebop.m,bebop.I,Ts);
xref=1; yref=0; zref=1.5; psiref=0;
ctr.setRef([xref yref zref]'-seq(1:3), psiref);

%controller for trajectory tracking
%TODO



%state and forces filter

initialStateGuess = zeros(16,1);
initialStateGuess(1:3)=initialState(1:3)';

stateForcesFilter=unscentedKalmanFilter(@stateForcesFilterTransitionNOvel,...
    @stateFilterMeas,initialStateGuess);
% initial state estimation error covariance
% choose a low value if you have a good confidence on the initialStateGuess
stateForcesFilter.StateCovariance=0.001; %the software uses the scalar value to create an Nx-by-Nx diagonal matrix.

stateForcesFilter.ProcessNoise= 1e-4*eye(16);
%stateForcesFilter.ProcessNoise(9,9)=0.01;
sigmaPosition = 0.001;
sigmaAttitude = 0.05;
R = diag( [ones(3,1)*sigmaPosition^2 ; ones(3,1)*sigmaAttitude^2; ] );
stateForcesFilter.MeasurementNoise=R;


%the filter story
yMeas = zeros(6,simulationSteps);
stateForcesResidBuf = zeros(6,simulationSteps);
stateForcesXcorBuf = zeros(16,simulationSteps);
stateForcesXpredBuf = zeros(16,simulationSteps);

%admittance controller
virtualMasses = 1e-1*[2 2 2 40]'; 
virtualDampings = 1e-1*[2 2 2 20]';  
adCtr=[];


deltaOmega = zeros(4,simulationSteps);

takingOff=true;
tol=5e-2;
forceExt=[-0.03 0.02 -0.1]'; torqueExt=0.01;
forceTime=[2 4];
for i=1:simulationSteps
    fprintf('time: %1.4f \n',times(i));
    curState = bebop.s(:,end);
    
    yMeas(:,i) = curState([1 2 3 7 8 9]).* (1+sqrt(R)*randn(6,1)); % sqrt(R): Standard deviation of noise
    
    if takingOff
        near =   norm(yMeas(1,i)-xref)<tol &&...
            norm(yMeas(2,i)-yref)<tol &&...
            norm(yMeas(3,i)-zref)<tol &&...
            norm(yMeas(6,i)-psiref)<tol;
        if near            
            adCtr = AdmittanceCtrlComplete( [ [curState(1:3)',curState(9)]; zeros(1,4)],...
                virtualMasses, virtualDampings, Ts);
            
            takingOff=false;
            switchStep=i;
            fprintf('tracking starting from step %d\n',switchStep);
        end
    end
    
    if ~takingOff       
        
        [Residual,ResidualCovariance] = residual(stateForcesFilter,yMeas(:,i));
        stateForcesResidBuf(:,i) = Residual;
        [correctedState,CorrectedStateCovariance] = correct(stateForcesFilter,yMeas(:,i));
        stateForcesXcorBuf(:,i) = correctedState;
        
        deltaState = stateForcesXcorBuf(1:12,i)-seq; %.* (1+sqrt(0.5)*randn(12,1));
        deltaCommands = ctr.update(deltaState);
        commands = deltaCommands+forcesEq;
        deltaOmega(:,i) = 1/(2*bebop.omegaHover)*(bebop.allocMat\deltaCommands);
        
        if (i-switchStep)*Ts>=forceTime(1) && (i-switchStep)*Ts<=forceTime(2)
            bebop.applyConstOmegaForceTorque(deltaOmega(:,i)+bebop.omegaHover,forceExt,[0 0 torqueExt]',Ts);
        else
            bebop.applyConstOmega(deltaOmega(:,i)+bebop.omegaHover,Ts);
        end
        adCtr.update(stateForcesXcorBuf(13:16,i));
        ref = adCtr.getRef(); refpos=ref(1:3); psiref=ref(4);
        ctr.setRef( refpos-seq(1:3),psiref);
        
        
        [predictedState,PredictedStateCovariance] = predict(stateForcesFilter,...
            commands,Ts,bebop.m,bebop.I);
        stateForcesXpredBuf(:,i) = predictedState;
    else

        
        [Residual,ResidualCovariance] = residual(stateForcesFilter,yMeas(:,i));
        stateForcesResidBuf(:,i) = Residual;
        [correctedState,CorrectedStateCovariance] = correct(stateForcesFilter,yMeas(:,i));
        stateForcesXcorBuf(:,i) = correctedState;
        
        
        
        deltaState = stateForcesXcorBuf(1:12,i)-seq; %.* (1+sqrt(0.5)*randn(12,1));
        deltaCommands = ctr.update(deltaState);
        commands = deltaCommands+forcesEq;
        deltaOmega(:,i) = 1/(2*bebop.omegaHover)*(bebop.allocMat\deltaCommands);
        bebop.applyConstOmega(deltaOmega(:,i)+bebop.omegaHover,Ts);
        
        [predictedState,PredictedStateCovariance] = predict(stateForcesFilter,...
            commands,Ts,bebop.m,bebop.I);
        stateForcesXpredBuf(:,i) = predictedState;
        
    end
    
    
end

bebop.plotState();
bebop.plotInputs();
adCtr.plotOutputs();

%%
fig1=figure();
%position
subplot(3,4,1);
plot(times,stateForcesXpredBuf(1,:));hold on;
plot(bebop.t,bebop.s(1,:));
plot(times,yMeas(1,:));
title('positions')
legend('filter','true','measure','Location','best')

subplot(3,4,5);
plot(times,stateForcesXpredBuf(2,:));hold on
plot(bebop.t,bebop.s(2,:));
plot(times,yMeas(2,:));


subplot(3,4,9);
plot(times,stateForcesXpredBuf(3,:)); hold on
plot(bebop.t,bebop.s(3,:));hold on
plot(times,yMeas(3,:));

%velocities
subplot(3,4,2);
plot(times,stateForcesXpredBuf(4,:));hold on
plot(bebop.t,bebop.s(4,:));hold on
title('velocities')
subplot(3,4,6);
plot(times,stateForcesXpredBuf(5,:));hold on
plot(bebop.t,bebop.s(5,:));hold on

subplot(3,4,10);
plot(times,stateForcesXpredBuf(6,:));hold on
plot(bebop.t,bebop.s(6,:));


%angles in degrees
subplot(3,4,3);
plot(times,rad2deg(stateForcesXpredBuf(7,:)));hold on;
plot(bebop.t,bebop.s(7,:));
plot(times,rad2deg(yMeas(4,:)));
ylabel('deg')
title('angles')
subplot(3,4,7);
plot(times,rad2deg(stateForcesXpredBuf(8,:)));hold on
plot(bebop.t,bebop.s(8,:));hold on
plot(times,rad2deg(yMeas(5,:)));
ylabel('deg')
subplot(3,4,11);
plot(times,rad2deg(stateForcesXpredBuf(9,:)));hold on
plot(bebop.t,bebop.s(9,:));hold on
plot(times,rad2deg(yMeas(6,:)));
ylabel('deg')

% angular velocities
subplot(3,4,4);
plot(times,rad2deg(stateForcesXpredBuf(10,:)));hold on;
plot(bebop.t,bebop.s(10,:));hold on
ylabel('deg/s')
title('angular velocities')
subplot(3,4,8);
plot(times,rad2deg(stateForcesXpredBuf(11,:))); hold on;
plot(bebop.t,bebop.s(11,:));hold on
ylabel('deg/s')
subplot(3,4,12);
plot(times,rad2deg(stateForcesXpredBuf(12,:)));hold on
plot(bebop.t,bebop.s(12,:));hold on
ylabel('deg/s')


%%
figure()
plot(times,stateForcesXpredBuf(16,:)); grid on; hold on
legend('Mz')

fig(2) = figure();
switchTime=switchStep*Ts;

stairs(times,stateForcesXpredBuf(13,:)','r'); grid on; hold on;
stairs(times,stateForcesXpredBuf(14,:)','b');
stairs(times,stateForcesXpredBuf(15,:)','g');

hold on; grid on;
plot(linspace(switchTime+forceTime(1),switchTime+forceTime(2),10),...
    ones(1,10)*forceExt(1),'--r','LineWidth',2);
plot(linspace(switchTime+forceTime(1),switchTime+forceTime(2),10),...
    ones(1,10)*forceExt(2),'--b','LineWidth',2);
plot(linspace(switchTime+forceTime(1) ,switchTime+forceTime(2),10),...
    ones(1,10)*forceExt(3),'--g','LineWidth',2);

ax=axis;
plot(ones(1,10)*(switchTime+forceTime(1)),linspace(ax(3),ax(4),10),'k--')
plot(ones(1,10)*(switchTime+forceTime(2)),linspace(ax(3),ax(4),10),'k--')
ylabel('N'); xlabel('s'); title('Estimated forces for bebop')
legend('x est','y est','z est','x ext force','y ext force','z ext force','Location','best');
