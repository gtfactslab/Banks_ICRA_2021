clear; close all;

K=1;
Ts=0.01;
T=0.3;



% filtered derivative
%z=tf('z',Ts);
%G = K/T * (z-1)/(z+Ts/T-1);
D = ss( 1-Ts/T, Ts/T, -K/T, K/T, Ts);


% simTime = (0:1000)*Ts;
% 
% dev=0.02;
% f=1.5;
% sinSig = sin(f*simTime);
% sinSigd= f*cos(f*simTime);
% sinSigdd = -f^2 * sin(f*simTime);
% sinSigddd = -f^3 * cos(f*simTime);
% sig = sinSig .* (1+dev*randn(length(sinSig),1))';
% sigd= lsim(D,sig);
% sigdd = lsim(D,sigd);
% sigddd = lsim(D,sigdd);
% 
% figure(1)
% stairs(simTime,sig); hold on; grid on;
% pause();
% %figure
% plot(simTime,sinSigd,'b--','LineWidth',2);hold on; grid on;
% stairs(simTime,sigd,'b');
% pause();
% %figure
% plot(simTime,sinSigdd,'r--','LineWidth',2);hold on; grid on;
% stairs(simTime,sigdd,'r');
% pause();
% %figure
% plot(simTime,sinSigddd,'g--','LineWidth',2);hold on; grid on;
% stairs(simTime,sigddd,'g');
% 
% 


%%
load('adCtrOut.mat');
z = adCtrRef(:,3);
zd = adCtrRefd(:,3);
zd2 = adCtrRefdd(:,3);
startFound=false;
% retrieve the values computed by the admittance controller
for i=1:length(zd2)
    if ~startFound && z(i)~=0
        start=i;
        startFound=true;
    end
    if startFound && z(i)==0
        finish=i-1;
        z=z(start:finish);
        zd=zd(start:finish);
        zd2=zd2(start:finish);
        break;
    end
end

simTime = (0:length(z)-1)*Ts;

%zd = lsim(G,zAdmittance,[],7.35);
zd2b= lsim(D,zd,[],zd(1));
zd3   = lsim(D,zd2,[],zd2(1));
zd4  = lsim(D,zd3,[],zd3(1));




figure()
stairs(simTime,z);hold on; grid on;
stairs(simTime,zd);
stairs(simTime,zd2); 
stairs(simTime,zd2b);
legend('z','zd','zd2','zd2b','Location','best')
% pause()
% stairs(simTime,zd3);
% stairs(simTime,zd4);
% legend('z','zd','zd2','zd3 num','zd4 num','Location','best')

title('Discrete filtered derivative crazyflie')

%%
yAdSim = adCtrTraj(:,2);
startFound=false;
% retrieve the values computed by the admittance controller
for i=1:length(yAdSim)
    if ~startFound && yAdSim(i)~=0
        start=i;
        startFound=true;
    end
    if startFound && yAdSim(i)==0
        finish=i-1;
        yAdSim=yAdSim(start:finish);
        break;
    end
end

zdot=zeros(length(yAdSim)-1,3);

simTime = (0:length(yAdSim)-1)*Ts;

%zd = lsim(G,yAdmittance,[],7.35);
yd   = lsim(D,yAdSim,[],yAdSim(1));
ydd  = lsim(D,yd,[],yd(1));
yddd = lsim(D,ydd,[],ydd(1));




figure()
stairs(simTime,yAdSim); hold on; grid on;

stairs(simTime,yd);

stairs(simTime,ydd);

stairs(simTime,yddd)

legend('y mass damper','yd','ydd','yddd','Location','best')

title('Discrete filtered derivative crazyflie')


%% build a realistic signal
t =   [0 1 1.4 2.0 2.5 2.9 3.3 3.7];
val = 2*[0 0 0.25 0.21 -0.2 0.15 0 0];
tq = t(1):Ts:t(end);

figure()
f1 = interp1(t,val,tq);
plot(tq,f1); grid on;
legend('f real')
ylabel('N'); xlabel('s');

dev=1.5;
f1Noise=f1.*(1+dev*rand(1,length(f1)));
figure()
stairs(tq,f1Noise); grid on; legend('f filter')
ylabel('N'); xlabel('s');

virtualMasses = [0.5 0.5 0.5];
virtualDampings = [0.5 0.5 0.5];
initialStates=[0 0 0; 0 0 0];
adCtr = AdmittanceContrTEST(initialStates,virtualMasses,virtualDampings,Ts);
points=initialStates(1,:)';

for i=2:length(tq)
    points(:,end+1)=adCtr.update(ones(3,1)*f1Noise(i));
end

zA=points(3,:);

zAd   = lsim(D,zA,  [],points(1));
zAdd  = lsim(D,zAd, [],zAd(1));
zAddd = lsim(D,zAdd,[],zAdd(1));
figure()
stairs(tq,zA); hold on; grid on;
stairs(tq,zAd);
stairs(tq,zAdd);
stairs(tq,zAddd);
legend('z mass damper','zd','zdd','zddd','Location','best')
title('Discrete filtered derivative')
xlabel('s');


