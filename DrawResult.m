close all

DEF_X=1;
DEF_Y=2;
DEF_Z=3;
%% ========Cartesian space reference and feedback ========%%
figure(1)
%subplot(3,1,1),plot(ref_in_x.time,ref_in_x.data,'--r','LineWidth',2); %reference
subplot(3,1,1),plot(PathPlanPointRec_R.time,PathPlanPointRec_R.data(:,DEF_X),'--r','LineWidth',2); %reference
hold on;
subplot(3,1,1),plot(PathIFKPointRec_R.time,PathIFKPointRec_R.data(:,DEF_X),'-b','LineWidth',2); %feedback
xlabel('t');
ylabel('x (mm)');
grid on;
title('t versus x') ; 
legend('reference','actual');

subplot(3,1,2),plot(PathPlanPointRec_R.time,PathPlanPointRec_R.data(:,DEF_Y),'--r','LineWidth',2); 
hold on;
subplot(3,1,2),plot(PathIFKPointRec_R.time,PathIFKPointRec_R.data(:,DEF_Y),'-b','LineWidth',2); 
xlabel('t');
ylabel('y (mm)');
grid on;
title('t versus y') ; 
legend('reference','actual');

subplot(3,1,3),plot(PathPlanPointRec_R.time,PathPlanPointRec_R.data(:,DEF_Z),'--r','LineWidth',2); 
hold on;
subplot(3,1,3),plot(PathIFKPointRec_R.time,PathIFKPointRec_R.data(:,DEF_Z),'-b','LineWidth',2); 
xlabel('t');
ylabel('z (mm)');
grid on;
title('t versus z') ; 
legend('reference','actual');

%% ========motor output  ========%%
figure(2)
for i=1:7
    plot(motor_out_R.time,motor_out_R.Data(:,i),'LineWidth',2);
    hold on;
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
xlabel('t');
ylabel('angle');
grid on;
title('angle for each axis') ; 

%% ========error ========%%
figure(3)
for i=1:3
    plot(err_R.time,err_R.Data(:,i),'LineWidth',2);
    hold on;
end
legend('x','y','z');
xlabel('t');
ylabel('mm');
grid on;
title('error in Cartesian space') ; 
