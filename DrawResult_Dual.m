close all

DEF_X=1;
DEF_Y=2;
DEF_Z=3;
%% ========Cartesian space reference and feedback ========%%
%right hand
figure;
for i=DEF_X:1:DEF_Z
    subplot(3,1,i),plot(PathPlanPointRec_R.time,PathPlanPointRec_R.data(:,i),'--r','LineWidth',2); 
    hold on;
    subplot(3,1,i),plot(PathIFKPointRec_R.time,PathIFKPointRec_R.data(:,i),'-b','LineWidth',2); 
    xlabel('t');
   
    grid on;
    legend('reference','actual');
    
    if i==DEF_X
        title('t versus x of right hand'); 
        ylabel('x (mm)');
    elseif i==DEF_Y
        title('t versus y of right hand'); 
        ylabel('y (mm)');
    elseif i==DEF_Z
        title('t versus z of right hand') ; 
        ylabel('z (mm)');
    end
end
%left hand
figure;
for i=DEF_X:1:DEF_Z
    subplot(3,1,i),plot(PathPlanPointRec_L.time,PathPlanPointRec_L.data(:,i),'--r','LineWidth',2); 
    hold on;
    subplot(3,1,i),plot(PathIFKPointRec_L.time,PathIFKPointRec_L.data(:,i),'-b','LineWidth',2); 
    xlabel('t');
   
    grid on;
    legend('reference','actual');
    
    if i==DEF_X
        title('t versus x of left hand'); 
        ylabel('x (mm)');
    elseif i==DEF_Y
        title('t versus y of left hand'); 
        ylabel('y (mm)');
    elseif i==DEF_Z
        title('t versus z of left hand') ; 
        ylabel('z (mm)');
    end
end

%% ========motor output  ========%%
%right hand
figure;
for i=1:7
    plot(motor_out_R.time,motor_out_R.Data(:,i),'LineWidth',2);
    hold on;
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
xlabel('t');
ylabel('angle');
grid on;
title('angle for each axis of right hand') ; 

%left hand
figure;
for i=1:7
    plot(motor_out_L.time,motor_out_L.Data(:,i),'LineWidth',2);
    hold on;
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
xlabel('t');
ylabel('angle');
grid on;
title('angle for each axis of left hand') ; 

%% ========error ========%%
%right hand
figure;
for i=1:3
    plot(err_R.time,err_R.Data(:,i),'LineWidth',2);
    hold on;
end
legend('x','y','z');
xlabel('t');
ylabel('mm');
grid on;
title('error in Cartesian space of right hand') ; 

%left hand
figure;
for i=1:3
    plot(err_L.time,err_L.Data(:,i),'LineWidth',2);
    hold on;
end
legend('x','y','z');
xlabel('t');
ylabel('mm');
grid on;
title('error in Cartesian space of left hand') ; 