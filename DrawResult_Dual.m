close all

DEF_X=1;
DEF_Y=2;
DEF_Z=3;
DEF_alpha=4;
DEF_beta=5;
DEF_gama=6;
DEF_rednt_alpha=7;
%% ========Cartesian space reference and feedback ========%%
%right hand
figure;
for i=DEF_X:1:DEF_Z
    subplot(3,1,i),plot(PathPlanPointRec_R.time,PathPlanPointRec_R.data(:,i),'-r','LineWidth',2); 
    hold on;
    subplot(3,1,i),plot(PathIFKPointRec_R.time,PathIFKPointRec_R.data(:,i),'-b','LineWidth',2); 
    xlabel('t');
   
    grid on;
    legend('reference','simulated');
    
    if i==DEF_X
        title('t versus x of right arm'); 
        ylabel('x (mm)');
    elseif i==DEF_Y
        title('t versus y of right arm'); 
        ylabel('y (mm)');
    elseif i==DEF_Z
        %set(gca,'ytick',[0:200:1000])
        %set(gca,'ytick',[25:5:35]);
        title('t versus z of right arm') ; 
        ylabel('z (mm)');
    end
end

figure;
for i=DEF_alpha:1:DEF_rednt_alpha
    subplot(4,1,i-3),plot(PathPlanPointRec_R.time,PathPlanPointRec_R.data(:,i),'-r','LineWidth',2); 
    hold on;
%     subplot(3,1,i),plot(PathIFKPointRec_R.time,PathIFKPointRec_R.data(:,i),'-b','LineWidth',2); 
    xlabel('t');
   
    grid on;
    legend('reference');
    
    if i==DEF_alpha
        title('t versus alpha of right arm'); 
        ylabel('theta');
    elseif i==DEF_beta
        title('t versus beta of right arm'); 
        ylabel('theta');
    elseif i==DEF_gama
        title('t versus gamma of right arm') ; 
        ylabel('theta');
     elseif i==DEF_rednt_alpha
        title('t versus rednt alpha of right arm') ; 
        ylabel('theta');    
    end
end


%left hand
figure;
for i=DEF_X:1:DEF_Z
    subplot(3,1,i),plot(PathPlanPointRec_L.time,PathPlanPointRec_L.data(:,i),'-r','LineWidth',2); 
    hold on;
    subplot(3,1,i),plot(PathIFKPointRec_L.time,PathIFKPointRec_L.data(:,i),'-b','LineWidth',2); 
    xlabel('t');
   
    grid on;
    legend('reference','simulated');
    
    if i==DEF_X
        title('t versus x of left arm'); 
        ylabel('x (mm)');
    elseif i==DEF_Y
        title('t versus y of left arm'); 
        ylabel('y (mm)');
    elseif i==DEF_Z
        %set(gca,'ytick',[25:5:40]);
        title('t versus z of left arm') ; 
        ylabel('z (mm)');
    end
end

figure;
for i=DEF_alpha:1:DEF_rednt_alpha
    subplot(4,1,i-3),plot(PathPlanPointRec_L.time,PathPlanPointRec_L.data(:,i),'-r','LineWidth',2); 
    hold on;
%     subplot(3,1,i),plot(PathIFKPointRec_L.time,PathIFKPointRec_L.data(:,i),'-b','LineWidth',2); 
    xlabel('t');
   
    grid on;
    legend('reference');
    
    if i==DEF_alpha
        title('t versus alpha of left arm'); 
        ylabel('theta');
    elseif i==DEF_beta
        title('t versus beta of left arm'); 
        ylabel('theta');
    elseif i==DEF_gama
        title('t versus gamma of left arm') ; 
        ylabel('theta');
     elseif i==DEF_rednt_alpha
        title('t versus rednt alpha of left arm') ; 
        ylabel('theta');    
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
title('angle for each axis of right arm') ; 

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
title('angle for each axis of left arm') ; 

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
title('error in Cartesian space of right arm') ; 

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
title('error in Cartesian space of left arm') ; 