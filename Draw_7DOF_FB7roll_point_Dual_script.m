%function r= Draw_7DOF_FB7roll_point_dual(ArmJoint_R,RotationM_R,PathPoint_R,ArmJoint_L,RotationM_L,PathPoint_L)
%DRAW_7DOF_POINT Summary of this function goes here
%   Detailed explanation goes here\



%%測試手臂向後 切換視角
%  AZ=0;
%  EL=0;
% 
% xlim([-100 100]) % 改變 X 軸範圍 
% ylim([-100 50]) % 改變 Y 軸範圍 
% zlim([-250 20]) % 改變 Z 軸範圍 


%Path測試用  L0=0; L1=100; L2=100; L3=10; 

%  AZ=-50;
%  EL=40;
%  
% xlim([-100 100]) % 改變 X 軸範圍 
% ylim([-100 50]) % 改變 Y 軸範圍 
% zlim([-60 20]) % 改變 Z 軸範圍 


%Path測試用 L0=255; L1=250; L2=250; L3=150;
% AZ=-50;
% EL=40;

% view(AZ,EL);


figure(9);


xlabel('x');
ylabel('y');

grid on;    box on; rotate3d on ;

max_cnt=TotalTime/DEF_CYCLE_TIME;

for cnt=1:10:max_cnt
    figure(9)
   
    hold off; 
       %% ========畫原點======== %%
    %畫原點
    plot3(0,0,0,'ro','MarkerSize',10,'Linewidth',4);text(0,0,0,'Org')
    hold on; 
    
    %% ========畫連桿======== %%
    ArmJoint_R=reshape(getdatasamples(ArmJoint_raw_R,cnt),[10,3]);
    %Right Arm
    for i=1:1:10
        if i==1
            plot3([0,ArmJoint_R(i,1)],[0,ArmJoint_R(i,2)],[0,ArmJoint_R(i,3)],'-r','LineWidth',2); %基座畫到Joint1
        else
            plot3([ArmJoint_R(i-1,1),ArmJoint_R(i,1)],[ArmJoint_R(i-1,2),ArmJoint_R(i,2)],[ArmJoint_R(i-1,3),ArmJoint_R(i,3)],'-r','LineWidth',2);
        end
    end
    
    ArmJoint_L=reshape(getdatasamples(ArmJoint_raw_L,cnt),[10,3]);
    %Left Arm
    for i=1:1:10
        if i==1
            plot3([0,ArmJoint_L(i,1)],[0,ArmJoint_L(i,2)],[0,ArmJoint_L(i,3)],'-r','LineWidth',2); %基座畫到Joint1
        else
            plot3([ArmJoint_L(i-1,1),ArmJoint_L(i,1)],[ArmJoint_L(i-1,2),ArmJoint_L(i,2)],[ArmJoint_L(i-1,3),ArmJoint_L(i,3)],'-r','LineWidth',2);
        end
    end

  
    %% ========畫每軸關節點======== %%
    %Right Arm
    for i=1:1:10
      plot3(ArmJoint_R(i,1),ArmJoint_R(i,2),ArmJoint_R(i,3),'bo','MarkerSize',5,'Linewidth',4);

      %標示
      if i==2
           text(ArmJoint_R(i,1),ArmJoint_R(i,2),ArmJoint_R(i,3),'shoulder');
      elseif i==5
          text(ArmJoint_R(i,1),ArmJoint_R(i,2),ArmJoint_R(i,3),'Elbow');
      elseif i==7
          text(ArmJoint_R(i,1),ArmJoint_R(i,2),ArmJoint_R(i,3),'Wst');   
      end
    end

    %Left Arm
    for i=1:1:10
      plot3(ArmJoint_L(i,1),ArmJoint_L(i,2),ArmJoint_L(i,3),'bo','MarkerSize',5,'Linewidth',4);
      
      %標示
      if i==2
           text(ArmJoint_L(i,1),ArmJoint_L(i,2),ArmJoint_L(i,3),'shoulder');
       elseif i==5
          text(ArmJoint_L(i,1),ArmJoint_L(i,2),ArmJoint_L(i,3),'Elbow');
      elseif i==7
          text(ArmJoint_L(i,1),ArmJoint_L(i,2),ArmJoint_L(i,3),'Wst');   
      end
    end


    %%  ========畫經過IK FK運算後路徑上的點======== %%
    %Right Arm
    plot3(PathIFKPointRec_R.data(1:10:cnt,1),PathIFKPointRec_R.data(1:10:cnt,2),PathIFKPointRec_R.data(1:10:cnt,3),'mo','MarkerSize',2,'Linewidth',1);%1:10:cnt make the interval more
    %plot3(PathPlanPointRec_R.data(1:cnt,1),PathPlanPointRec_R.data(1:cnt,2),PathPlanPointRec_R.data(1:cnt,3),'b--','MarkerSize',2,'Linewidth',2);



    %Left Arm
    plot3(PathIFKPointRec_L.data(1:10:cnt,1),PathIFKPointRec_L.data(1:10:cnt,2),PathIFKPointRec_L.data(1:10:cnt,3),'mo','MarkerSize',2,'Linewidth',1);%real
    %plot3(PathPlanPointRec_L.data(1:cnt,1),PathPlanPointRec_L.data(1:cnt,2),PathPlanPointRec_L.data(1:cnt,3),'b--','MarkerSize',2,'Linewidth',2);%reference


    %% ========End effector======== %%
    %Right Arm
    plot3(ArmJoint_R(10,1),ArmJoint_R(10,2),ArmJoint_R(10,3),'go','MarkerSize',10,'Linewidth',4);text(ArmJoint_R(10,1),ArmJoint_R(10,2),ArmJoint_R(10,3),'R_End');
    % %Left Arm
    plot3(ArmJoint_L(10,1),ArmJoint_L(10,2),ArmJoint_L(10,3),'go','MarkerSize',10,'Linewidth',4);text(ArmJoint_L(10,1),ArmJoint_L(10,2),ArmJoint_L(10,3),'L_End');

    %% ========末點姿態座標軸標示  orientation V_H_hat_x V_H_hat_y V_H_hat_z ========%%
    
    %Right Arm
    RotationM_R=reshape(getdatasamples(RotationM_raw_R,cnt),[3,3]);
    V_H_HAT_UNIT_LEN=100;
    RotationM_R=RotationM_R*V_H_HAT_UNIT_LEN;
    V_H_hat_x=RotationM_R(1:3,1);
    V_H_hat_y=RotationM_R(1:3,2);
    V_H_hat_z=RotationM_R(1:3,3);
    plot3([ArmJoint_R(10,1),ArmJoint_R(10,1)+V_H_hat_x(1,1)],[ArmJoint_R(10,2),ArmJoint_R(10,2)+V_H_hat_x(2,1)],[ArmJoint_R(10,3),ArmJoint_R(10,3)+V_H_hat_x(3,1)],'-m','LineWidth',2); text(ArmJoint_R(10,1)+V_H_hat_x(1,1),ArmJoint_R(10,2)+V_H_hat_x(2,1),ArmJoint_R(10,3)+V_H_hat_x(3,1),'X')
    plot3([ArmJoint_R(10,1),ArmJoint_R(10,1)+V_H_hat_y(1,1)],[ArmJoint_R(10,2),ArmJoint_R(10,2)+V_H_hat_y(2,1)],[ArmJoint_R(10,3),ArmJoint_R(10,3)+V_H_hat_y(3,1)],'-g','LineWidth',2); text(ArmJoint_R(10,1)+V_H_hat_y(1,1),ArmJoint_R(10,2)+V_H_hat_y(2,1),ArmJoint_R(10,3)+V_H_hat_y(3,1),'Y')
    plot3([ArmJoint_R(10,1),ArmJoint_R(10,1)+V_H_hat_z(1,1)],[ArmJoint_R(10,2),ArmJoint_R(10,2)+V_H_hat_z(2,1)],[ArmJoint_R(10,3),ArmJoint_R(10,3)+V_H_hat_z(3,1)],'-b','LineWidth',2); text(ArmJoint_R(10,1)+V_H_hat_z(1,1),ArmJoint_R(10,2)+V_H_hat_z(2,1),ArmJoint_R(10,3)+V_H_hat_z(3,1),'Z')

    %Left Arm
    RotationM_L=reshape(getdatasamples(RotationM_raw_L,cnt),[3,3]);
    V_H_HAT_UNIT_LEN=100;
    RotationM_L=RotationM_L*V_H_HAT_UNIT_LEN;
    V_H_hat_x=RotationM_L(1:3,1);
    V_H_hat_y=RotationM_L(1:3,2);
    V_H_hat_z=RotationM_L(1:3,3);
    plot3([ArmJoint_L(10,1),ArmJoint_L(10,1)+V_H_hat_x(1,1)],[ArmJoint_L(10,2),ArmJoint_L(10,2)+V_H_hat_x(2,1)],[ArmJoint_L(10,3),ArmJoint_L(10,3)+V_H_hat_x(3,1)],'-m','LineWidth',2); text(ArmJoint_L(10,1)+V_H_hat_x(1,1),ArmJoint_L(10,2)+V_H_hat_x(2,1),ArmJoint_L(10,3)+V_H_hat_x(3,1),'X')
    plot3([ArmJoint_L(10,1),ArmJoint_L(10,1)+V_H_hat_y(1,1)],[ArmJoint_L(10,2),ArmJoint_L(10,2)+V_H_hat_y(2,1)],[ArmJoint_L(10,3),ArmJoint_L(10,3)+V_H_hat_y(3,1)],'-g','LineWidth',2); text(ArmJoint_L(10,1)+V_H_hat_y(1,1),ArmJoint_L(10,2)+V_H_hat_y(2,1),ArmJoint_L(10,3)+V_H_hat_y(3,1),'Y')
    plot3([ArmJoint_L(10,1),ArmJoint_L(10,1)+V_H_hat_z(1,1)],[ArmJoint_L(10,2),ArmJoint_L(10,2)+V_H_hat_z(2,1)],[ArmJoint_L(10,3),ArmJoint_L(10,3)+V_H_hat_z(3,1)],'-b','LineWidth',2); text(ArmJoint_L(10,1)+V_H_hat_z(1,1),ArmJoint_L(10,2)+V_H_hat_z(2,1),ArmJoint_L(10,3)+V_H_hat_z(3,1),'Z')

    r=0;

    %% ========縫紉使用  ========%%
    plot3(Needle_RobotF(1),Needle_RobotF(2),Needle_RobotF(3),'bx','MarkerSize',2,'Linewidth',4);

    ObjCorner=reshape(getdatasamples(ObjCornerRec_raw,cnt),[4,3]);
    ObjCorner=[ObjCorner;ObjCorner(1,1:3)];%for last point link to first point
    plot3(ObjCorner(:,1),ObjCorner(:,2),ObjCorner(:,3),':c','LineWidth',2);

    
    xlabel('x');
    ylabel('y');

    grid on;    %rotate3d on ;
    
    xlim([-200 550]) % 改變 X 軸範圍 
    ylim([-600 500]) % 改變 Y 軸範圍 
    zlim([-400 200]) % 改變 Z 軸範圍 


    AZ=-45;
    EL=45;

    view(AZ,EL);
    
    pause(0.001);
end



