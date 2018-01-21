%function r= Draw_7DOF_FB7roll_point_dual(ArmJoint_R,RotationM_R,PathPoint_R,ArmJoint_L,RotationM_L,PathPoint_L)
%DRAW_7DOF_POINT Summary of this function goes here
%   Detailed explanation goes here\

figure(1)
cla reset


%%���դ��u�V�� ��������
%  AZ=0;
%  EL=0;
% 
% xlim([-100 100]) % ���� X �b�d�� 
% ylim([-100 50]) % ���� Y �b�d�� 
% zlim([-250 20]) % ���� Z �b�d�� 


%Path���ե�  L0=0; L1=100; L2=100; L3=10; 

%  AZ=-50;
%  EL=40;
%  
% xlim([-100 100]) % ���� X �b�d�� 
% ylim([-100 50]) % ���� Y �b�d�� 
% zlim([-60 20]) % ���� Z �b�d�� 


%Path���ե� L0=255; L1=250; L2=250; L3=150;
% AZ=-50;
% EL=40;

AZ=-90;
EL=90;
 
xlim([-110 380]) % ���� X �b�d�� 
ylim([-600 0]) % ���� Y �b�d�� 
zlim([-400 200]) % ���� Z �b�d�� 

view(AZ,EL);


xlabel('x');
ylabel('y');

hold on;    grid on;    box on; rotate3d on ;

%% ========�e�s��======== %%
%Right Arm
for i=1:1:10
    if i==1
        plot3([0,ArmJoint_R(i,1)],[0,ArmJoint_R(i,2)],[0,ArmJoint_R(i,3)],'-r','LineWidth',2); %��y�e��Joint1
    else
        plot3([ArmJoint_R(i-1,1),ArmJoint_R(i,1)],[ArmJoint_R(i-1,2),ArmJoint_R(i,2)],[ArmJoint_R(i-1,3),ArmJoint_R(i,3)],'-r','LineWidth',2);
    end
end

% %Left Arm
% for i=1:1:10
%     if i==1
%         plot3([0,ArmJoint_L(i,1)],[0,ArmJoint_L(i,2)],[0,ArmJoint_L(i,3)],'-r','LineWidth',2); %��y�e��Joint1
%     else
%         plot3([ArmJoint_L(i-1,1),ArmJoint_L(i,1)],[ArmJoint_L(i-1,2),ArmJoint_L(i,2)],[ArmJoint_L(i-1,3),ArmJoint_L(i,3)],'-r','LineWidth',2);
%     end
% end

%% ========�e���I======== %%
%�e���I
plot3(0,0,0,'ro','MarkerSize',10,'Linewidth',4);text(0,0,0,'Org')

%% ========�e�C�b���`�I======== %%
%Right Arm
for i=1:1:10
  plot3(ArmJoint_R(i,1),ArmJoint_R(i,2),ArmJoint_R(i,3),'bo','MarkerSize',5,'Linewidth',4);
  
  %�Х�
  if i==2
       text(ArmJoint_R(i,1),ArmJoint_R(i,2),ArmJoint_R(i,3),'shoulder');
  elseif i==5
      text(ArmJoint_R(i,1),ArmJoint_R(i,2),ArmJoint_R(i,3),'Elbow');
  elseif i==7
      text(ArmJoint_R(i,1),ArmJoint_R(i,2),ArmJoint_R(i,3),'Wst');   
  end
end

%Left Arm
% for i=1:1:10
%   plot3(ArmJoint_L(i,1),ArmJoint_L(i,2),ArmJoint_L(i,3),'bo','MarkerSize',5,'Linewidth',4);
%   
%   %�Х�
%   if i==2
%        text(ArmJoint_L(i,1),ArmJoint_L(i,2),ArmJoint_L(i,3),'shoulder');
%    elseif i==5
%       text(ArmJoint_L(i,1),ArmJoint_L(i,2),ArmJoint_L(i,3),'Elbow');
%   elseif i==7
%       text(ArmJoint_L(i,1),ArmJoint_L(i,2),ArmJoint_L(i,3),'Wst');   
%   end
% end


%%  ========�e�g�LIK FK�B�����|�W���I======== %%
%Right Arm
plot3(PathIFKPointRec_R(:,1),PathIFKPointRec_R(:,2),PathIFKPointRec_R(:,3),'mo','MarkerSize',2,'Linewidth',1);
plot3(PathPlanPointRec_R(:,1),PathPlanPointRec_R(:,2),PathPlanPointRec_R(:,3),'b--','MarkerSize',2,'Linewidth',2);



% %Left Arm
% plot3(PathIFKPointRec_L(:,1),PathIFKPointRec_L(:,2),PathIFKPointRec_L(:,3),'mo','MarkerSize',2,'Linewidth',1);

%% ========End effector======== %%
%Right Arm
plot3(ArmJoint_R(10,1),ArmJoint_R(10,2),ArmJoint_R(10,3),'go','MarkerSize',10,'Linewidth',4);text(ArmJoint_R(10,1),ArmJoint_R(10,2),ArmJoint_R(10,3),'R_End');
% %Left Arm
% plot3(ArmJoint_L(10,1),ArmJoint_L(10,2),ArmJoint_L(10,3),'go','MarkerSize',10,'Linewidth',4);text(ArmJoint_L(10,1),ArmJoint_L(10,2),ArmJoint_L(10,3),'L_End');

%% ========���I���A�y�жb�Х�  orientation V_H_hat_x V_H_hat_y V_H_hat_z ========%%
%Right Arm
V_H_HAT_UNIT_LEN=100;
RotationM_R=RotationM_R*V_H_HAT_UNIT_LEN;
V_H_hat_x=RotationM_R(1:3,1);
V_H_hat_y=RotationM_R(1:3,2);
V_H_hat_z=RotationM_R(1:3,3);
plot3([ArmJoint_R(10,1),ArmJoint_R(10,1)+V_H_hat_x(1,1)],[ArmJoint_R(10,2),ArmJoint_R(10,2)+V_H_hat_x(2,1)],[ArmJoint_R(10,3),ArmJoint_R(10,3)+V_H_hat_x(3,1)],'-m','LineWidth',2); text(ArmJoint_R(10,1)+V_H_hat_x(1,1),ArmJoint_R(10,2)+V_H_hat_x(2,1),ArmJoint_R(10,3)+V_H_hat_x(3,1),'X')
plot3([ArmJoint_R(10,1),ArmJoint_R(10,1)+V_H_hat_y(1,1)],[ArmJoint_R(10,2),ArmJoint_R(10,2)+V_H_hat_y(2,1)],[ArmJoint_R(10,3),ArmJoint_R(10,3)+V_H_hat_y(3,1)],'-g','LineWidth',2); text(ArmJoint_R(10,1)+V_H_hat_y(1,1),ArmJoint_R(10,2)+V_H_hat_y(2,1),ArmJoint_R(10,3)+V_H_hat_y(3,1),'Y')
plot3([ArmJoint_R(10,1),ArmJoint_R(10,1)+V_H_hat_z(1,1)],[ArmJoint_R(10,2),ArmJoint_R(10,2)+V_H_hat_z(2,1)],[ArmJoint_R(10,3),ArmJoint_R(10,3)+V_H_hat_z(3,1)],'-b','LineWidth',2); text(ArmJoint_R(10,1)+V_H_hat_z(1,1),ArmJoint_R(10,2)+V_H_hat_z(2,1),ArmJoint_R(10,3)+V_H_hat_z(3,1),'Z')

% %Left Arm
% V_H_HAT_UNIT_LEN=100;
% RotationM_L=RotationM_L*V_H_HAT_UNIT_LEN;
% V_H_hat_x=RotationM_L(1:3,1);
% V_H_hat_y=RotationM_L(1:3,2);
% V_H_hat_z=RotationM_L(1:3,3);
% plot3([ArmJoint_L(10,1),ArmJoint_L(10,1)+V_H_hat_x(1,1)],[ArmJoint_L(10,2),ArmJoint_L(10,2)+V_H_hat_x(2,1)],[ArmJoint_L(10,3),ArmJoint_L(10,3)+V_H_hat_x(3,1)],'-m','LineWidth',2); text(ArmJoint_L(10,1)+V_H_hat_x(1,1),ArmJoint_L(10,2)+V_H_hat_x(2,1),ArmJoint_L(10,3)+V_H_hat_x(3,1),'X')
% plot3([ArmJoint_L(10,1),ArmJoint_L(10,1)+V_H_hat_y(1,1)],[ArmJoint_L(10,2),ArmJoint_L(10,2)+V_H_hat_y(2,1)],[ArmJoint_L(10,3),ArmJoint_L(10,3)+V_H_hat_y(3,1)],'-g','LineWidth',2); text(ArmJoint_L(10,1)+V_H_hat_y(1,1),ArmJoint_L(10,2)+V_H_hat_y(2,1),ArmJoint_L(10,3)+V_H_hat_y(3,1),'Y')
% plot3([ArmJoint_L(10,1),ArmJoint_L(10,1)+V_H_hat_z(1,1)],[ArmJoint_L(10,2),ArmJoint_L(10,2)+V_H_hat_z(2,1)],[ArmJoint_L(10,3),ArmJoint_L(10,3)+V_H_hat_z(3,1)],'-b','LineWidth',2); text(ArmJoint_L(10,1)+V_H_hat_z(1,1),ArmJoint_L(10,2)+V_H_hat_z(2,1),ArmJoint_L(10,3)+V_H_hat_z(3,1),'Z')

r=0;

%% ========�_���ϥ�  ========%%
% plot3(Needle_RobotF(1),Needle_RobotF(2),Needle_RobotF(3),'bo','MarkerSize',5,'Linewidth',4);



% ObjCorner=[ObjCorner;ObjCorner(1,1:3)];
% plot3(ObjCorner(:,1),ObjCorner(:,2),ObjCorner(:,3),':c','LineWidth',2);

% axis('equal');%ekXYZ�C�@�檺���Z�۵�
% xlim([-150 250]) % ���� X �b�d�� 
% ylim([-250 100]) % ���� y �b�d�� 
% zlim([-300 30]) % ���� z �b�d�� 




