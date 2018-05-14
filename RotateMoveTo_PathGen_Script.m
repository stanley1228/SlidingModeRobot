
%input parameter
%arc_cen=Needle_RobotF %������
%R_starP %�k�����_�I
%R_endP %�k�������I
%L_starP %�������_�I
%L_endP %���������I
%rot_rad %����ɪ��_�l���ਤ��
%CostTime ��O�ɶ�


if (Coordinate == DEF_OBJFRAME_COOR)
    arc_cen=arc_cen+TranFrameToRobot;
    L_starP(1,1:3)=L_starP(1,1:3)+TranFrameToRobot;
    L_endP(1,1:3)=L_endP(1,1:3)+TranFrameToRobot;
    R_starP(1,1:3)=R_starP(1,1:3)+TranFrameToRobot;
    R_endP(1,1:3)=R_endP(1,1:3)+TranFrameToRobot;
end

%�k���P���|
rR=sqrt((R_starP(1)-arc_cen(1))^2+(R_starP(2)-arc_cen(2))^2);%�k�����b�|
ini_rad_R=pi+atan((R_starP(2)-arc_cen(2))/(R_starP(1)-arc_cen(1)));%����ɪ��_�l���ਤ��

%�����P���|
rL=sqrt((L_starP(1)-arc_cen(1))^2+(L_starP(2)-arc_cen(2))^2);
ini_rad_L=atan((L_starP(2)-arc_cen(2))/(L_starP(1)-arc_cen(1)));

%parabolic parameter
acc_deg_L=50; %cartesian space���઺���ת����t��
acc_deg_R=50;
DEF_ACC_L=[acc_deg_L*(pi/180) acc_deg_L*(pi/180) acc_deg_L*(pi/180) 100 100 100 100]; %item x,y,z use the same compenet to interpolate unit is rad/s^2   the rest of item's unit is len/s^2
DEF_ACC_R=[acc_deg_R*(pi/180) acc_deg_R*(pi/180) acc_deg_R*(pi/180) 100 100 100 100]; %

for i=1:1:7
    if(i<=3)%�e�T�� xyz�@�ΦP�@�Ӯt�Ȥ���
        acc_L_min(i)=4*rot_rad/(CostTime^2);
    else
        acc_L_min(i)=4*(L_endP(i)-L_starP(i))/(CostTime^2);
    end
    
    if DEF_ACC_L(i) < abs(acc_L_min(i))
        error('L cost time too short');
    end 

    if(i<=3)
        tb_L(i)=(DEF_ACC_L(i)*CostTime-sqrt(DEF_ACC_L(i)^2*CostTime^2-4*DEF_ACC_L(i)*rot_rad))/(2*DEF_ACC_L(i));
    else
        tb_L(i)=(DEF_ACC_L(i)*CostTime-sqrt(DEF_ACC_L(i)^2*CostTime^2-4*DEF_ACC_L(i)*(L_endP(i)-L_starP(i))))/(2*DEF_ACC_L(i));
    end
    
    
    %%%%%%%%%%%%
    if(i<=3)
        acc_R_min(i)=4*rot_rad/(CostTime^2);
    else
        acc_R_min(i)=4*(R_endP(i)-R_starP(i))/(CostTime^2);
    end 
    
    if DEF_ACC_R(i) < abs(acc_R_min(i))
        error('R cost time too short');
    end 
    
    if(i<=3)
        tb_R(i)=(DEF_ACC_R(i)*CostTime-sqrt(DEF_ACC_R(i)^2*CostTime^2-4*DEF_ACC_R(i)*rot_rad))/(2*DEF_ACC_R(i));    
    else
        tb_R(i)=(DEF_ACC_R(i)*CostTime-sqrt(DEF_ACC_R(i)^2*CostTime^2-4*DEF_ACC_R(i)*(R_endP(i)-R_starP(i))))/(2*DEF_ACC_R(i));    
    end
end

for t=DEF_CYCLE_TIME:DEF_CYCLE_TIME:CostTime
    for i=1:1:7
        %%right
        if(i<=3)
            if(t<tb_R(i))
                current_rad_R=ini_rad_R+0.5*DEF_ACC_R(i)*t^2;
            elseif (t<CostTime-tb_R(i))
                current_rad_R=ini_rad_R+0.5*DEF_ACC_R(i)*tb_R(i)^2+DEF_ACC_R(i)*tb_R(i)*(t-tb_R(i)); 
            else
                current_rad_R=(ini_rad_R+rot_rad)-0.5*DEF_ACC_R(i)*(CostTime-t)^2;
            end
            
        else
            if (R_starP(i) == R_endP(i))
                PathPlanPoint_R(i)=R_endP(i);
            else
                if(t<tb_R(i))
                    PathPlanPoint_R(i)=R_starP(i)+0.5*DEF_ACC_R(i)*t^2;
                elseif (t<CostTime-tb_R(i))
                    PathPlanPoint_R(i)=R_starP(i)+0.5*DEF_ACC_R(i)*tb_R(i)^2+DEF_ACC_R(i)*tb_R(i)*(t-tb_R(i));   
                else
                    PathPlanPoint_R(i)=R_endP(i)-0.5*DEF_ACC_R(i)*(CostTime-t)^2;
                end
            end
        end    
        
        %%Left
        if(i<=3)
            if(t<tb_L(i))
                current_rad_L=ini_rad_L+0.5*DEF_ACC_L(i)*t^2;
            elseif (t<CostTime-tb_L(i))
                current_rad_L=ini_rad_L+0.5*DEF_ACC_L(i)*tb_L(i)^2+DEF_ACC_L(i)*tb_L(i)*(t-tb_L(i)); 
            else
                current_rad_L=(ini_rad_L+rot_rad)-0.5*DEF_ACC_L(i)*(CostTime-t)^2;
            end
%             i=3;% 1~3 calculate the same thing current_rad_R
        else
            if (L_starP(i) == L_endP(i))
                PathPlanPoint_L(i)=L_endP(i);
            else
                if(t<tb_L(i))
                    PathPlanPoint_L(i)=L_starP(i)+0.5*DEF_ACC_L(i)*t^2;
                elseif (t<CostTime-tb_L(i))
                    PathPlanPoint_L(i)=L_starP(i)+0.5*DEF_ACC_L(i)*tb_L(i)^2+DEF_ACC_L(i)*tb_L(i)*(t-tb_L(i));   
                else
                    PathPlanPoint_L(i)=L_endP(i)-0.5*DEF_ACC_L(i)*(CostTime-t)^2;
                end
            end
        end
    end
    
    PathPlanPoint_R=[arc_cen 0 0 0 0] +rR*[cos(current_rad_R) sin(current_rad_R) 0 0 0 0 0]+[0 0 0 PathPlanPoint_R(4:7)]; %�k��W��U����
    PathPlanPoint_L=[arc_cen 0 0 0 0] +rL*[cos(current_rad_L) sin(current_rad_L) 0 0 0 0 0]+[0 0 0 PathPlanPoint_L(4:7)]; %�k��W��U����
   
    %PathPlanPoint_R=[arc_cen 0 0 0 0] +rR*[cos(rot_rad*t/CostTime+ini_rad_R) sin(rot_rad*t/CostTime+ini_rad_R) 0 0 0 0 0]+[0 0 0 R_starP(4:7)+(R_endP(4:7)-R_starP(4:7))*t/CostTime]; %�k��W��U����
    %PathPlanPoint_L=[arc_cen 0 0 0 0] +rL*[cos(rot_rad*t/CostTime+ini_rad_L) sin(rot_rad*t/CostTime+ini_rad_L) 0 0 0 0 0]+[0 0 0 L_starP(4:7)+(L_endP(4:7)-L_starP(4:7))*t/CostTime]; %����W��U����

    if (FRAME_UPDATE==true)
        ObjCenter=(PathPlanPoint_R(1:3)+PathPlanPoint_L(1:3))/2;%�p���_�����|�P����I�y��
        V_oc_lend=PathPlanPoint_L(1:3)-ObjCenter;%�_���������I�쥪�⪺�V�q
        V_oc_lend_ro_p90=[V_oc_lend 1]*Rz(0.5*pi);
        V_oc_lend_ro_n90=[V_oc_lend 1]*Rz(-0.5*pi);
        ObjCorner=[ PathPlanPoint_L(1:3); 
                    ObjCenter+V_oc_lend_ro_p90(1:3);
                    PathPlanPoint_R(1:3);
                    ObjCenter+V_oc_lend_ro_n90(1:3)];         
    end     
    
    ObjCorner_raw=reshape(ObjCorner,1,[]);%for drawing the object frame
    ObjCornerRec_raw=addsample(ObjCornerRec_raw,'Time',abst,'Data',ObjCorner_raw);
    
    PathPlanPointRec_R=addsample(PathPlanPointRec_R,'Time',abst,'Data',PathPlanPoint_R);
    PathPlanPointRec_L=addsample(PathPlanPointRec_L,'Time',abst,'Data',PathPlanPoint_L);
    abst=abst+DEF_CYCLE_TIME;
    %VerifyOutput_script;
    
end


