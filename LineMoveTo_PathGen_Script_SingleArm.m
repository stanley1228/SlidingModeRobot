if (Coordinate == DEF_OBJFRAME_COOR)
    if(arm_sel == DEF_LEFT_HAND)
        L_starP(1,1:3)=L_starP(1,1:3)+TranFrameToRobot;
        L_endP(1,1:3)=L_endP(1,1:3)+TranFrameToRobot;
    elseif(arm_sel == DEF_RIGHT_HAND)
        R_starP(1,1:3)=R_starP(1,1:3)+TranFrameToRobot;
        R_endP(1,1:3)=R_endP(1,1:3)+TranFrameToRobot;
    end
end

DEF_ACC_L=[50 50 50 50 50 50 50]; %len/s^2
DEF_ACC_R=[50 50 50 50 50 50 50];

for i=1:1:7
    
    if(arm_sel == DEF_LEFT_HAND)
        acc_L_min(i)=4*(L_endP(i)-L_starP(i))/(CostTime^2);

        if DEF_ACC_L(i) < abs(acc_L_min(i))
            error('L cost time too short');
        end 

        tb_L(i)=(DEF_ACC_L(i)*CostTime-sqrt(DEF_ACC_L(i)^2*CostTime^2-4*DEF_ACC_L(i)*abs(L_endP(i)-L_starP(i))))/(2*DEF_ACC_L(i));

        if (L_endP(i)-L_starP(i)) < 0
            DEF_ACC_L(i)=-DEF_ACC_L(i);
        end

        if tb_L(i) < 0
            error('tb_L < 0');
        end 
    elseif(arm_sel == DEF_RIGHT_HAND)
        acc_R_min(i)=4*(R_endP(i)-R_starP(i))/(CostTime^2);

        if DEF_ACC_R(i) < abs(acc_R_min(i))
            error('R cost time too short');
        end 

        tb_R(i)=(DEF_ACC_R(i)*CostTime-sqrt(DEF_ACC_R(i)^2*CostTime^2-4*DEF_ACC_R(i)*abs(R_endP(i)-R_starP(i))))/(2*DEF_ACC_R(i));

        if (R_endP(i)-R_starP(i)) < 0
             DEF_ACC_R(i)=-DEF_ACC_R(i);
        end

        if tb_R(i) < 0
            error('tb_R < 0');
        end 
    end
end

 for t=DEF_CYCLE_TIME:DEF_CYCLE_TIME:CostTime
    
    for i=1:1:7
        if(arm_sel == DEF_LEFT_HAND)
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
        elseif(arm_sel == DEF_RIGHT_HAND)
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
    end
    
%     PathPlanPoint_L=L_starP+(L_endP-L_starP)*t/CostTime;
%     PathPlanPoint_R=R_starP+(R_endP-R_starP)*t/CostTime;
    
    %縫紉物四周抓取點座標     
    if(FRAME_UPDATE==true)
        ObjCorner=[ PathPlanPoint_L(1:3)+HoldLen_L;
                    PathPlanPoint_R(1:3)+HoldLen_R;
                    PathPlanPoint_R(1:3);
                    PathPlanPoint_L(1:3)];
    end            
    
    ObjCorner_raw=reshape(ObjCorner,1,[]);
    ObjCornerRec_raw=addsample(ObjCornerRec_raw,'Time',abst,'Data',ObjCorner_raw);
    
    if(arm_sel == DEF_RIGHT_HAND)
        PathPlanPointRec_R=addsample(PathPlanPointRec_R,'Time',abst,'Data',PathPlanPoint_R);
    elseif(arm_sel == DEF_LEFT_HAND)
        PathPlanPointRec_L=addsample(PathPlanPointRec_L,'Time',abst,'Data',PathPlanPoint_L);
    end
    abst=abst+DEF_CYCLE_TIME;
    %VerifyOutput_script;
    
 end


