%�ĤC�b��roll�b
%�ϥ�tic toc �p�ɡA�ثe�ݪ�0.25ms~0.3ms
function theta = IK_7DOF_FB7roll(RLHand,linkL,base,Pend,PoseAngle,Rednt_alpha)
    
    %
    DEF_NORM_VERY_SMALL =1.e-3;
    DEF_COSVAL_VERY_SMALL =1.e-7;
    %define 
    DEF_RIGHT_HAND=1;
    DEF_LEFT_HAND=2;

    %��X�Ѽ�initial
    theta=zeros(1,7);

    %��J�s�����
    L0=linkL(1);%L0 �Y��ӻH
    L1=linkL(2);%L1 �W�uL������
    L2=linkL(3);%L2 �W�uL���u��
    L3=linkL(4);%L3 �W�uL���u��
    L4=linkL(5);%L4 �W�uL������
    L5=linkL(6);%L5 end effector
    %% == �D�XH_hat_x ==%%
    %R=R_z1x2z3(alpha,beta,gamma);
    R=R_z1x2y3(PoseAngle(1),PoseAngle(2),PoseAngle(3)); %alpha,beta,gamma
    V_H_hat_x=R(1:3,1);%���X�کԨ��ഫ������x�}�A���X��1�欰X�b�����V�q
    V_H_hat_x=V_H_hat_x/norm(V_H_hat_x);
    V_H_hat_y=R(1:3,2);%���X�کԨ��ഫ������x�}�A���X��2�欰Y�b�����V�q
    V_H_hat_z=R(1:3,3);
 
    V_r_end=Pend-base;
    V_r_h=L5*V_H_hat_x;
    V_r_wst=V_r_end-V_r_h;


     %% ==Axis4== %%
    ru_norm=(L1^2+L2^2)^0.5; %L�����������
    rf_norm=(L3^2+L4^2)^0.5;

    theta_tmp=acos((ru_norm^2 + rf_norm^2- norm(V_r_wst)^2) / (2*ru_norm*rf_norm));
    theta(4)=2*pi-atan2(L1,L2)-atan2(L4,L3)-theta_tmp;

    %% ==Axis1 2== %%
    V_r_m=(ru_norm^2-rf_norm^2+norm(V_r_wst)^2)/(2*norm(V_r_wst)^2)*V_r_wst;

    %Redundant circle �b�|R
    Rednt_cir_R=ru_norm^2-((ru_norm^2-rf_norm^2+norm(V_r_wst)^2)/(2*norm(V_r_wst)))^2;
    Rednt_cir_R=Rednt_cir_R^0.5;

    %�ꤤ���I��Elbow�V�q V_r_u
    V_shx=[1;0;0];
    V_shy=[0;1;0];
    V_shz=[0;0;1];

    V_alpha_hat=cross(V_r_wst,V_shz)/norm(cross(V_r_wst,V_shz));
    V_beta_hat=cross(V_r_wst,V_alpha_hat)/norm(cross(V_r_wst,V_alpha_hat));

    temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst))*[Rednt_cir_R*V_beta_hat;1];  %Rednt_alpha����V�M�פ�W����V�ʬۤ�
    V_R_u=temp(1:3,1);
    V_r_u=V_r_m+V_R_u;

    %���� V_r_u  ��V_ru_l1
    V_r_f=V_r_wst-V_r_u;
    Vn_u_f=cross(V_r_u,V_r_f)/norm(cross(V_r_u,V_r_f)); %ru �� rf���k�V�q
    theat_upoff=atan(L2/L1);
    temp=Rogridues(-theat_upoff,Vn_u_f)*[V_r_u;1];  %���� V_r_u  ��V_ru_l1
    V_ru_l1=temp(1:3,1);
    V_ru_l1=V_ru_l1*L1/norm(V_ru_l1); %�վ㦨L1����

    theta(1)=atan2(V_ru_l1(1),-V_ru_l1(3));
    
    if theta(1) ~= 0
        theta(2)=atan2(V_ru_l1(2),V_ru_l1(1)/sin(theta(1)));
    else
        theta(2)=atan2(V_ru_l1(2),-V_ru_l1(3));   
    end   

    
    %% ==Axis3== %%
    %��shy(V_r_u,V_r_f���k�V�q)�g�L1,2�b�����  �PV_r_u,V_r_f �ݭn��3�b��h��
    V_n_yrot12=Ry(-theta(1))*Rx(theta(2))*[-V_shy;1];  %�Ĥ@�b�M�j�aY�y�Ф�V�ۤ�
    V_n_yrot12=V_n_yrot12(1:3,1);

    Vn_nuf_nyrot12=cross(Vn_u_f,V_n_yrot12);
    Vn_nuf_nyrot12=Vn_nuf_nyrot12/norm(Vn_nuf_nyrot12);

    temp=V_n_yrot12'*Vn_u_f/norm(V_n_yrot12)/norm(Vn_u_f); 

    %����bacos(1.000000.....)���ɭԷ|�X�{�곡�����p
    if abs(temp-1)<DEF_COSVAL_VERY_SMALL 
       if temp >0
           temp=1;
       else
           temp=-1;
       end
    end
    
    %Vn_u_f �M V_n_yrot12���k�V�q   �P V_ru_l1�P��V theta(3)�ݭn�[�t��
    if norm(Vn_nuf_nyrot12 - V_ru_l1/norm(V_ru_l1)) < DEF_NORM_VERY_SMALL
        theta(3)=-acos(temp);
    else
        theta(3)=acos(temp);
    end
    
 
    %% ==Axis5== %%
    %����V_r_f �� V_rf_l4
    theat_lowoff=atan(L3/L4);
    temp=Rogridues(theat_lowoff,Vn_u_f)*[V_r_f;1];  %���� V_r_f  V_rf_l4
    V_rf_l4=temp(1:3,1);
    V_rf_l4=V_rf_l4*L4/norm(V_rf_l4); %�վ㦨L4����

    %V_n_rfl4 ��V_n_rf�Φ������� ���k�V�q
    Vn_rfl4_nuf=cross(V_rf_l4,Vn_u_f)/norm(cross(V_rf_l4,Vn_u_f));
    t_rfl4_nuf=(Vn_rfl4_nuf'*V_r_wst-Vn_rfl4_nuf'*V_r_end)/(norm(Vn_rfl4_nuf)^2); %V_n_rf,V_n_rfl4�����W�A�B�g�LV_r_end�I�����u�ѼƦ���t ��rfl4_nuf
    Vproj_end_rfl4_nuf=V_r_end+t_rfl4_nuf*Vn_rfl4_nuf;%V_r_end �u��V_n_rfl4,V_n_rf�����k�V�q��v�b�����W���I
    V_wst_to_projend_rfl4_nuf=Vproj_end_rfl4_nuf-V_r_wst;

    %����bacos(1.000000.....)���ɭԷ|�X�{�곡�����p
    temp=V_rf_l4'*V_wst_to_projend_rfl4_nuf/norm(V_rf_l4)/norm(V_wst_to_projend_rfl4_nuf);
    if abs(temp-1)<DEF_COSVAL_VERY_SMALL 
       if temp >0
           temp=1;
       else
           temp=-1;
       end
    end

    %�����k�V�q �M Vn_rfl4_nuf  �P��n�[�t��  �P�_theta5�n���W�Ω��U��
    Vn_rfl4_WstToProjEndRfl4Nuf=cross(V_rf_l4/norm(V_rf_l4),V_wst_to_projend_rfl4_nuf/norm(V_wst_to_projend_rfl4_nuf));
    Vn_rfl4_WstToProjEndRfl4Nuf=Vn_rfl4_WstToProjEndRfl4Nuf/norm(Vn_rfl4_WstToProjEndRfl4Nuf);
    if norm(Vn_rfl4_WstToProjEndRfl4Nuf - Vn_rfl4_nuf) < DEF_NORM_VERY_SMALL
        theta(5)=-acos(temp); 
    else
        theta(5)=acos(temp); 
    end


    %% ==Axis6== %%
    temp=Rogridues(-theta(5),Vn_rfl4_nuf)*[Vn_u_f;1]; 
    Vn_nuf_rotx5_along_NRfl4Nuf=temp(1:3,1);%nuf �u�� Vn_rfl4_nuf �����5�b���ױo���v�I�P�ؼ��I�������k�V�q
    Vn_nuf_rotx5_along_NRfl4Nuf=Vn_nuf_rotx5_along_NRfl4Nuf/norm(Vn_nuf_rotx5_along_NRfl4Nuf);
    V_wst_to_end=V_r_end-V_r_wst;
    Vn_WstToEnd_WstToProjEndRfl4Nuf=cross(V_wst_to_end,V_wst_to_projend_rfl4_nuf);%V_wst_to_projend �M V_wst_to_end���k�V�q
    Vn_WstToEnd_WstToProjEndRfl4Nuf=Vn_WstToEnd_WstToProjEndRfl4Nuf/norm(Vn_WstToEnd_WstToProjEndRfl4Nuf);

    %�Q�Ϊk�V�q��V �P�_theta7�����V
    temp=V_wst_to_projend_rfl4_nuf'*V_wst_to_end/norm(V_wst_to_projend_rfl4_nuf)/norm(V_wst_to_end);
    if norm(Vn_WstToEnd_WstToProjEndRfl4Nuf - Vn_nuf_rotx5_along_NRfl4Nuf) < DEF_NORM_VERY_SMALL
        theta(6)=-acos(temp); 
    else
        theta(6)=acos(temp); 
    end
      %% ==Axis7== %%
     
    %V_shx�g�L123456�b����������ӭn�P���I�y�Шt��Z�b�K��
    V_x_rot1to6=Ry(-theta(1))*Rx(theta(2))*[V_shx;1];  %�Ĥ@�b�M�j�aZ�y�Ф�V�ۤ�
    temp=Rogridues(theta(3),V_ru_l1/norm(V_ru_l1))*V_x_rot1to6;  
    temp=Rogridues(theta(4),Vn_u_f/norm(Vn_u_f))*temp;  
    temp=Rogridues(theta(5),Vn_rfl4_nuf/norm(Vn_rfl4_nuf))*temp; 
    temp=Rogridues(theta(6),Vn_nuf_rotx5_along_NRfl4Nuf)*temp; 
    V_x_rot1to6=temp(1:3,1); 
    V_x_rot1to6=V_x_rot1to6/norm(V_x_rot1to6);
    
    %xrot1to6 �M V_H_hat_z ���k�V�q�ӧP�_��7�b�����V
    Vn_xrot1to6_VHhatz=cross(V_x_rot1to6,V_H_hat_z);
    Vn_xrot1to6_VHhatz=Vn_xrot1to6_VHhatz/norm(Vn_xrot1to6_VHhatz);
    
    %V_shx�g�L123456�b�����M���I�y�Шt��Z�b�ٮt�X��
    %����bacos(1.000000.....)���ɭԷ|�X�{�곡�����p
    temp=V_x_rot1to6'*V_H_hat_z/norm(V_x_rot1to6)/norm(V_H_hat_z);
    if abs(temp-1)<DEF_COSVAL_VERY_SMALL 
       if temp>0
           temp=1;
       else
           temp=-1;
       end
    end
    
    if norm(Vn_xrot1to6_VHhatz - V_H_hat_x) <  DEF_NORM_VERY_SMALL
        theta(7)=acos(temp);
    else
        theta(7)=-acos(temp);
    end
    
   
    %%  ==���k���1�b��V�ۤ�== %%
    if RLHand == DEF_LEFT_HAND %����M�k��Ĥ@�b��V�ۤ�
        theta(1)=-theta(1);
    end
end

