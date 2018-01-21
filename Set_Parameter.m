clear all
%==============
%Motor parameter
%==============
% J = 3.2284e-6;
% b = 3.5077e-6;
% K = 0.0274;
% R = 4;
% L = 2.75e-6;
J = 0.01;%moment of inertia of the rotor
b = 0.1; %motor viscous friction constant
K = 0.01;%electromotive force constant   %motor torque constant  
R = 1;   %electric resistance  
L = 0.5; %electric resistance    
%command=[10,12,14,16,18,20,22];

%==============
%counter
%==============
global Pcnt;%stanley
Pcnt=1;

%==============
%sewing offset
%==============
global Needle_RobotF;%�w�I�b���u���Шt��m   
global Needle_ini_Plate;%�U�w�I�b�[�lplate�y�Шt�W����l�I
global TranFrameToRobot;%�Q�Ψ�Ӫ��t�ȥh�����

global MovOutLen;%���X����I������
global SewingLength;%�_����{
global RelMovLen;%�ج[����I���Z

Needle_RobotF=[350 -300 30];%�w�I�b���u���Шt��m   
Needle_ini_Plate=[30 -30 0];%�U�w�I�b�[�lplate�y�Шt�W����l�I
TranFrameToRobot=Needle_RobotF-Needle_ini_Plate;%�Q�Ψ�Ӫ��t�ȥh�����

MovOutLen=50;%���X����I������
SewingLength=60;%�_����{
RelMovLen=180;%�ج[����I���Z

%==============
%initial motor angle
%==============
global DEF_RIGHT_HAND;
global DEF_LEFT_HAND;
global L0;   %�Y��ӻH
global L1;
global L2;
global L3;
global L4;
global L5;

DEF_RIGHT_HAND=1;
DEF_LEFT_HAND=2;
L0=248;   %�Y��ӻH
L1=250;   %L�� ����
L2=25;    %L�� �u��
L3=25;    %L�� �u��
L4=230;   %L�� ���� 
L5=195;   %��end-effector

PathPlanPoint_R=[[-90 -90 0] [50  0 0] -50]; 
PathPlanPoint_R=[PathPlanPoint_R(1:3)+TranFrameToRobot PathPlanPoint_R(4:7)]
in_linkL=[L0;L1;L2;L3;L4;L5];
in_base=[0;-L0;0];%header0 �y�Шt������shoulder0 �y�Шt �tY��V��L0
in_end=[PathPlanPoint_R(1);PathPlanPoint_R(2);PathPlanPoint_R(3)];
in_PoseAngle=[PathPlanPoint_R(4)*pi/180;PathPlanPoint_R(5)*pi/180;PathPlanPoint_R(6)*pi/180];
Rednt_alpha_R=PathPlanPoint_R(7)*pi/180;
theta_R=IK_7DOF_FB7roll(DEF_RIGHT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_R);
IniJointAngle=theta_R;

global PathPlanPointRec_R;
PathPlanPointRec_R=PathPlanPoint_R;
%Just fot test
a1=10;
a2=20;
arr1=[31 54];


