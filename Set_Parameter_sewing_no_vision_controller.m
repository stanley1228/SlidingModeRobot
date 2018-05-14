clear all
close all
clc


%==============
%Motor parameter
%==============
% J = 0.01 kg.m^2;
% b = 0.1 N.m.s; motor viscous friction constant 
% K = 0.01; electromotive force constant(ke)  0.01 V/rad/sec  motor torque constant(kt) 0.01 N.m/Amp
% R = 1;electric resistance  Ohm
% L = 0.5; electric inductance 

J = [10,10,8,8,4,4,4];%moment of inertia of the motor

P=[350,100,300,150,50,100,20,20];
I=[5,5,5,2,2,2,1,1];
D=[40,40,40,30,20,30,20,20];

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
PathPlanPoint_R=[PathPlanPoint_R(1:3)+TranFrameToRobot PathPlanPoint_R(4:7)];
in_linkL=[L0;L1;L2;L3;L4;L5];
in_base=[0;-L0;0];%header0 �y�Шt������shoulder0 �y�Шt �tY��V��L0
in_end=[PathPlanPoint_R(1);PathPlanPoint_R(2);PathPlanPoint_R(3)];
in_PoseAngle=[PathPlanPoint_R(4)*pi/180;PathPlanPoint_R(5)*pi/180;PathPlanPoint_R(6)*pi/180];
Rednt_alpha_R=PathPlanPoint_R(7)*pi/180;
theta_R=IK_7DOF_FB7roll(DEF_RIGHT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_R);
IniJointAngle_R=theta_R;

PathPlanPoint_L=[[-90  90 0] [-90  0 0]  90];
PathPlanPoint_L=[PathPlanPoint_L(1:3)+TranFrameToRobot PathPlanPoint_L(4:7)];
in_linkL=[L0;L1;L2;L3;L4;L5];
in_base=[0;L0;0];%header0 �y�Шt������shoulder0 �y�Шt �tY��V��L0
in_end=[PathPlanPoint_L(1);PathPlanPoint_L(2);PathPlanPoint_L(3)];
in_PoseAngle=[PathPlanPoint_L(4)*pi/180;PathPlanPoint_L(5)*pi/180;PathPlanPoint_L(6)*pi/180];
Rednt_alpha_L=PathPlanPoint_L(7)*pi/180;
theta_L=IK_7DOF_FB7roll(DEF_LEFT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_L);
IniJointAngle_L=theta_L;

% global PathPlanPointRec_R;
% PathPlanPointRec_R=PathPlanPoint_R;

%% �T�w�Ѽ�
DEF_ROBOT_COOR=1;
DEF_OBJFRAME_COOR=2;

x_base_R=0;   %����I
y_base_R=0;
z_base_R=0;

x_base_L=0;   %����I
y_base_L=0;
z_base_L=0;

%set_param('PID_Dual_Robot', 'StopTime','30')
%set_param('PID_Dual_Robot', 'FixedStep','0.2')
FixedStep_as_str=get_param('PID_Dual_Robot_no_vision_controller', 'FixedStep');
DEF_CYCLE_TIME=str2double(FixedStep_as_str);

Pcnt=1;%��X�`��

HoldLen_L=[180 0 0];%�������I���Z �ѮبM�w
HoldLen_R=[180 0 0];%�������I���Z �ѮبM�w
%% �U�Ϭq���I��  �N���|�I�b��l�ɶ��إߦn
%�o��O�ϥά[�l���Шt�A��LineMoveToScript�̭��~���ഫ

TotalTime=0;
Seg=0;
abst=0;
PathPlanPointRec_R=timeseries;
PathPlanPointRec_L=timeseries;
ObjCornerRec_raw=timeseries;
% %�����} ��
% disp('footlifter up');
% 
% %�k�⧨ ���⧨
% disp('right hold');disp('left hold');
% 
% %�����} ��
% disp('footlifter down');
% 
% %�D�b�Ұ�
% disp('spindle on');

%�k�⩹��X SewingLenth ���⩹��X �_�u���� SewingLenth
FRAME_UPDATE=true;%�[�lø��
R_starP=[[-90 -90 0] [50  0 0] -50]; 
R_endP=[[-90+SewingLength -90 0]  70 0 0 -50]; 
L_starP=[[-90  90 0] [-90  0 0]  90];
L_endP=[[-90+SewingLength  90 0] [-90 0 0]  90];
CostTime=3;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

% %�D�b����
% disp('spindle off');
% 
% %�k�⤣�� ����}
% disp('left release');

%�k�⤣�� ���⩹��y���� 
FRAME_UPDATE=false;
R_starP=[[-90+SewingLength -90 0]  [70 0 0] -50]; 
R_endP=[[-90+SewingLength -90 0]  [50 0 0] -50]; 
L_starP=[[-90+SewingLength  90 0] [-90 0 0]  90];
L_endP= [[-90+SewingLength  90+MovOutLen 0] [-90 0 0]  90];
CostTime=3;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%�k�⤣�� ���⩹��X ����I���j����(Release move length)
R_starP=[[-90+SewingLength -90 0]  [50 0 0] -50];
R_endP=[[-90+SewingLength -90 0]  [50 0 0] -50];
L_starP=[[-90+SewingLength  90+MovOutLen 0] [-90 0 0]  90];
L_endP=[[-90+SewingLength+RelMovLen  90+MovOutLen 0] [-60 0 0]  90];
CostTime=3;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%�k�⤣�� ���⩹�ty����MovOutLen
R_starP=[[-90+SewingLength -90 0]  [50 0 0] -50];
R_endP=[[-90+SewingLength -90 0]  [50 0 0] -50];
L_starP=[[-90+SewingLength+RelMovLen  90+MovOutLen 0] [-60 0 0]  90];
L_endP=[[-90+SewingLength+RelMovLen  90 0] [-60 0 0]  90];
CostTime=3;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

% %�k�⤣�� ���⧨
% disp('left hold');
% 
% %�����}��
% disp('footlifter up')

%�k����੹��X ������੹�tX
FRAME_UPDATE=true;
arc_cen=Needle_ini_Plate; %�����߬��w�b�[�l�W���_�l�I
R_starP=[[-90+SewingLength -90 0]  [50 0 0] -50];
R_endP=[[90 -90 0] [50 0 0] -50];
L_starP=[[-90+SewingLength+RelMovLen  90 0] [-60 0 0]  90];
L_endP=[[-90 90 0] -90 0 0  90];
rot_rad=0.5*pi; %����ɪ��_�l���ਤ��
CostTime=6;
Coordinate=DEF_OBJFRAME_COOR;
RotateMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

% %�����}��
% disp('footlifter down');
% 
% %�k��} ���⤣��1
% disp('left release');

%�k�⩹X�tY�t���X  ���⤣��1 
FRAME_UPDATE=false;
R_starP=[[90 -90 0] [50 0 0] -50  ];
R_endP=[[90-MovOutLen -90-MovOutLen 0]  [50 0 0] -70];
L_starP=[[-90 90 0] -90 0 0  90];
L_endP=[[-90 90 0] -90 0 0  90];
CostTime=3;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%�k�⩹X�t����RelMovLen  ���⤣��1 
R_starP=[[90-MovOutLen -90-MovOutLen 0]  [50 0 0] -70];
R_endP=[[90-MovOutLen-RelMovLen -90-MovOutLen 0]  [50 0 0] -70];
L_starP=[[-90 90 0] -90 0 0  90];
L_endP=[[-90 90 0] -90 0 0  90];
CostTime=4;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%�k�⩹X��Y��MovOutLen  ���⤣��1 
R_starP=[[90-MovOutLen-RelMovLen -90-MovOutLen 0]  [50 0 0] -70];
R_endP=[[90-RelMovLen -90 0]  [50 0 0] -70];
L_starP=[[-90 90 0] -90 0 0  90];
L_endP=[[-90 90 0] -90 0 0  90];
CostTime=3;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;


StopTime_str=num2str(TotalTime);
set_param('PID_Dual_Robot_no_vision_controller', 'StopTime',StopTime_str)

% %�k�⧨ ���⤣��1
% disp('right hold');
%   
% set_param('modelname', 'StopTime', '3000')


