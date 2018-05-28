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

J = [0.016,0.016,0.012,0.012,0.01,0.01,0.01];%moment of inertia of the motor
%J = [0.01,0.01,0.01,0.01,0.01,0.01,0.01];%ok
b = 0.1; %motor viscous friction constant
K = 0.01;%ke electromotive force constant   %kt motor torque constant  
R = 1;   %electric resistance  
L = 0.5; %electric inductance    
%command=[10,12,14,16,18,20,22];

P=[60,60,50,50,30,30,20,20];
I=[10,10,5,5,5,2,2,2];
D=[30,30,20,20,20,20,20,20];

TOP_P=5;
TOP_I=1;
TOP_D=0.9;
% P=[20,20,20,20,20,20,20,20];%ok
% D=[20,20,20,20,20,20,20,20];%ok
%==============
%counter
%==============
global Pcnt;%stanley
Pcnt=1;

%==============
%sewing offset
%==============
global Needle_RobotF;%針點在手臂坐標系位置   
global Needle_ini_Plate;%下針點在架子plate座標系上的初始點
global TranFrameToRobot;%利用兩個的差值去做比較

global MovOutLen;%移出抓取點的長度
global SewingLength;%縫紉行程 
global RelMovLen;%框架抓取點間距

Needle_RobotF=[350 -300 30];%針點在手臂坐標系位置   
Needle_ini_Plate=[30 -30 0];%下針點在架子plate座標系上的初始點
TranFrameToRobot=Needle_RobotF-Needle_ini_Plate;%利用兩個的差值去做比較

MovOutLen=50;%移出抓取點的長度
SewingLength=60;%縫紉行程
RelMovLen=180;%框架抓取點間距

%==============
%initial motor angle
%==============
global DEF_RIGHT_HAND;
global DEF_LEFT_HAND;
global L0;   %頭到肩膀
global L1;
global L2;
global L3;
global L4;
global L5;

DEF_RIGHT_HAND=1;
DEF_LEFT_HAND=2;
L0=248;   %頭到肩膀
L1=250;   %L型 長邊
L2=25;    %L型 短邊
L3=25;    %L型 短邊
L4=230;   %L型 長邊 
L5=195;   %到end-effector

PathPlanPoint_R=[[210 -360 0] [50 0 0] -50]; 
% PathPlanPoint_R=[PathPlanPoint_R(1:3)+TranFrameToRobot PathPlanPoint_R(4:7)];
in_linkL=[L0;L1;L2;L3;L4;L5];
in_base=[0;-L0;0];%header0 座標系偏移到shoulder0 座標系 差Y方向的L0
in_end=[PathPlanPoint_R(1);PathPlanPoint_R(2);PathPlanPoint_R(3)];
in_PoseAngle=[PathPlanPoint_R(4)*pi/180;PathPlanPoint_R(5)*pi/180;PathPlanPoint_R(6)*pi/180];
Rednt_alpha_R=PathPlanPoint_R(7)*pi/180;
theta_R=IK_7DOF_FB7roll(DEF_RIGHT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_R);
IniJointAngle_R=theta_R;

PathPlanPoint_L=[[350  100 0] [-60 0 0]  60];
% PathPlanPoint_L=[PathPlanPoint_L(1:3)+TranFrameToRobot PathPlanPoint_L(4:7)];
in_linkL=[L0;L1;L2;L3;L4;L5];
in_base=[0;L0;0];%header0 座標系偏移到shoulder0 座標系 差Y方向的L0
in_end=[PathPlanPoint_L(1);PathPlanPoint_L(2);PathPlanPoint_L(3)];
in_PoseAngle=[PathPlanPoint_L(4)*pi/180;PathPlanPoint_L(5)*pi/180;PathPlanPoint_L(6)*pi/180];
Rednt_alpha_L=PathPlanPoint_L(7)*pi/180;
theta_L=IK_7DOF_FB7roll(DEF_LEFT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_L);
IniJointAngle_L=theta_L;

% global PathPlanPointRec_R;
% PathPlanPointRec_R=PathPlanPoint_R;

%% 固定參數
DEF_ROBOT_COOR=1;
DEF_OBJFRAME_COOR=2;

x_base_R=0;   %基準點
y_base_R=0;
z_base_R=0;

x_base_L=0;   %基準點
y_base_L=0;
z_base_L=0;

%set_param('PID_Dual_Robot', 'StopTime','30')
%set_param('PID_Dual_Robot', 'FixedStep','0.2')
FixedStep_as_str=get_param('PID_Dual_Robot', 'FixedStep');
DEF_CYCLE_TIME=str2double(FixedStep_as_str);

Pcnt=1;%輸出總數

HoldLen_L=[180 0 0];%左手抓取點間距 由框決定
HoldLen_R=[180 0 0];%左手抓取點間距 由框決定
%% 各區段的點位  將路徑點在初始時間建立好
%這邊是使用架子坐標系，到LineMoveToScript裡面才做轉換

TotalTime=0;
Seg=0;
abst=0;
PathPlanPointRec_R=timeseries;
PathPlanPointRec_L=timeseries;
ObjCornerRec_raw=timeseries;

% R_p=[   210 -360 0  50 -90 0 -50;
%         350 -360 0  50 -90 0 -50;
%         350 -360 0  50 -90 0 -50;
%         390 -360 0  50 -90 0 -50
%         210 -360 0  50 -90 0 -50];

%右手畫直線
FRAME_UPDATE=true;%架子繪圖
R_starP=[[210 -360 0] [50 0 0] -50]; 
R_endP=[[450 -360 0]  [50 0 0] -50]; 
arm_sel = DEF_RIGHT_HAND;
CostTime=12;
Coordinate=DEF_ROBOT_COOR;
LineMoveTo_PathGen_Script_SingleArm;
%TotalTime=TotalTime+CostTime;

abst=0;

%左手畫圓
arc_cen=[350,200,0]; %旋轉圓心為針在架子上的起始點
L_starP=[[350  100 0] [-60 0 0]  60];
L_endP=[[350,100,0] [-60 0 0]  60];
arm_sel = DEF_LEFT_HAND;
rot_rad=2*pi; %旋轉角度
CostTime=12;
Coordinate=DEF_ROBOT_COOR;
RotateMoveTo_PathGen_Script_SingleArm;
%TotalTime=TotalTime+CostTime;

TotalTime=12;


% %抬壓腳壓
% disp('footlifter down');
% 


StopTime_str=num2str(TotalTime);
set_param('PID_Dual_Robot', 'StopTime',StopTime_str)

% %右手夾 左手不動1
% disp('right hold');
%   
% set_param('modelname', 'StopTime', '3000')


