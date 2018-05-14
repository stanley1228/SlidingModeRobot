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

PathPlanPoint_R=[[-90 -90 0] [50  0 0] -50]; 
PathPlanPoint_R=[PathPlanPoint_R(1:3)+TranFrameToRobot PathPlanPoint_R(4:7)];
in_linkL=[L0;L1;L2;L3;L4;L5];
in_base=[0;-L0;0];%header0 座標系偏移到shoulder0 座標系 差Y方向的L0
in_end=[PathPlanPoint_R(1);PathPlanPoint_R(2);PathPlanPoint_R(3)];
in_PoseAngle=[PathPlanPoint_R(4)*pi/180;PathPlanPoint_R(5)*pi/180;PathPlanPoint_R(6)*pi/180];
Rednt_alpha_R=PathPlanPoint_R(7)*pi/180;
theta_R=IK_7DOF_FB7roll(DEF_RIGHT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_R);
IniJointAngle_R=theta_R;

PathPlanPoint_L=[[-90  90 0] [-90  0 0]  90];
PathPlanPoint_L=[PathPlanPoint_L(1:3)+TranFrameToRobot PathPlanPoint_L(4:7)];
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
FixedStep_as_str=get_param('PID_Dual_Robot_no_vision_controller', 'FixedStep');
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
% %抬壓腳 抬
% disp('footlifter up');
% 
% %右手夾 左手夾
% disp('right hold');disp('left hold');
% 
% %抬壓腳 壓
% disp('footlifter down');
% 
% %主軸啟動
% disp('spindle on');

%右手往正X SewingLenth 左手往正X 縫線長度 SewingLenth
FRAME_UPDATE=true;%架子繪圖
R_starP=[[-90 -90 0] [50  0 0] -50]; 
R_endP=[[-90+SewingLength -90 0]  70 0 0 -50]; 
L_starP=[[-90  90 0] [-90  0 0]  90];
L_endP=[[-90+SewingLength  90 0] [-90 0 0]  90];
CostTime=3;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

% %主軸停止
% disp('spindle off');
% 
% %右手不動 左手開
% disp('left release');

%右手不動 左手往正y移動 
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

%右手不動 左手往正X 抓取點間隔長度(Release move length)
R_starP=[[-90+SewingLength -90 0]  [50 0 0] -50];
R_endP=[[-90+SewingLength -90 0]  [50 0 0] -50];
L_starP=[[-90+SewingLength  90+MovOutLen 0] [-90 0 0]  90];
L_endP=[[-90+SewingLength+RelMovLen  90+MovOutLen 0] [-60 0 0]  90];
CostTime=3;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%右手不動 左手往負y移動MovOutLen
R_starP=[[-90+SewingLength -90 0]  [50 0 0] -50];
R_endP=[[-90+SewingLength -90 0]  [50 0 0] -50];
L_starP=[[-90+SewingLength+RelMovLen  90+MovOutLen 0] [-60 0 0]  90];
L_endP=[[-90+SewingLength+RelMovLen  90 0] [-60 0 0]  90];
CostTime=3;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

% %右手不動 左手夾
% disp('left hold');
% 
% %抬壓腳抬
% disp('footlifter up')

%右手旋轉往正X 左手旋轉往負X
FRAME_UPDATE=true;
arc_cen=Needle_ini_Plate; %旋轉圓心為針在架子上的起始點
R_starP=[[-90+SewingLength -90 0]  [50 0 0] -50];
R_endP=[[90 -90 0] [50 0 0] -50];
L_starP=[[-90+SewingLength+RelMovLen  90 0] [-60 0 0]  90];
L_endP=[[-90 90 0] -90 0 0  90];
rot_rad=0.5*pi; %旋轉時的起始旋轉角度
CostTime=6;
Coordinate=DEF_OBJFRAME_COOR;
RotateMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

% %抬壓腳壓
% disp('footlifter down');
% 
% %右手開 左手不動1
% disp('left release');

%右手往X負Y負移出  左手不動1 
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

%右手往X負移動RelMovLen  左手不動1 
R_starP=[[90-MovOutLen -90-MovOutLen 0]  [50 0 0] -70];
R_endP=[[90-MovOutLen-RelMovLen -90-MovOutLen 0]  [50 0 0] -70];
L_starP=[[-90 90 0] -90 0 0  90];
L_endP=[[-90 90 0] -90 0 0  90];
CostTime=4;
Coordinate=DEF_OBJFRAME_COOR;
LineMoveTo_PathGen_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%右手往X往Y正MovOutLen  左手不動1 
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

% %右手夾 左手不動1
% disp('right hold');
%   
% set_param('modelname', 'StopTime', '3000')


