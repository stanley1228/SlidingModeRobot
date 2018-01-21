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
PathPlanPoint_R=[PathPlanPoint_R(1:3)+TranFrameToRobot PathPlanPoint_R(4:7)]
in_linkL=[L0;L1;L2;L3;L4;L5];
in_base=[0;-L0;0];%header0 座標系偏移到shoulder0 座標系 差Y方向的L0
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


