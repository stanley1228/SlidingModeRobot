function [sys,x0,str,ts,simStateCompliance] = FK_7DOF_FB7roll_Sfun(t,x,u,flag)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u)

sys = [];


function sys=mdlUpdate(t,x,u)

sys = [];



function sys=mdlOutputs(t,x,u)

    %from input
    theta_R=u;

    %calculate FK
    global DEF_RIGHT_HAND;
    global DEF_LEFT_HAND;
    global L0;
    global L1;
    global L2;
    global L3;
    global L4;
    global L5;
    
    x_base_R=0;   %基準點
    y_base_R=0;
    z_base_R=0;

    x_base_L=0;   %基準點
    y_base_L=0;
    z_base_L=0;
    
    %out_Rednt_alpha_R=0.5*pi;%temp for test. i dont know how to calculate from FK yes
    global ArmJoint_R;
    global RotationM_R;
    global PathIFKPointRec_R;
    global Pcnt;
    [out_x_end_R,out_y_end_R,out_z_end_R,out_alpha_R,out_beta_R,out_gamma_R,ArmJoint_R,RotationM_R] = FK_7DOF_FB7roll(DEF_RIGHT_HAND,L0,L1,L2,L3,L4,L5,x_base_R,y_base_R,z_base_R,theta_R);
    
    Rednt_alpha_R=-50;%Rednt_alpha_R 還不會算，直接用跟規劃的依樣
    PathIFKPoint_R=[out_x_end_R out_y_end_R out_z_end_R out_alpha_R out_beta_R out_gamma_R Rednt_alpha_R];%Rednt_alpha_R 還不會算，直接用跟規劃的依樣
    PathIFKPointRec_R(Pcnt,1:7)=PathIFKPoint_R;
    %evalin('base','a=1258');
    %assignin('base','a(length(a)+1)',5)
    %Draw_7DOF_FB7roll_point_script;
    
sys = [out_x_end_R,out_y_end_R,out_z_end_R];%now just consider x,y,z



function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;



function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
