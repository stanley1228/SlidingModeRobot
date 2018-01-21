 %?是一?采用Matlab level-2 S-function所?的???信??行??理的?本文件Vision_data_reforming_S_func
% ?入?15?vision target 向量
% ?出?有信息的vision target，融在一?矩?中
% ?前??S-function适用于模型Intelligent_Driving_Data_Fusion

function Vision_data_reforming_S_func(block)
  setup(block);
  
%endfunction

function setup(block)
  
  block.NumDialogPrms  = 0;    
  
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 3;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).Dimensions        = [120 1]; %?里的dimension最好?是指明几行几列
  block.InputPort(1).DirectFeedthrough = false; 
  
  block.OutputPort(1).Dimensions       = [8 15]; %?里的dimension最好?是指明几行几列
  block.OutputPort(2).Dimensions       = [8 15]; %?里的dimension最好?是指明几行几列
  block.OutputPort(3).Dimensions       = 1;
  
  %% Set block sample time to [-1 0]
  block.SampleTimes = [0.05 0]; %??采?rate的?定一定要和前面的保持一致，要不?据?出?偏差
  % [0 0] ??采???
  % [-1 0] ?承S函??入信?或父?模型的采???
  % [0.5, 0.1] 离散采???，?0.1秒?始，每0.5秒采?一次
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Update',                  @Update);
  block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
  
%endfunction

function DoPostPropSetup(block)

  %% Setup Dwork
  block.NumDworks = 17; % ?里一定要?得更改，如果添加了新的Dwork vector
  
  block.Dwork(1).Name = 'Vision_target_info_1'; % 信息?自Vision sensor 1
  block.Dwork(1).Dimensions      =8; % ?里好像默?是一?列向量，然后value代表行
  % Dwork must be a vector, and its width must be a positive integer
  block.Dwork(1).DatatypeID      = 0; 
  % 端口?据?型，可以端口ID指定，也可以直接指定?据?型名
  % inherited: -1        int16: 4
  % double: 0            unit16: 5
  % single: 1            int32: 6
  % int8: 2              unit32: 7
  % unit8: 3             boolean或定??型: 8
  % The data type of Dwork(1) in 'learning_2/Level-2 MATLAB S-Function1' 
  % cannot be dynamically-typed. Data type of Dwork must be one of the MATLAB 'uint8',
  % 'uint16', 'uint32', 'int8', 'int16', 'int32', 'single', or 'double' data types
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

  block.Dwork(2).Name = 'Vision_target_info_2'; % 信息?自Vision sensor #2
  block.Dwork(2).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  % Dwork must be a vector, and its width must be a positive integer
  block.Dwork(2).DatatypeID      = 0; 
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name = 'Vision_target_info_3'; % 信息?自Vision sensor #3
  block.Dwork(3).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(3).DatatypeID      = 0; 
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name = 'Vision_target_info_4'; % 信息?自Vision sensor #4
  block.Dwork(4).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(4).DatatypeID      = 0; 
  block.Dwork(4).Complexity      = 'Real';
  block.Dwork(4).UsedAsDiscState = true;
  
  block.Dwork(5).Name = 'Vision_target_info_5'; % 信息?自Vision sensor #5
  block.Dwork(5).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(5).DatatypeID      = 0; 
  block.Dwork(5).Complexity      = 'Real';
  block.Dwork(5).UsedAsDiscState = true;
  
  block.Dwork(6).Name = 'Vision_target_info_6'; % 信息?自Vision sensor #6
  block.Dwork(6).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(6).DatatypeID      = 0; 
  block.Dwork(6).Complexity      = 'Real';
  block.Dwork(6).UsedAsDiscState = true;
  
  block.Dwork(7).Name = 'Vision_target_info_7'; % 信息?自Vision sensor #7
  block.Dwork(7).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(7).DatatypeID      = 0; 
  block.Dwork(7).Complexity      = 'Real';
  block.Dwork(7).UsedAsDiscState = true;
  
  block.Dwork(8).Name = 'Vision_target_info_8'; % 信息?自Vision sensor #8
  block.Dwork(8).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(8).DatatypeID      = 0; 
  block.Dwork(8).Complexity      = 'Real';
  block.Dwork(8).UsedAsDiscState = true;
  
  block.Dwork(9).Name = 'Vision_target_info_9'; % 信息?自Vision sensor #9
  block.Dwork(9).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(9).DatatypeID      = 0; 
  block.Dwork(9).Complexity      = 'Real';
  block.Dwork(9).UsedAsDiscState = true;
  
  block.Dwork(10).Name = 'Vision_target_info_10'; % 信息?自Vision sensor #10
  block.Dwork(10).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(10).DatatypeID      = 0; 
  block.Dwork(10).Complexity      = 'Real';
  block.Dwork(10).UsedAsDiscState = true;
  
  block.Dwork(11).Name = 'Vision_target_info_11'; % 信息?自Vision sensor #11 
  block.Dwork(11).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(11).DatatypeID      = 0; 
  block.Dwork(11).Complexity      = 'Real';
  block.Dwork(11).UsedAsDiscState = true;
  
  block.Dwork(12).Name = 'Vision_target_info_12'; % 信息?自Vision sensor #12
  block.Dwork(12).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(12).DatatypeID      = 0; 
  block.Dwork(12).Complexity      = 'Real';
  block.Dwork(12).UsedAsDiscState = true;
  
  block.Dwork(13).Name = 'Vision_target_info_13'; % 信息?自Vision sensor #13
  block.Dwork(13).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(13).DatatypeID      = 0; 
  block.Dwork(13).Complexity      = 'Real';
  block.Dwork(13).UsedAsDiscState = true;
  
  block.Dwork(14).Name = 'Vision_target_info_14'; % 信息?自Vision sensor #14
  block.Dwork(14).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(14).DatatypeID      = 0; 
  block.Dwork(14).Complexity      = 'Real';
  block.Dwork(14).UsedAsDiscState = true;
  
  block.Dwork(15).Name = 'Vision_target_info_15'; % 信息?自Vision sensor #15
  block.Dwork(15).Dimensions      = 8; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(15).DatatypeID      = 0; 
  block.Dwork(15).Complexity      = 'Real';
  block.Dwork(15).UsedAsDiscState = true;
  
  block.Dwork(16).Name = 'Vision_target_info_all_input'; % 假??入信息?自Vision sensor
  block.Dwork(16).Dimensions      = 120; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(16).DatatypeID      = 0; 
  block.Dwork(16).Complexity      = 'Real';
  block.Dwork(16).UsedAsDiscState = true;
  
  block.Dwork(17).Name = 'Vision_targets_number'; % Vision targets??
  block.Dwork(17).Dimensions      = 1; % ?里好像默?是一?列向量，然后value代表行
  block.Dwork(17).DatatypeID      = 0; 
  block.Dwork(17).Complexity      = 'Real';
  block.Dwork(17).UsedAsDiscState = true;
%endfunction

function InitConditions(block)

  %% Initialize Dwork
% block.Dwork(1).Data = block.DialogPrm(1).Data;
% ??一般是?S-function中填?的parameter?值?Dwork，作?初始值

block.Dwork(16).Data = block.InputPort(1).Data;

%endfunction

function Output(block)

Num_vision_targets = 0;
% ?1?120*1的列向量分拆，并且?值?各?Dwork向量
for i=1:1:15
    for j=1:1:8
        block.Dwork(i).Data(j) = block.Dwork(16).Data(j+(i-1)*8);
        
        if j == 2 && block.Dwork(16).Data(j+(i-1)*8) ~= 0 %?里的j=2取得是confidence level
            %?算有效??的vision target??，依据的是condidenc level
            Num_vision_targets = Num_vision_targets + 1;
        end
    end
end

block.Dwork(17).Data = Num_vision_targets;
counter = 1;
temp = Num_vision_targets;

for i=1:1:15
    if block.Dwork(i).Data(2) ~= 0
        A(:,temp) = block.Dwork(i).Data;
        temp = temp - 1;
    else
        A(:,Num_vision_targets+counter) = block.Dwork(i).Data;
        counter = counter + 1;
    end
end

for i=1:1:15
    B(:,i) = block.Dwork(i).Data;
end
        
block.OutputPort(1).Data = A;
block.OutputPort(2).Data = B;
block.OutputPort(3).Data = block.Dwork(17).Data;
% ???出好像必?要?在??局部的函??中，要不然??x_kal_filter?量?有被定? 
%endfunction

function Update(block)

  block.Dwork(16).Data = block.InputPort(1).Data;
  % 做累加器，要不然每次?用initiate函?就????Dwork(2)便?默?值，默?值好像是0
  % ???句感??是很tricky的，不知道?什么如果在output??函?中做累加，好像就是加不上去
%endfunction

function SetInputPortSamplingMode(block, idx, fd)
block.InputPort(idx).SamplingMode = fd;
block.InputPort(idx).SamplingMode = fd;

block.OutputPort(1).SamplingMode = fd;
block.OutputPort(2).SamplingMode = fd;
block.OutputPort(3).SamplingMode = fd;
%endfunction