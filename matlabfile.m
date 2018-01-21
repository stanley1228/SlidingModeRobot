 %?�O�@?����Matlab level-2 S-function��?��???�H??��??�z��?�����Vision_data_reforming_S_func
% ?�J?15?vision target �V�q
% ?�X?���H����vision target�A�Ħb�@?�x?��
% ?�e??S-function��Τ_�ҫ�Intelligent_Driving_Data_Fusion

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

  block.InputPort(1).Dimensions        = [120 1]; %?����dimension�̦n?�O�����L��L�C
  block.InputPort(1).DirectFeedthrough = false; 
  
  block.OutputPort(1).Dimensions       = [8 15]; %?����dimension�̦n?�O�����L��L�C
  block.OutputPort(2).Dimensions       = [8 15]; %?����dimension�̦n?�O�����L��L�C
  block.OutputPort(3).Dimensions       = 1;
  
  %% Set block sample time to [-1 0]
  block.SampleTimes = [0.05 0]; %??��?rate��?�w�@�w�n�M�e�����O���@�P�A�n��?�u?�X?���t
  % [0 0] ??��???
  % [-1 0] ?��S��??�J�H?�Τ�?�ҫ�����???
  % [0.5, 0.1] �ô���???�A?0.1��?�l�A�C0.5���?�@��
  
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
  block.NumDworks = 17; % ?���@�w�n?�o���A�p�G�K�[�F�s��Dwork vector
  
  block.Dwork(1).Name = 'Vision_target_info_1'; % �H��?��Vision sensor 1
  block.Dwork(1).Dimensions      =8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  % Dwork must be a vector, and its width must be a positive integer
  block.Dwork(1).DatatypeID      = 0; 
  % �ݤf?�u?���A�i�H�ݤfID���w�A�]�i�H�������w?�u?���W
  % inherited: -1        int16: 4
  % double: 0            unit16: 5
  % single: 1            int32: 6
  % int8: 2              unit32: 7
  % unit8: 3             boolean�Ωw??��: 8
  % The data type of Dwork(1) in 'learning_2/Level-2 MATLAB S-Function1' 
  % cannot be dynamically-typed. Data type of Dwork must be one of the MATLAB 'uint8',
  % 'uint16', 'uint32', 'int8', 'int16', 'int32', 'single', or 'double' data types
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

  block.Dwork(2).Name = 'Vision_target_info_2'; % �H��?��Vision sensor #2
  block.Dwork(2).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  % Dwork must be a vector, and its width must be a positive integer
  block.Dwork(2).DatatypeID      = 0; 
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name = 'Vision_target_info_3'; % �H��?��Vision sensor #3
  block.Dwork(3).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(3).DatatypeID      = 0; 
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name = 'Vision_target_info_4'; % �H��?��Vision sensor #4
  block.Dwork(4).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(4).DatatypeID      = 0; 
  block.Dwork(4).Complexity      = 'Real';
  block.Dwork(4).UsedAsDiscState = true;
  
  block.Dwork(5).Name = 'Vision_target_info_5'; % �H��?��Vision sensor #5
  block.Dwork(5).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(5).DatatypeID      = 0; 
  block.Dwork(5).Complexity      = 'Real';
  block.Dwork(5).UsedAsDiscState = true;
  
  block.Dwork(6).Name = 'Vision_target_info_6'; % �H��?��Vision sensor #6
  block.Dwork(6).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(6).DatatypeID      = 0; 
  block.Dwork(6).Complexity      = 'Real';
  block.Dwork(6).UsedAsDiscState = true;
  
  block.Dwork(7).Name = 'Vision_target_info_7'; % �H��?��Vision sensor #7
  block.Dwork(7).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(7).DatatypeID      = 0; 
  block.Dwork(7).Complexity      = 'Real';
  block.Dwork(7).UsedAsDiscState = true;
  
  block.Dwork(8).Name = 'Vision_target_info_8'; % �H��?��Vision sensor #8
  block.Dwork(8).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(8).DatatypeID      = 0; 
  block.Dwork(8).Complexity      = 'Real';
  block.Dwork(8).UsedAsDiscState = true;
  
  block.Dwork(9).Name = 'Vision_target_info_9'; % �H��?��Vision sensor #9
  block.Dwork(9).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(9).DatatypeID      = 0; 
  block.Dwork(9).Complexity      = 'Real';
  block.Dwork(9).UsedAsDiscState = true;
  
  block.Dwork(10).Name = 'Vision_target_info_10'; % �H��?��Vision sensor #10
  block.Dwork(10).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(10).DatatypeID      = 0; 
  block.Dwork(10).Complexity      = 'Real';
  block.Dwork(10).UsedAsDiscState = true;
  
  block.Dwork(11).Name = 'Vision_target_info_11'; % �H��?��Vision sensor #11 
  block.Dwork(11).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(11).DatatypeID      = 0; 
  block.Dwork(11).Complexity      = 'Real';
  block.Dwork(11).UsedAsDiscState = true;
  
  block.Dwork(12).Name = 'Vision_target_info_12'; % �H��?��Vision sensor #12
  block.Dwork(12).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(12).DatatypeID      = 0; 
  block.Dwork(12).Complexity      = 'Real';
  block.Dwork(12).UsedAsDiscState = true;
  
  block.Dwork(13).Name = 'Vision_target_info_13'; % �H��?��Vision sensor #13
  block.Dwork(13).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(13).DatatypeID      = 0; 
  block.Dwork(13).Complexity      = 'Real';
  block.Dwork(13).UsedAsDiscState = true;
  
  block.Dwork(14).Name = 'Vision_target_info_14'; % �H��?��Vision sensor #14
  block.Dwork(14).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(14).DatatypeID      = 0; 
  block.Dwork(14).Complexity      = 'Real';
  block.Dwork(14).UsedAsDiscState = true;
  
  block.Dwork(15).Name = 'Vision_target_info_15'; % �H��?��Vision sensor #15
  block.Dwork(15).Dimensions      = 8; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(15).DatatypeID      = 0; 
  block.Dwork(15).Complexity      = 'Real';
  block.Dwork(15).UsedAsDiscState = true;
  
  block.Dwork(16).Name = 'Vision_target_info_all_input'; % ��??�J�H��?��Vision sensor
  block.Dwork(16).Dimensions      = 120; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(16).DatatypeID      = 0; 
  block.Dwork(16).Complexity      = 'Real';
  block.Dwork(16).UsedAsDiscState = true;
  
  block.Dwork(17).Name = 'Vision_targets_number'; % Vision targets??
  block.Dwork(17).Dimensions      = 1; % ?���n���q?�O�@?�C�V�q�A�M�Zvalue�N���
  block.Dwork(17).DatatypeID      = 0; 
  block.Dwork(17).Complexity      = 'Real';
  block.Dwork(17).UsedAsDiscState = true;
%endfunction

function InitConditions(block)

  %% Initialize Dwork
% block.Dwork(1).Data = block.DialogPrm(1).Data;
% ??�@��O?S-function����?��parameter?��?Dwork�A�@?��l��

block.Dwork(16).Data = block.InputPort(1).Data;

%endfunction

function Output(block)

Num_vision_targets = 0;
% ?1?120*1���C�V�q����A�}�B?��?�U?Dwork�V�q
for i=1:1:15
    for j=1:1:8
        block.Dwork(i).Data(j) = block.Dwork(16).Data(j+(i-1)*8);
        
        if j == 2 && block.Dwork(16).Data(j+(i-1)*8) ~= 0 %?����j=2���o�Oconfidence level
            %?�⦳��??��vision target??�A���u���Ocondidenc level
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
% ???�X�n����?�n?�b??��������??���A�n���M??x_kal_filter?�q?���Q�w? 
%endfunction

function Update(block)

  block.Dwork(16).Data = block.InputPort(1).Data;
  % ���֥[���A�n���M�C��?��initiate��?�N????Dwork(2)�K?�q?�ȡA�q?�Ȧn���O0
  % ???�y�P??�O��tricky���A�����D?���\�p�G�boutput??��?�����֥[�A�n���N�O�[���W�h
%endfunction

function SetInputPortSamplingMode(block, idx, fd)
block.InputPort(idx).SamplingMode = fd;
block.InputPort(idx).SamplingMode = fd;

block.OutputPort(1).SamplingMode = fd;
block.OutputPort(2).SamplingMode = fd;
block.OutputPort(3).SamplingMode = fd;
%endfunction