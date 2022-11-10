% ����MPC���ٹ켣
% ���ߣ�zhaokun
% ���ڣ�2022/04/29
clc
clear
close all
load path.mat

%% ��ʼ����
dt = 0.1;   % ʱ�䲽��
L = 2.9;    % ���
max_steer =60 * pi/180; % in rad
target_v =30.0 / 3.6;

%% �ο��켣����ز���
% ����ο��켣
refPos = path;
refPos_x = refPos(:,1);
refPos_y = refPos(:,2);

% ���㺽��Ǻ�����
diff_x = diff(refPos_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y);
diff_y(end+1) = diff_y(end);
derivative1 = gradient(refPos_y) ./ abs(diff_x);              % һ�׵���
derivative2 = del2(refPos_y) ./ abs(diff_x);                  % ���׵���
refHeading = atan2(diff_y , diff_x);                   % �����
refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % �������ʣ��ο�����

% ���ݰ�����ת��ԭ������ο�ǰ��ת��
refDelta = atan(L*refK);%�ο�ǰ��ת��

% ��ͼ
figure
plot(refPos_x,refPos_y,'r-')
hold on

%% ������
x = refPos_x(1)+0.5; 
y = refPos_y(1) + 0.5; 
yaw = refHeading(1)+0.02; 
v = 0.1;
U = [0.01;0.01];%��ʼ������
idx =0;
pos_actual = [refPos_x,refPos_y];
latError_MPC = [];
speedError_MPC=[];
deltaError_MPC=[];

% ѭ��
while idx < length(refPos_x)-1
    
    % ����MPC��������zhaokun ȱ�ٲο��ٶȲ���
    [Delta,v,idx,latError,U] = mpc_control(x,y,yaw,refPos_x,refPos_y,refHeading,refK,refDelta,dt,L,U,target_v) ;
    
    % ���̫���˳�����
    if abs(latError) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % �����˶�ѧ����,����״̬��
    [x,y,yaw] = updateState(x,y,yaw,v , Delta, dt,L, max_steer); 
    
    % ����ÿһ����ʵ����
    pos_actual(end+1,:) = [x,y];
    latError_MPC(end+1,:) = [idx,latError];
    speedError_MPC(end+1,:)=[idx,U(1)];
    deltaError_MPC(end+1,:)=[idx,U(2)];
    % �����ٹ켣ͼ
    scatter(x,y,150,'b.')
    pause(0.01);
end

figure(2);
%��������ڲο��ߵĺ������
title('�������')
plot(latError_MPC(:,1),latError_MPC(:,2),'r');

figure(3)%�ٶ����
title('�ٶ����')

plot(speedError_MPC(:,1),speedError_MPC(:,2),'r');

figure(4)%ǰ��ת�����
title('ǰ��ת�����')
plot(deltaError_MPC(:,1),deltaError_MPC(:,2),'b');


%% ����
path_MPC = pos_actual;
save path_MPC.mat path_MPC
save latError_MPC.mat latError_MPC