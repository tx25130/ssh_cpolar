



%% 雷达信号处理仿真程序
clc;clear;close all;

%% 参数设置

% 雷达参数
c = 3e8; %光速
f0 = 77e9; %载频 77GHz
lambda = c / f0; %波长

idle_t = 340e-6; %空闲时间
start_t = 6e-6; %发射信号起始位置
end_t = 160e-6; %发射信号结束位置
% Tc = end_t - start_t; %调频时间
Tp = idle_t + end_t; %脉冲重复时间

miu = 24e12; %调频斜率 Hz/s
fs = 4e6; %采样率(Hz)

% 回波参数
numSamplePerChirp = 512;
numChirpPerLoop = 64;
numLoops = 12;
numDevice = 4; %芯片数
numRXPerDevice = 4;
numTXPerDevice = 3;
numFrame = 10; %帧数

Tc = numSamplePerChirp/fs; %调频时间
Be = miu * Tc; %有效带宽
% B = miu * Tp * 10e12; %理论最大带宽

% 关键参数
Rres = c / (2 * Be);
Vres = lambda / (2*numChirpPerLoop*Tp);
Ru = numSamplePerChirp * Rres; %最大不模糊距离
Vu = lambda / (4*Tp);
fprintf('关键参数: ');
fprintf('Rres: %.4fm Vres: %.4fm/s Ru: %.2fm Vu: %.2fm/s\n',...
    Rres,Vres,Ru,Vu);

%% MIMO阵列配置

% 天线参数
numTx_az = 3;
numRx_az = 4;
numVx_az = numTx_az * numRx_az;

% 阵元间距(单位:λ)
d_tx_az = 2;
d_rx_az = 0.5;

% 天线位置
pos_tx_az = (0:numTx_az-1)*d_tx_az;
pos_rx_az = (0:numRx_az-1)*d_rx_az;

% 虚拟阵列
pos_vx_az = zeros(1,numVx_az);

for i=1:numTx_az
    pos_vx_az(1+(i-1)*numRx_az:i*numRx_az)=pos_rx_az+pos_tx_az(i);
end

%% 目标参数

% 目标的  距离(m) 速度(m/s) 加速度(m/s²) 方位角(°) 俯仰角(°) RCS
target = [
          2.54,    1,       0,           1,        5,        1;
          % 2.70,    0,       0,           1,        5,        1;
          3.66,    0,       0,           -22,      5,        1;
          ];

numTarget = size(target, 1); %目标数

% 打印目标参数
paraTarget = input('输入 1 打印目标参数: ');
if paraTarget

    fprintf('%-8s%-12s%-12s%-12s%-12s%-12s%-8s\n', ...
        '序号', '距离(m)', '速度(m/s)', '加速度(m/s²)', ...
        '方位角(°)', '俯仰角(°)', 'RCS');
    for i = 1:numTarget
        fprintf(['%-8d  %-12.2f  %-12.2f  %-12.2f   %-12.2f' ...
            '   %-12.2f  %-8.2f\n'], i, target(i,1), target(i,2),...
            target(i,3), target(i,4), target(i,5), target(i,6));
    end

end

%% 信号生成
% x = s+n+c+j
% tx_signal = exp(1j*2*pi*(f0*t+1/2*miu*t.^2));
% rx_signal = A * exp(1j*2*pi*(f0*(t-tau)+1/2*miu*(t-tau).^2));
% 混频: f_IF = tx.*conj(rx)
% phi_IF = 2*pi*miu*tau*t + 2*pi*(f0*tau-0.5*miu*tau.^2)
%          | 线性项(fb和fd) | | 常数项(只影响初相位) |
% 添加噪声: awgn(中频信号,信噪比)

% 时间向量
t = (0:numSamplePerChirp-1)*Tc/numSamplePerChirp;

%% 1.发射信号

% Tx(t) = exp(j*2π*(f0*t + 0.5*miu*t^2))
txSignal = exp(1j * 2 * pi * (f0 * t + 0.5 * miu * t.^2));

%% 2.接收信号

% 初始化矩阵
rxSignal = zeros(numSamplePerChirp, numChirpPerLoop, numVx_az);

% 生成回波信号
for i = 1:numTarget

    % 目标参数
    target_R = target(i,1);
    target_v = target(i,2);
    target_az = target(i,4);
    RCS = target(i,6);

    % 时延和多普勒
    tau = 2*target_R/c;
    fd = 2*target_v/lambda;


    for m = 1:numVx_az

        % 空间相位
        phase_az = 2*pi*pos_vx_az(m)*sind(target_az);

        for n =1:numChirpPerLoop

            % 慢时间相位(多普勒)
            phase_v = -2*pi*fd*(n-1)*Tp;

            % 中频信号拍频
            fb = miu*tau;
            % 快时间相位
            phase_IF = -2*pi*fb*t;

            % 回波幅度
            % amp = sqrt(RCS)/target_R.^2;
            amp = 1;


            % 接收信号(注: 快时间相位与慢时间相位一致)
            rxSignal(:,n,m) = rxSignal(:,n,m) + (amp*txSignal.*...
                exp(1j*(phase_IF+phase_v+phase_az))).';

        end

    end

end

%% 3.混频

% 初始化矩阵
if_signal = zeros(numSamplePerChirp, numChirpPerLoop, numVx_az);

for m = 1:numVx_az
    for n = 1:numChirpPerLoop
        if_signal(:,n,m)=txSignal.*conj(squeeze(rxSignal(:,n,m)).');
    end
end

%% 4.添加噪声

% 距离*速度*角度
SNR = 20;
%awgn:给信号增加SNR的信噪比,从而生成噪声
if_signal=awgn(if_signal,SNR,"measured"); 


%% 加窗

% 提高检测弱信号的能力
% win=hamming(numSamplePerChirp);
% win_mat=repmat(win,1,numChirpPerLoop,numVx_az);
% if_signal=if_signal.*win_mat;



% 对于距离fft,速度fft,DBF等,
% 提高的信噪比为pow2db(N),提高的信号增益为mag2db(N),N为采样点数
% 可以通过幅度谱验证
%% 距离处理

range_fft = fft(if_signal,[],1);
range_fft_mag=abs(range_fft);

% 坐标轴
range_axis=(0:numSamplePerChirp-1)*Rres;

% 绘制距离谱
figure;
range_map=squeeze(mean(mean(range_fft_mag,2),3));
plot(range_axis,mag2db(range_map));
xlabel('距离(m)');
ylabel('幅度(dB)');
title('距离FFT结果');
grid on;
hold on;


%% CFAR-1d

% 参数
Pfa=10e-5;%虚警概率(噪声检测概率)
RCr=4;%参考单元
GCr=2;%保护单元

%================CFAR-1d================
%
%       RC + GC + CUT1 + GC + RC
%
%=======================================

num_AC=2*RCr+2*GCr+1;%总的单元数
num_GC=2*GCr+1;%保护单元数(包含CUT)
num_RC=num_AC-num_GC;%参考单元数 或者为num_RC=2*RCr


% 计算阈值因子(幅度)    Y=常数K*估计值Z   Z=参考单元数量
Y = (Pfa^(-1/num_RC) - 1)*num_RC;     %Pfa =1 / ((1+K)^M)  M=num_RC

% 初始化矩阵
num_CFAR_map=numSamplePerChirp-(2*RCr+2*GCr);
CFAR_threshold = zeros(num_CFAR_map,1);

% 遍历距离平面
for i=1+RCr+GCr:numSamplePerChirp-RCr-GCr

    % 1.构建参考单元区域(包含保护单元)
    RC_left=range_map(i-RCr-GCr:i-GCr);
    RC_right=range_map(i+GCr:i+GCr+RCr);
    RC=range_map(i-RCr-GCr:i+GCr+RCr);

    % 2.去除保护单元(保护单元设为0)
    RC(RCr+1:1+RCr+GCr+GCr) = 0;

    % 3.求平均功率
    X=sum(RC(:))/num_RC;

    % 4.计算检测阈值(判别门限)
    threshold = Y * X;
    CFAR_threshold(i-RCr-GCr) =threshold;%将阈值序号从1开始排列

end

CFAR_axis=(1+RCr+GCr:numSamplePerChirp-RCr-GCr)*Rres;
plot(CFAR_axis,mag2db(CFAR_threshold));

%% 速度处理
% MTD = MTI+速度fft+CFAR

% 速度fft(+fftshift)
doppler_fft=fftshift(fft(if_signal,[],2),2);
doppler_fft_mag = abs(doppler_fft);

% 坐标轴
doppler_axis=(-numChirpPerLoop/2:numChirpPerLoop/2-1)*Vres;

% 绘制速度谱
figure;
doppler_map=squeeze(mean(mean(doppler_fft_mag,1),3));
plot(doppler_axis, mag2db(doppler_map));
xlabel('速度 (m/s)');
ylabel('幅度 (dB)');
title('多普勒FFT结果');

%% RD图绘制

% 距离fft
range_fft=fft(if_signal,[],1);

% 速度fft(+fftshift)
rd_fft=fftshift(fft(range_fft, [], 2), 2);
rd_fft_mag=abs(rd_fft);

% 提取rd矩阵
rd_matrix=mean(rd_fft_mag,3);

% 坐标轴
range_axis=(0:numSamplePerChirp-1)*Rres;
doppler_axis=(-numChirpPerLoop/2:numChirpPerLoop/2-1)*Vres;

% 生成网格(注意维度对应) %X:二维网格坐标 x: 坐标轴
[V_grid, R_grid] = meshgrid(doppler_axis, range_axis);

% 绘制RD图
figure;
rd_map=rd_matrix;
surf(R_grid, V_grid, mag2db(rd_map), 'EdgeColor', 'none');
xlabel('距离 (m)');
ylabel('速度 (m/s)');
zlabel('幅度 (dB)');
title('距离-多普勒图');
colorbar;

%% 1.对RD图做MTI

% 距离fft
range_fft=fft(if_signal,[],1);

% 初始化矩阵
mti=zeros(numSamplePerChirp,numChirpPerLoop,numVx_az);

for i=1:numChirpPerLoop-2
    mti(:,i,:)=range_fft(:,i+2,:)-2*range_fft(:,i+1,:)+range_fft(:,i,:);
end

% mti=squeeze(abs(mean(mti(),3)));
%
% figure;
% surf(abs(mti), 'EdgeColor', 'none');

% 速度fft(+fftshift)
rd_fft=fftshift(fft(mti, [], 2), 2);
rd_fft_mag=abs(rd_fft);

% 提取rd矩阵
rd_matrix=mean(rd_fft_mag,3);

% 坐标轴
range_axis=(0:numSamplePerChirp-1)*Rres;
doppler_axis=(-numChirpPerLoop/2:numChirpPerLoop/2-1)*Vres;

% 生成网格(注意维度对应) %X:二维网格坐标 x: 坐标轴
[V_grid, R_grid] = meshgrid(doppler_axis, range_axis);

% MTI后的RD图
figure;
rd_map_mti=rd_matrix;
surf(R_grid, V_grid, mag2db(rd_map_mti), 'EdgeColor', 'none');
xlabel('距离 (m)');
ylabel('速度 (m/s)');
zlabel('幅度 (dB)');
title('距离-多普勒图');
colorbar;

%% CFAR-2d

% 交换维度==>numChirpPerLoop*numSamplePerChirp %坐标轴的x,y 分别对应矩阵的列,行
dr_map=permute(rd_map,[2,1]); %permute(A,[dim-order])调整维度顺序

% 参数
Pfa = 10e-6; % 虚警概率
RCr = 4;     % 距离维参考单元(单边)
RCd = 2;     % 多普勒维参考单元(单边)
GCr = 2;     % 距离维保护单元(单边)
GCd = 1;     % 多普勒维保护单元(单边)

%================CFAR-2d================
%
%                   速度维d
%          RC + RC +  RC  + RC + RC
%          RC + GC +  GC  + GC + RC
% 距离维r  RC + GC + CUT1 + GC + RC
%          RC + GC +  GC  + GC + RC
%          RC + RC +  RC  + RC + RC
%
%=======================================

num_AC=(2*RCr+2*GCr+1)*(2*RCd+2*GCd+1);%总的单元数
num_GC=(2*GCr+1)*(2*GCd+1);%保护单元数(包含CUT)
num_RC=num_AC-num_GC;%参考单元数

% 计算阈值因子(幅度)  Y=常数K*Z  Z=参考单元数量
Y = (Pfa^(-1/num_RC) - 1)*num_RC; %Pfa =1 / ((1+K)^M)   M=num_RC

% 初始化矩阵
CFAR_result = zeros(size(dr_map));
CFAR_threshold = zeros(size(dr_map));

% 遍历距离-多普勒平面(避开边缘区域)
for r = (1+RCr+GCr):(numSamplePerChirp-RCr-GCr)
    for d = (1+RCd+GCd):(numChirpPerLoop-RCd-GCd)

        % 1.构建参考单元区域(包含保护单元)
        RC = abs(dr_map(d-RCd-GCd:d+RCd+GCd, r-RCr-GCr:r+RCr+GCr));

        % 2.去除保护单元(保护单元设为0)
        RC(1+RCd:RCd+GCd+1+GCd, RCr+1:RCr+GCr+1+GCr) = 0;

        % 3.求平均功率
        X = sum(RC(:)) / (num_RC);

        % 4.计算检测阈值(判别门限)
        threshold = Y * X; %threshold门限
        CFAR_threshold(d-RCd-GCd, r-RCr-GCr) = threshold;


        %=========目标检测=========
        % 1.提取当前CUT值
        CUT = abs(dr_map(d, r));
        % 2.检测单元＞门限
        if CUT > threshold
            CFAR_result(d, r) = 1;
        end
        %==========================

    end
end


% 标记CFAR目标
[CFAR_d, CFAR_r] = find(CFAR_result == 1);

%求目标的所在位置
maxima=imregionalmax(abs(dr_map)); %imregionalmax:图像区域最大值
valid=maxima&CFAR_result;%求出目标的位置

% 标记峰值目标
[max_d,max_r]=find(valid);

% 二维CFAR结果
figure;
subplot(2,1,1);
imagesc(range_axis, doppler_axis, CFAR_result); %坐标轴的x,y 分别对应 矩阵的列,行
xlabel('距离 (m)'); ylabel('速度 (m/s)');
title('二维CFAR检测结果(1=目标)');
colorbar;
hold on;

plot(range_axis(CFAR_r), doppler_axis(CFAR_d), 'ro'); %圈选标记

subplot(2,1,2);
imagesc(range_axis, doppler_axis, abs(valid)); %imagesc(距离轴,速度轴,z)显示二维R-D图
xlabel('距离 (m)'); ylabel('速度 (m/s)');
title('R-D图峰值点(测试)');
colorbar;
hold on;

plot(range_axis(max_r), doppler_axis(max_d), 'ro'); %圈选标记

hold off;

%% 相位补偿 (补偿速度)

% 角度*速度*距离
if_signal=permute(if_signal,[3,2,1]);

estimated__doppler=(-1)*doppler_axis(max_d);%速度补偿


% 绘制角度谱
figure;

for i=1:numTarget


    range_fft=fft(if_signal,[],3);
    %提取距离门
    doppler_matrix=squeeze(range_fft(:,:,max_r(i)));

    % 多普勒fft(+fftshift)
    doppler_fft = fftshift(fft(doppler_matrix, [], 2), 2); %fft(X,num,dim) fftshift(X,dim)

    % 初始化矩阵
    compensated_doppler = zeros(size(doppler_fft));

    for m=1:numVx_az
        for n=1:numChirpPerLoop

            %相位补偿
            phase_compensated=exp(1j*2*pi *(n-1)* 2*estimated__doppler(i)*Tp/lambda);
            compensated_doppler(m,n) = doppler_fft(m,n) * phase_compensated;
        end
    end

    %% 角度估计 (DBF)
    % 提取目标距离-多普勒单元数据
    angle_data = squeeze(compensated_doppler(:,max_d(i)));

    % 角度搜索范围
    angle_range = -90:0.5:90;

    % 初始化矩阵
    angle_spectrum = zeros(size(angle_range));

    % DBF波束形成
    for k = 1:length(angle_range)

        angle = deg2rad(angle_range(k));

        % 计算波程差
        delta_d=pos_vx_az*sin(angle);
        steering_vector = exp(1j*2*pi*delta_d);

        % 计算波束形成输出
        angle_spectrum(k) = abs(steering_vector * angle_data)^2;

    end

    % 归一化角度谱
    angle_spectrum_db = mag2db(angle_spectrum/max(angle_spectrum));

    % 绘制角度谱
    plot(angle_range, angle_spectrum_db);
    xlabel('角度 (度)');
    ylabel('归一化幅度 (dB)');
    title('DBF角度谱(方位角)');
    grid on;
    hold on;

    % 目标仿真信息
    [M,I]=max(angle_spectrum_db);
    fprintf('目标%d仿真距离: %.2f 速度: %.2f 方位角: %.2f\n', ...
        i,range_axis(max_r(i)),doppler_axis(max_d(i)),angle_range(I));

end

















