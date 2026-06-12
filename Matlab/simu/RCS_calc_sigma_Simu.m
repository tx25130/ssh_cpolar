%% AWR2243雷达街景实验回波数据处理程序
% 地物回波采集
% 方位角范围为（-60°~60°）、分辨率为1.4°，
% 俯仰角范围为（-30°~30°）、分辨率为18°

clc;clear;close all
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
% numVx_az = numTx_az * numRx_az;
numVx_az = 12;


% 阵元间距(单位:λ)
d_tx_az = 2;
d_rx_az = 0.5;

% 天线位置
pos_tx_az = (0:numTx_az-1)*d_tx_az;
pos_rx_az = (0:numRx_az-1)*d_rx_az;

% 虚拟阵列
% pos_vx_az = zeros(1,numVx_az);

% for i=1:numTx_az
%     pos_vx_az(1+(i-1)*numRx_az:i*numRx_az)=pos_rx_az+pos_tx_az(i);
% end

pos_vx_az = (0:numVx_az-1)*d_rx_az;

%% 目标参数

% 目标的  距离(m) 速度(m/s) 加速度(m/s²) 方位角(°) 俯仰角(°) RCS
target = [
    % 2.54,    1,       0,           1,        5,        1;
    % 3.66,    1.5,     0,           -22,      5,        1;
    % 1.25,      0.55,    0,           0,        0,        1;
    1.25,      0,       0,           0,        0,        1;
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

%% 定标体(dB)
% Pr = (Pt * G^2 * λ^2 * σ) / ((4π)^3 * R^4)
% Pr = K * σ / (R^4)

rcs1 = pow2db(pi*0.05^2);% 定标球RCS

R = 3.125;
Pr = 79.3984326514203;
Pr = Pr - mag2db(512); %实际功率(减去了距离fft带来的增益)
%+3 矫正偏差
PR4_1 = Pr + pow2db(R^4);
PR4_az_dingbiao = 155.37;

K = PR4_1 - rcs1; %定标系数

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
            RCS = db2pow(-10)*target_R*2/numVx_az*Rres/cosd(59.9);
            amp = sqrt(db2pow(K)*RCS/target_R.^4);
            %% 2.1测试(裸土HH_1)
            
            % A = pow2db(R0.*(2/(N-1)) .* Rres .* (1./cosd(angle)));
            A = pow2db(1.25.*(2/(12-1)) .* 0.0488 .* (1./cosd(59.7)));


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

%% 相位补偿 (补偿速度)

% 角度*速度*距离
if_signal=permute(if_signal,[3,2,1]);


target_R = target(1:numTarget,1);
target_v = target(1:numTarget,2);
max_r = round(target_R / Rres)+1;% ceil向上取值
max_d = round((target_v+Vu) / Vres)+1;

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
    theta_res = rad2deg(2/(numVx_az-1));
    angle_range = -60:theta_res:60;

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
    % angle_spectrum_db = mag2db(angle_spectrum/max(angle_spectrum));
    angle_spectrum_db = mag2db(angle_spectrum);


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

%% 仿真测试
Pr = mag2db(range_map(max_r));

PR4 = Pr' + pow2db(range_axis(max_r).^4);

az_angle = 0;% 选取角度
max_angle = ceil((az_angle+60)/theta_res) + 1;
az_angle_1 = angle_spectrum_db(max_angle);
Pr_az = angle_spectrum_db(max_angle)/2;

PR4_az = Pr_az + pow2db(range_axis(max_r).^4);

fprintf('Pr: %.2f PR4: %.2f PR4_az: %.2f\n',Pr,PR4,PR4_az);

% 后向散射系数计算
rcs2 = PR4_az + rcs1 - PR4_az_dingbiao;
sigma0 = rcs2 - A;

fprintf('后向散射系数: %.4f\n',sigma0);

