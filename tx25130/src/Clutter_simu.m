%% 地杂波仿真----频谱塑造法
% 参考网址:
% https://blog.csdn.net/moonquakeTT/article/details/155278155

%% 参数设置
clc;clear;close all

% 雷达参数
c       = 3e8;         % 光速 (m/s)
f0      = 77e9;        % 载频 (Hz)，77GHz毫米波雷达
lambda  = c / f0;      % 波长 (m)

% FMCW波形参数
idle_t  = 340e-6;      % 空闲时间 (s)
end_t   = 160e-6;      % 发射信号结束位置 (s)
Tp      = idle_t + end_t;  % 脉冲重复时间 (PRI) (s)
miu     = 24e12;       % 调频斜率 (Hz/s)

% 采样参数
fs                  = 4e6;               % ADC采样率 (Hz)
numSamplePerChirp   = 512;               % 每个Chirp的采样点数
numChirpPerLoop     = 64;                % 每帧Chirp数（慢时间脉冲数）
numRangeGates       = numSamplePerChirp; % 距离门数 = 采样点数
numPulses           = numChirpPerLoop;   % 脉冲数（与Chirp数对应）


% 关键分辨参数 
Tc   = numSamplePerChirp / fs;   % 调频时间 (s)
Be   = miu * Tc;                 % 有效带宽 (Hz)
Rres = c / (2 * Be);             % 距离分辨率 (m)
Vres = lambda / (2 * numPulses * Tp);  % 速度分辨率 (m/s)
Ru   = numSamplePerChirp * Rres;      % 最大不模糊距离 (m)
PRF  = 1 / Tp;                        % 脉冲重复频率 (Hz)

%% MIMO阵列配置

numTx   = 3;    % 发射天线数
numRx   = 4;    % 接收天线数
numVx   = numTx * numRx;  % 虚拟阵列元素数


%% 地杂波环境参数

% 杂波参数
fdc     = 0;           % 杂波中心多普勒频率 (Hz)
                        %   静止雷达对地杂波，中心频率为0
sigma_c = 50;          % 杂波谱宽 (Hz)
                        %   由风速、天线波束宽度等因素决定
                        %   典型值 20~200 Hz
% 杂波功率
Pc      = 10;          % 每个距离门的杂波功率 (线性值)
                        %   可通过CNR（杂噪比）换算
CNR_dB  = 30;          % 杂噪比 (dB)

% 杂波距离分布
Rmin        = 2;       % 杂波起始距离 (m)
Rmax        = min(Ru, 50);  % 杂波截止距离 (m)，限制在可观测范围内

% 使用last从后向前查找,确定有效杂波距离门数量
% index = find(condition, n, direction)
% condition:逻辑条件 n:返回索引数量 direction:first/last
clutter_idx = find((Rmin:Rres:Ru) <= Rmax, 1, 'last');

numClutterGates = max(clutter_idx, 1);  % 有效杂波距离门数


%% 频谱塑型

%% 1.确定PSD形状

% 多普勒频率轴（以零频为中心）
freq_axis = (-numPulses/2 : numPulses/2 - 1) * (1/Tp) /numPulses;

% 高斯型功率谱密度形状
%   PSD(f) = exp(-(f - fdc)^2 / (2 * sigma_c^2))
PSD_shape = exp(-(freq_axis - fdc).^2 / (2 * sigma_c^2));

%% 2.生成 2D 复高斯白噪声(时域)

% 实部虚部独立同分布 ~ N(0, 1/sqrt(2))，除以sqrt(2)保证单位功率
noise_2d = (randn(numRangeGates, numPulses) + ...
            1i * randn(numRangeGates, numPulses)) / sqrt(2);

% 将时域白噪声转换到频域
% 由于复高斯白噪声在时域和频域统计特性不变,可以直接加在频域
% <<此步可以省略>>
% fft(..., [], 2) 对第2维（慢时间/脉冲维）做 FFT
noise_2d = fft(noise_2d, [], 2);

%% 3.慢时间维频谱塑形

% 将 sqrt(PSD) 广播为 [N_range × N_pulse] 矩阵
% 快时间维（第1维）不乘任何因子，保持白色
clutter_freq = noise_2d .* repmat(sqrt(PSD_shape),...
                                         numRangeGates, 1);

%% 4.慢时间维 IFFT → 时域

clutter_2d = ifft(ifftshift(clutter_freq, 2), [], 2);

%% 5.频率归一化

% 统一缩放至 Pc（基准功率），后续由 R_atten 逐门衰减
current_power = mean(abs(clutter_2d(:)).^2);
clutter_2d = clutter_2d * sqrt(Pc / current_power);

% 添加距离衰减 R^4, 逐距离门乘以实数衰减因子
% 实数缩放不改变复高斯性质 → 包络仍为瑞利分布
% R^4 距离衰减因子:
range_axis = (0:numRangeGates-1) * Rres;  % 距离轴 (m)
R_atten = zeros(numRangeGates, 1);         % 距离衰减因子（列向量）

for i = 1:numRangeGates
    R_i = range_axis(i);
    if R_i >= Rmin && R_i <= Rmax
        % R^4 雷达方程衰减，归一化到 Rmin（最近距离门衰减为1）
        R_atten(i) = (Rmin / max(R_i, Rmin))^4;
    end
end

Clutter = clutter_2d .* R_atten;


%% 添加噪声

% 根据CNR计算噪声功率
Pn = Pc / (10^(CNR_dB / 10));  % 噪声功率（线性值）
noise_power = sqrt(Pn / 2);     % 噪声标准差（分配到实部和虚部）

% 生成复高斯白噪声矩阵
noise = noise_power * (randn(numRangeGates, numPulses) + ...
                       1i * randn(numRangeGates, numPulses));

% 合成总信号：杂波 + 噪声
signal_total = Clutter + noise;

%% RD图绘制

% 距离FFT
range_fft = fft(signal_total, [], 1);
range_fft_mag = abs(range_fft);

% 速度FFT（多普勒处理）
rd_fft = fftshift(fft(range_fft, [], 2), 2);
rd_fft_mag = abs(rd_fft);

% 距离轴和多普勒轴
range_axis  = (0:numRangeGates-1) * Rres;
doppler_axis = (-numPulses/2 : numPulses/2-1) * Vres;

% 绘制RD图
figure;
[V_grid, R_grid] = meshgrid(doppler_axis, range_axis);
rd_clutter = mag2db(rd_fft_mag);
surf(R_grid, V_grid, rd_clutter, 'EdgeColor', 'none');
xlabel('距离 (m)');
ylabel('速度 (m/s)');
zlabel('幅度 (dB)');
title('距离-多普勒三维图');
colorbar;
colormap jet; %设置颜色映射
view(-45, 30); %设置视图角度:方位角-45度,俯仰角30度







