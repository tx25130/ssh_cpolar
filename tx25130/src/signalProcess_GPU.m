%% GPU加速雷达信号处理演示脚本
% 本脚本演示FMCW（调频连续波）雷达信号处理的GPU加速流程
% 功能: 从原始ADC数据中提取距离-多普勒信息
% 处理流程: 数据上传 → Chirp生成 → 混频 → 加窗 → 距离FFT → 多普勒FFT → 结果下载
%
% 注意: 需要NVIDIA GPU和Parallel Computing Toolbox支持

% 清理环境
clear;      % 清除工作区变量
close all;  % 关闭所有图形窗口
clc        % 清空命令行

% 检查GPU可用性
if ~gpuDeviceCount
    error('未检测到GPU设备，请确保已安装NVIDIA显卡和CUDA驱动');
end
g = gpuDevice;
fprintf('检测到GPU: %s\n', g.Name);
fprintf('显存: %.2f GB\n', g.AvailableMemory / 1e9);

%% 模拟雷达参数配置
% 在实际应用中，这些参数从雷达配置文件读取
cfg = struct();
cfg.sampleRate   = 1e6;      % 采样率: 1 MHz
cfg.pulseWidth   = 50e-6;    % Chirp脉宽: 50 us
cfg.slope        = 5e12;     % 调频斜率: 5 MHz/us (5e12 Hz/s)
cfg.centerFreq   = 77e9;     % 载波频率: 77 GHz (车载毫米波雷达)
cfg.numSamples   = 512;      % 每Chirp采样点数
cfg.numChannels  = 4;        % 接收通道数 (MIMO配置)
cfg.numFrames    = 128;      % 帧数 (用于多普勒处理)

% 生成模拟ADC数据（用于演示）
% 在实际应用中，adcData由雷达硬件采集得到
numSamples = cfg.numSamples;
numChannels = cfg.numChannels;
numFrames = cfg.numFrames;

% 生成包含目标信号的模拟数据
adcData = zeros(numSamples, numChannels, numFrames);
for frame = 1:numFrames
    for ch = 1:numChannels
        % 添加两个模拟目标（不同距离、速度）
        t = (0:numSamples-1)' / cfg.sampleRate;
        
        % 目标1: 距离10m, 速度5m/s
        beatFreq1 = 2 * cfg.slope * (10 / 3e8);  % 拍频 = 2*slope*R/c
        phaseShift1 = 2 * pi * 2 * cfg.centerFreq * 5 * (frame-1) * cfg.pulseWidth / 3e8;
        signal1 = 0.5 * sin(2*pi*beatFreq1*t + phaseShift1);
        
        % 目标2: 距离30m, 速度-3m/s
        beatFreq2 = 2 * cfg.slope * (30 / 3e8);
        phaseShift2 = 2 * pi * 2 * cfg.centerFreq * (-3) * (frame-1) * cfg.pulseWidth / 3e8;
        signal2 = 0.3 * sin(2*pi*beatFreq2*t + phaseShift2);
        
        % 添加高斯白噪声
        noise = 0.05 * randn(numSamples, 1);
        
        adcData(:, ch, frame) = signal1 + signal2 + noise;
    end
end

fprintf('模拟ADC数据生成完成: [%d x %d x %d]\n', numSamples, numChannels, numFrames);

%% GPU信号处理流程开始

%% 1. GPU数据传输 - 将ADC数据从CPU内存移至GPU显存
% gpuArray() 自动在GPU上创建数组，后续运算自动在GPU执行
fprintf('正在上传数据到GPU...\n');
gpuADC = gpuArray(single(adcData));  % 转换为single类型以减少显存占用
% 获取数据维度信息
[numSamples, numChannels, numFrames] = size(gpuADC);

%% 2. 生成参考Chirp信号（去斜/Dechirp处理用）
% FMCW雷达通过将回波与发射信号混频，提取拍频（beat frequency）
% 拍频频率与目标距离成正比

% 创建时间向量（列向量）
t = gpuArray.linspace(0, cfg.pulseWidth, numSamples)';  % [numSamples x 1]

% 生成参考Chirp信号: s(t) = exp(1j * 2 * pi * (fc * t + 0.5 * slope * t^2))
% 这里使用简化模型生成复数Chirp信号
refChirp = exp(1j * 2 * pi * (cfg.centerFreq * t + 0.5 * cfg.slope * t.^2));
refChirp = refChirp / max(abs(refChirp));  % 归一化

% 扩展维度以匹配ADC数据 [numSamples x 1 x 1]
refChirp = reshape(refChirp, numSamples, 1, 1);
fprintf('参考Chirp信号生成完成\n');

%% 3. 混频处理 (Dechirp / 去斜处理)
% 原理: 将接收信号与参考Chirp共轭相乘，提取拍频分量
% 数学: mixed = adcData .* conj(refChirp)
% 结果包含目标距离信息的低频拍频信号

fprintf('正在进行混频处理...\n');
mixedSignal = gpuADC .* conj(refChirp);  % 逐元素乘法，GPU自动并行

%% 4. 加窗处理 - 抑制频谱泄漏
% 使用Hann窗减少FFT旁瓣，提高动态范围
% 窗函数作用: 平滑信号边缘，减少吉布斯现象

fprintf('正在应用窗函数...\n');
% 生成Hann窗并移至GPU
window = gpuArray(hann(numSamples, 'periodic'));
% 扩展窗函数维度以匹配信号 [numSamples x 1 x 1]
window = reshape(window, numSamples, 1, 1);
% 应用窗函数
windowedSignal = mixedSignal .* window;

%% 5. 距离FFT - 第一次FFT，提取距离信息
% 原理: 对加窗后的混频信号做FFT，拍频峰值位置对应目标距离
% R = (f_beat * c) / (2 * slope)
% 输出: rangeProfile [numSamples x numChannels x numFrames]

fprintf('正在进行距离FFT...\n');
% 沿第1维（采样点维）执行FFT
rangeProfile = fft(windowedSignal, [], 1);
% 转换为单边谱并取幅度（可选）
% rangeProfile = abs(rangeProfile(1:numSamples/2, :, :));

%% 6. 多普勒处理 - 第二次FFT，提取速度信息
% 原理: 对不同帧（慢时间）的同一距离单元做FFT，提取多普勒频移
% 速度: v = (f_doppler * lambda) / 2
% fftshift将零频移到中心，正负频率对应接近/远离目标

fprintf('正在进行多普勒FFT...\n');
% 沿第3维（帧维）执行FFT，并用fftshift将零频移到中心
velocityProfile = fftshift(fft(rangeProfile, [], 3), 3);

%% 7. 结果回传CPU
% gather() 将GPU数组传回CPU内存
fprintf('正在下载结果到CPU...\n');
rangeProfile = gather(rangeProfile);
velocityProfile = gather(velocityProfile);

fprintf('GPU处理完成!\n');


%% 距离像显示（取第一个通道、第一帧）
figure('Name', '距离像');
rangeAxis = (0:numSamples-1) * cfg.sampleRate / numSamples;  % 距离轴（需根据实际参数校准）
plot(rangeAxis/1e3, abs(rangeProfile(:, 1, 1)));
xlabel('频率 (kHz)');
ylabel('幅度');
title('距离像 (Range Profile) - Channel 1, Frame 1');
grid on;

%% 距离-多普勒图显示（取第一个通道、所有帧平均）
figure('Name', '距离-多普勒图');
rangeDoppler = mean(abs(velocityProfile(:, 1, :)), 3);  % 沿帧维平均
imagesc(1:numFrames, rangeAxis/1e3, 20*log10(abs(rangeDoppler) + eps));
xlabel('多普勒单元');
ylabel('频率 (kHz)');
title('距离-多普勒图 (Range-Doppler Map) - Channel 1');
colorbar;
colormap('jet');
axis xy;

%% 性能统计
fprintf('\n========== 处理结果统计 ==========\n');
fprintf('输入数据尺寸: [%d x %d x %d]\n', numSamples, numChannels, numFrames);
fprintf('距离像尺寸:   [%d x %d x %d]\n', size(rangeProfile,1), size(rangeProfile,2), size(rangeProfile,3));
fprintf('速度像尺寸:   [%d x %d x %d]\n', size(velocityProfile,1), size(velocityProfile,2), size(velocityProfile,3));
fprintf('==================================\n');

%% 清理GPU内存
reset(g);
fprintf('GPU内存已释放\n');
