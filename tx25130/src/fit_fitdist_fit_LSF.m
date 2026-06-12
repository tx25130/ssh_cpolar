%% 地杂波仿真----频谱塑造法
%% 1.使用分布拟合+最小二乘拟合

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

% 添加种子
rng(1);

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


%% ==>新增:回波幅度分布拟合(x)
% fitdist: 拟合分布

%{
%% 1.提取<<原始>>回波幅度

echo = signal_total;
echo = abs(echo(:));

%% 2.使用分布拟合

% 过滤掉幅度为零的采样点(对应无回波或噪声极低的距离门),
% 避免零值干扰分布拟合结果
pd = fitdist(echo(echo~=0),'Weibull'); 
% pd = fitdist(echo,'Rayleigh');

%% 3.绘制分布直方图


figure;
% 100:直方图bin数量
histogram(echo, 100, 'Normalization', 'pdf', 'FaceColor', [0.2 0.4 0.6]);
hold on;

%% 4.绘制拟合曲线

n_bin = linspace(0, max(echo), 1000); %拟合曲线bin数量
y = pdf(pd, n_bin);

plot(n_bin, y, 'r-.', 'LineWidth', 2)
xlabel('幅度');
ylabel('概率密度');
title('原始回波幅度分布');
legend('实测直方图', '分布拟合');
grid on;

%}

%% ==>新增:功率谱拟合(x)
% fittype + fit: 最小二乘拟合

%{
%% 1.参数初始化

% 杂波距离门选择
%   仅分析距离 [Rmin, Rmax] 内的数据，排除远距离噪声基底
Rmin       = 2;    % 杂波起始距离 (m)
Rmax       = 50;   % 杂波截止距离 (m)
gate_start = round(Rmin / Rres) + 1;  % 起始距离门索引（1-based）
gate_end   = round(Rmax / Rres) + 1;  % 截止距离门索引

% FFT 参数
nfft = 256;    % FFT 点数
               %   零填充至 256 > 原始脉冲数 64
               %   效果：频域插值，使谱线更密集，拟合点更多

%% 2.数据预处理

% 安全裁剪距离门索引，防止数组越界
gate_start = max(1, min(gate_start, numSamplePerChirp));
gate_end   = max(gate_start, min(gate_end, numSamplePerChirp));
idx_range  = gate_start : gate_end;

% 频率轴（以零频为中心）
freq_axis  = (-nfft/2 : nfft/2-1) * (1/Tp) / nfft;         % 频率轴 (Hz)
% 构建多普勒速度轴（多普勒频移 → 径向速度）
doppler_axis = freq_axis * lambda / 2;          % 速度轴 (m/s)

%% 3.计算功率谱密度

% 加窗:提高检测弱信号的能力
% win=hamming(numSamplePerChirp);
% win_mat=repmat(win,1,numChirpPerLoop);
% signal_total = signal_total .* win_mat;

% 提取杂波区域回波切片
clutter_data = signal_total(idx_range, :);

% 慢时间维fft
clutter_fft = fftshift(fft(clutter_data, nfft, 2), 2);

psd_V_map = squeeze(mean(clutter_fft, 1));

psd_V_map_pow = abs(psd_V_map).^2;
psd_V_map_db = pow2db(psd_V_map_pow);
psd_V_map_db_norm = psd_V_map_db - max(psd_V_map_db);

% 绘制功率谱
figure;
plot(freq_axis,psd_V_map_db);
xlabel('频率 (Hz)');
ylabel('幅度 (dB)');
title('杂波功率谱密度PSD');


%% 4.非线性最小二乘拟合-高斯模型

% 高斯参数
% 高斯拟合参数初始化
%   初始猜测值影响迭代收敛速度和结果（局部最优 vs 全局最优）
%   对地杂波场景，f0 通常接近 0，sigma 通常在 20~200 Hz
gauss_init_A     = [];    % 峰值功率初始值（空 = 自动取 PSD 最大值）
gauss_init_f0    = 0;     % 中心频率初始值 (Hz)，静止杂波理论为 0
gauss_init_sigma = 100;   % 谱宽初始值 (Hz)

% 拟合区域选择
%   仅对 PSD 主瓣附近的数据拟合，避免噪声基底干扰参数估计
fit_freq_limit = 500;  % 拟合频率范围 ±fit_freq_limit (Hz)
                       %   若杂波谱宽 > 200 Hz，建议增大至 800~1000

% 参数约束边界
% 约束防止拟合结果偏离物理合理范围
PRF = 1/Tp;
bound_A_lo     = 0;       % A 下界：功率不能为负
bound_A_hi     = Inf;     % A 上界：不限制
bound_f0_lo    = -PRF/2;  % f0 下界：不超过奈奎斯特频率
bound_f0_hi    = PRF/2;   % f0 上界：不超过奈奎斯特频率
bound_sigma_lo = 1;       % sigma 下界：谱宽至少 1 Hz
bound_sigma_hi = PRF/2;   % sigma 上界：不超过半 PRF

% 1.定义高斯模型
%   fittype() 创建自定义拟合模型对象
%   模型函数: y = A * exp(-(x - f0)^2 / (2 * sigma^2))
gauss_model = fittype(...
    'A * exp(-(x - f0).^2 / (2 * sigma.^2))', ...
    'independent', 'x', ...
    'dependent', 'y', ...
    'coefficients', {'A', 'f0', 'sigma'});

% 2.设置拟合选项
%   Method: NonlinearLeastSquares（Levenberg-Marquardt 算法）
%   StartPoint: 参数初始猜测值
%   Lower/Upper: 参数约束边界
if isempty(gauss_init_A)
    gauss_init_A = max(psd_V_map_pow);  % 自动取 PSD 最大值作为峰值初始值
end

opts_gauss = fitoptions(...
    'Method',     'NonlinearLeastSquares', ...
    'StartPoint', [gauss_init_A, gauss_init_f0, gauss_init_sigma], ...
    'Lower',      [bound_A_lo, bound_f0_lo, bound_sigma_lo], ...
    'Upper',      [bound_A_hi, bound_f0_hi, bound_sigma_hi], ...
    'Display',    'off');  % 关闭迭代过程的命令行输出

% 选取拟合数据区域
%   仅用主瓣附近的数据拟合，避免噪声基底拉偏参数
fit_mask = abs(freq_axis) <= fit_freq_limit;
if sum(fit_mask) < 4
    % 若主瓣范围内数据点过少，自动回退到更宽范围
    fit_mask = abs(freq_axis) <= PRF / 4;
    fprintf('警告: 拟合区域数据不足，已自动扩大至 ±%.0f Hz\n', PRF/4);
end

x_fit = freq_axis(fit_mask);   % 拟合用频率轴
y_fit = psd_V_map_pow(fit_mask);     % 拟合用 PSD（线性值，非 dB）

% 执行拟合
[fitresult, gof] = fit(x_fit.', y_fit.', gauss_model, opts_gauss);
%   fitresult: 拟合结果对象，包含参数 A / f0 / sigma
%   gof:       拟合优度统计，包含 rsquare / rmse / sse 等

% 生成完整频率轴上的拟合曲线
gauss_fitted     = feval(fitresult, freq_axis);      % 线性值

% 频率域: 原始 PSD 与高斯拟合对比
figure;

plot(freq_axis, psd_V_map_pow, 'b-', 'LineWidth', 1.2);
hold on;
plot(freq_axis, gauss_fitted, 'r-', 'LineWidth', 2.5);
xlabel('多普勒频率 (kHz)');
ylabel('功率谱 (dB)');
title('杂波功率谱密度 — 高斯最小二乘拟合');

%}














