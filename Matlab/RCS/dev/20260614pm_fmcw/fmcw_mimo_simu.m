%% =========================================================================
%  FMCW MIMO 雷达仿真 — 完整单脚本版本
%  =========================================================================
%  基于 TI AWR2243 4芯片级联 TDM-MIMO 体制
%  支持 方位角 + 俯仰角 二维角度估计
%  俯仰虚拟阵列: 借用方位TX1 + 3俯仰TX → 4元阵 [0, 0.5, 2, 3] λ
%  =========================================================================
%  整合自以下模块（按处理顺序）：
%    fmcw_sim_config.m      → 步骤 1: 参数配置
%    fmcw_sim_signal.m      → 步骤 2: TDM-MIMO 差频信号生成
%    fmcw_sim_clutter.m     → 步骤 3: 杂波叠加
%    fmcw_sim_noise.m       → 步骤 4: 噪声叠加
%    fmcw_sim_range_fft.m   → 步骤 5: Range-FFT 测距
%    fmcw_sim_mti.m         → 步骤 6: MTI 两脉冲对消
%    fmcw_sim_doppler_fft.m → 步骤 7: Doppler-FFT 测速
%    fmcw_sim_virtual_array.m → 步骤 8: MIMO虚拟阵列重组(方位+俯仰)
%    fmcw_sim_cfar.m        → 步骤 9: 二维 CA-CFAR 检测
%    fmcw_sim_angle.m       → 步骤 10: 角度估计(方位+俯仰)
%    fmcw_sim_pointcloud.m  → 步骤 11: 点云可视化
%    fmcw_sim_verify.m      → 步骤 12: 仿真验证
%  =========================================================================

clear; clc; close all;

%% ========================================================================
%  目标定义（用户可修改）
%  格式: [R(m), v(m/s), θ_az(°), θ_el(°), RCS(m²)]
%  或:   [R(m), v(m/s), θ_az(°), RCS(m²)]（无俯仰角时，El默认0°）
%% ========================================================================
targets = [5.0, 0, 10.0, 5.0, 10;   % 目标1: R=5m, v=0, Az=10°, El=5°, RCS=10m²
        %    10.0, 0, -15.0, 0.0, 20]; % 目标2: R=10m, v=0, Az=-15°, El=0°, RCS=20m²
           5.0, 0, 50.0, 25.0, 10]; % 目标2: R=10m, v=0, Az=-15°, El=0°, RCS=20m²

%% ========================================================================
%  [对应模块: fmcw_sim_config.m]
%  步骤 1/12：参数配置
%  集中管理所有 FMCW MIMO 雷达仿真参数
%% ========================================================================
fprintf('[步骤 1/12] 参数配置...\n');

% ------ 1. 物理常数 ------
cfg.c = 3e8;  % 光速 (m/s)

% ------ 2. FMCW 波形参数（参考 AWR2243 data_process.m） ------
cfg.fc     = 77e9;       % 载频 77GHz
cfg.gama   = 24e12;      % 调频斜率 (Hz/s)
cfg.fs     = 4e6;        % 采样率 (Hz)
cfg.idle_t = 340e-6;     % 空闲时间 (s)
cfg.start_t = 6e-6;      % 发射信号起始位置 (s)
cfg.end_t  = 160e-6;     % 发射信号结束位置 (s)

% ------ 3. 信号维度 ------
cfg.Ns     = 512;        % chirp 内采样点数（快时间）
cfg.numLoops = 64;       % 一次发射轮询周期 chirp 数（慢时间）

% ------ 4. MIMO 阵列配置（4 芯片级联 AWR2243） ------
cfg.numDevice    = 4;    % 芯片个数
cfg.numRXPerDev  = 4;    % 每个芯片接收天线数
cfg.numTXPerDev  = 3;    % 每个芯片发射天线数
cfg.Ntx = (cfg.numDevice - 1) * cfg.numTXPerDev;  % 方位角发射天线 9
cfg.Nrx = cfg.numDevice * cfg.numRXPerDev;         % 方位角接收天线 16
cfg.Nv  = cfg.Ntx * cfg.Nrx;                       % 方位虚拟阵元数 144
cfg.Ntx_total = cfg.Ntx + 3;  % 总发射天线 = 9(方位) + 3(俯仰) = 12

% 俯仰角阵列参数
cfg.Ntx_el = 3;             % 俯仰发射天线数（芯片4的3个TX）
cfg.Nrx_el = cfg.Nrx;       % 俯仰接收天线数（所有RX参与，与方位共享）
cfg.Nv_el  = cfg.Ntx_el * cfg.Nrx_el;  % 俯仰虚拟阵元数 = 3×16 = 48

% 信号维度（依赖 MIMO 配置）
cfg.numChirpPerLoop = cfg.Ntx + cfg.Ntx_el;  % 一帧内轮询次数 = 12 (9方位+3俯仰)
cfg.Nchirp = cfg.numLoops * cfg.numChirpPerLoop;  % 一帧总 chirp 数 = 768

% 阵元间距（单位：lambda）
cfg.d_az_tx1 = 2;       % 方位维发射天线间距（测方位角）
cfg.d_az_tx2 = 0.5;     % 方位维发射天线间距（用于俯仰TX间距）
cfg.d_el_tx  = [0, 0.5, 2, 3];  % 俯仰维发射天线间距（旧版兼容）
cfg.d_el_tx_pos = [0; 0.5; 2; 3];  % 俯仰维虚拟阵列位置（单位：lambda，4元阵）
% 说明：位置0借用方位TX1（俯仰位置=0λ），0.5/2/3为俯仰TX10~12的垂直位置
cfg.el_borrowed_tx = 1;  % 借用的方位TX索引（TX1，俯仰位置=0λ）
cfg.d_el_rx_pos = [0; 0.5; 1; 1.5];  % 俯仰维RX间距参考（芯片4的4个RX，仅用于参考）
cfg.d_az_rx1 = 0.5;     % 接收天线方位维间距
cfg.d_az_rx2 = 4;       % 接收天线方位维间距（芯片间）
cfg.d_az_rx3 = 16;      % 接收天线方位维间距（芯片间）

% ------ 5. 处理流程开关 ------
cfg.enable_clutter    = true;   % 是否启用杂波
cfg.enable_MTI        = false;  % 静态目标(v=0)须关闭MTI，否则会被对消
cfg.enable_pointcloud = true;   % 是否启用点云可视化
cfg.angle_method      = 'DBF';  % 角度估计方法：'FFT' / 'DBF' / 'MUSIC'

% 图像保存参数（-batch 模式下用于查看图像）
cfg.save_fig = true;            % 是否自动保存图像为 PNG
cfg.fig_dir  = '';               % 图像保存目录（空则使用当前工作目录）

% ------ 6. 杂波参数 ------
cfg.clutter_dist    = 'Rayleigh';  % 杂波幅度分布：'Rayleigh'/'Weibull'/'LogNormal'
cfg.clutter_power   = 1e-15;       % 杂波功率 (W)，典型信号功率约 5.77e-15 W
cfg.clutter_shape   = 1.5;         % Weibull 形状参数 k（仅 Weibull 有效）
cfg.clutter_doppler_std = 0.5;     % 杂波多普勒谱宽 (m/s)

% ------ 7. 噪声参数 ------
cfg.SNR_dB = 20;  % 信噪比 (dB)

% ------ 8. CFAR 参数（1D + 2D） ------
% 一维 CFAR（距离维）
cfg.cfar1_Pfa = 1e-5;    % 虚警概率
cfg.cfar1_Tr  = 15;      % 距离维参考单元数（单边）
cfg.cfar1_Gr  = 10;      % 距离维保护单元数（单边）

% 二维 CFAR（R-D 谱）
cfg.cfar2_Pfa = 1e-6;    % 虚警概率
cfg.cfar2_Tr  = 3;       % 距离维参考单元数（单边）
cfg.cfar2_Td  = 1;       % 多普勒维参考单元数（单边）
cfg.cfar2_Gr  = 2;       % 距离维保护单元数（单边）
cfg.cfar2_Gd  = 0;       % 多普勒维保护单元数（单边）

% ------ 9. 角度估计参数 ------
cfg.az_angle_range = -60:0.1:60;   % 方位角扫描范围 (°)
cfg.el_angle_range = -30:0.1:30;   % 俯仰角扫描范围 (°)
cfg.angle_fft_n    = 256;           % 角度 FFT 点数
cfg.music_nsig     = 2;             % MUSIC 信源数

% ------ [原 compute_derived 子函数] 导出计算参数 ------
% 波长
cfg.lambda = cfg.c / cfg.fc;

% 有效 chirp 时间与周期
cfg.Tchirp = cfg.end_t - cfg.start_t;
cfg.Tp = cfg.idle_t + cfg.end_t;

% 扫频带宽
cfg.B = cfg.gama * cfg.Tchirp;

% 调频斜率别名
cfg.mu = cfg.gama;

% 分辨率
cfg.Rres = cfg.c * cfg.fs / (2 * cfg.gama * cfg.Ns);
cfg.Vres = cfg.lambda / (2 * cfg.numLoops * cfg.Tp);

% 最大无模糊距离/速度
cfg.Rmax = cfg.c * cfg.fs / (2 * cfg.gama * cfg.Ns) * cfg.Ns;
cfg.Vmax = cfg.lambda / (4 * cfg.Tp);

% 每 TX 的 chirp 数
cfg.Nchirp_per_tx = cfg.numLoops;

% 距离轴与速度轴
cfg.R_axis = (0:cfg.Ns-1) * cfg.Rres / 1e-2;       % 单位：cm
cfg.V_axis = (-1) * ((-cfg.numLoops/2:cfg.numLoops/2-1) * cfg.Vres);  % 单位：m/s

% ====== 天线位置计算 ======
% 方位维发射天线位置
cfg.az_tx_pos = (0:cfg.Ntx-1) * cfg.d_az_tx1;

% 方位维接收天线位置（4 芯片级联布局）
rx_pos_perDev = (0:cfg.numRXPerDev-1) * cfg.d_az_rx1;
cfg.az_rx_pos = [rx_pos_perDev, ...
                 1.5 + cfg.d_az_rx2 + rx_pos_perDev, ...
                 3 + cfg.d_az_rx2 + cfg.d_az_rx3 + rx_pos_perDev, ...
                 5 + cfg.d_az_rx2 + cfg.d_az_rx3 + rx_pos_perDev];

% ====== 虚拟阵列位置 ======
% 方位维虚拟阵元位置（含重叠）
virtual_az_origin = zeros(1, cfg.Nv);
for tx_idx = 1:cfg.Ntx
    idx_start = (tx_idx - 1) * cfg.Nrx + 1;
    idx_end   = tx_idx * cfg.Nrx;
    virtual_az_origin(idx_start:idx_end) = cfg.az_tx_pos(tx_idx) + cfg.az_rx_pos;
end

% 去除重叠后的方位维虚拟阵元位置（单位：m）
cfg.virtual_az_pos = unique(virtual_az_origin) * cfg.lambda;
cfg.Nv_unique = length(cfg.virtual_az_pos);  % 去重叠虚拟阵元数

% ====== 俯仰维虚拟阵元位置 ======
% 俯仰虚拟阵列由3个俯仰TX + 1个借用方位TX组成4元阵
% 借用TX1（俯仰位置=0λ）+ TX10(0.5λ) + TX11(2λ) + TX12(3λ)
cfg.el_tx_pos = cfg.d_el_tx_pos;  % 俯仰虚拟阵列位置（含借用，4元）(单位：lambda)
cfg.el_tx_pos_physical = cfg.d_el_tx_pos(2:end);  % 物理俯仰TX位置（3个，不含借用）(单位：lambda)
cfg.el_rx_pos = cfg.d_el_rx_pos;  % 俯仰接收天线参考位置 (单位：lambda)

% 俯仰虚拟阵元位置（去重叠，4个不同位置）
cfg.virtual_el_pos = sort(unique(cfg.el_tx_pos)) * cfg.lambda;  % 去重叠 (单位：m)
cfg.Nv_el_unique = length(cfg.virtual_el_pos);

% ------ [原 validate_params 子函数] 参数合法性校验 ------
% numLoops 必须为正整数
if cfg.numLoops <= 0 || mod(cfg.numLoops, 1) ~= 0
    error('numLoops(%d) 必须为正整数', cfg.numLoops);
end
% 带宽必须为正
if cfg.B <= 0
    error('带宽 B 必须为正，当前值: %.2e Hz', cfg.B);
end
% 采样率必须为正
if cfg.fs <= 0
    error('采样率 fs 必须为正，当前值: %.2e Hz', cfg.fs);
end
% Tchirp 必须大于零
if cfg.Tchirp <= 0
    error('有效 chirp 时间必须大于零，当前值: %.2e s', cfg.Tchirp);
end
% 角度估计方法合法性
valid_methods = {'FFT', 'DBF', 'MUSIC'};
if ~ismember(cfg.angle_method, valid_methods)
    error('angle_method 必须为 %s 之一，当前值: %s', ...
        strjoin(valid_methods, '/'), cfg.angle_method);
end

fprintf('  载频: %.1f GHz, 波长: %.4f mm\n', cfg.fc/1e9, cfg.lambda*1e3);
fprintf('  距离分辨率: %.4f m, 速度分辨率: %.4f m/s\n', cfg.Rres, cfg.Vres);
fprintf('  最大距离: %.2f m, 最大速度: %.2f m/s\n', cfg.Rmax, cfg.Vmax);
fprintf('  方位TX=%d, 俯仰TX=%d, 总TX=%d, RX=%d\n', ...
    cfg.Ntx, cfg.Ntx_el, cfg.Ntx_total, cfg.Nrx);
fprintf('  方位虚拟阵元: %d (去重叠 %d)\n', cfg.Nv, cfg.Nv_unique);
fprintf('  俯仰虚拟阵元: %d (去重叠 %d)\n', cfg.Nv_el, cfg.Nv_el_unique);
fprintf('  角度估计方法: %s\n', cfg.angle_method);
fprintf('  杂波: %s, MTI: %s\n', ...
    mat2str(cfg.enable_clutter), mat2str(cfg.enable_MTI));
fprintf('参数配置完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_signal.m]
%  步骤 2/12：TDM-MIMO 差频信号生成
%  信号模型（去斜接收/Dechirp）：
%    快时间差频相位: exp(j·2π·fb·t_fast),  fb = 2·mu·R/c
%    TDM慢时间相位:  exp(j·4π·R(t_slow)/λ)
%    TX空间相位(方位): exp(j·2π·pos_tx_az·sin(θ_az)/λ)
%    TX空间相位(俯仰): exp(j·2π·pos_tx_el·sin(θ_el)/λ)
%    RX空间相位(方位): exp(j·2π·pos_rx_az·sin(θ_az)/λ)
%    幅度:           sqrt(RCS) · (λ/(4πR))²  自由空间衰减
%% ========================================================================
fprintf('[步骤 2/12] TDM-MIMO 差频信号生成...\n');

% ------ [原 validate_inputs 子函数] 输入校验 ------
if ~isstruct(cfg)
    error('cfg 必须为结构体');
end
req_fields = {'c', 'lambda', 'mu', 'fs', 'Ns', 'Ntx', 'Nrx', ...
              'Tp', 'numLoops', 'az_tx_pos', 'az_rx_pos', 'Tchirp'};
for i = 1:length(req_fields)
    if ~isfield(cfg, req_fields{i})
        error('cfg 缺少必要字段: %s', req_fields{i});
    end
end
if ~isnumeric(targets) || size(targets, 2) < 4
    error('targets 必须为 N×4 [R,v,θ_az,RCS] 或 N×5 [R,v,θ_az,θ_el,RCS] 矩阵');
end
if any(targets(:, 1) <= 0)
    error('目标距离必须为正值');
end
rcs_col_check = min(size(targets, 2), 4) + (size(targets, 2) >= 5);
if any(targets(:, rcs_col_check) <= 0)
    error('目标 RCS 必须为正值');
end

% ------ 目标矩阵兼容处理 ------
% 支持 N×4（无俯仰角）和 N×5（含俯仰角）两种格式
if size(targets, 2) >= 5
    theta_el_targets = targets(:, 4);   % 俯仰角 (°)，第4列
    rcs_col = 5;                         % RCS 在第5列
else
    theta_el_targets = zeros(size(targets, 1), 1);  % 俯仰角默认 0°
    rcs_col = 4;                         % RCS 在第4列
end

% ------ 提取参数 ------
c      = cfg.c;
lambda = cfg.lambda;
mu     = cfg.mu;         % 调频斜率 = gama
fs     = cfg.fs;
Ns     = cfg.Ns;
Ntx    = cfg.Ntx;        % 方位TX数（9）
Nrx    = cfg.Nrx;        % RX总数（16）
Ntx_el = cfg.Ntx_el;    % 俯仰TX数（3）
Nrx_el = cfg.Nrx_el;    % 俯仰RX数（4）
Tp     = cfg.Tp;         % chirp 周期（含空闲）
numLoops = cfg.numLoops; % 每TX的chirp数

% 方位天线位置（单位：m）
pos_tx_az = cfg.az_tx_pos * lambda;
pos_rx_az = cfg.az_rx_pos * lambda;

% 俯仰天线位置（单位：m）
% 信号生成使用物理俯仰TX位置（3个），不含借用的方位TX
pos_tx_el = cfg.el_tx_pos_physical(:)' * lambda;  % [1 × Ntx_el]
pos_rx_el = cfg.el_rx_pos(:)' * lambda;  % [1 × Nrx_el]

numTargets = size(targets, 1);

% ------ 快时间轴 ------
t_fast = (0:Ns-1).' / fs;  % [Ns × 1]

% ------ 初始化输出 ------
% 总 TX 数 = 方位 TX + 俯仰 TX = 9 + 3 = 12
Ntx_total = Ntx + Ntx_el;
if_signal = zeros(Ns, numLoops, Nrx, Ntx_total);

% ------ 方位TX信号生成（TX 1~9）------
for tx = 1:Ntx
    for t_idx = 1:numTargets
        R0    = targets(t_idx, 1);
        v     = targets(t_idx, 2);
        theta_az = targets(t_idx, 3) * pi / 180;  % 方位角转弧度
        theta_el = theta_el_targets(t_idx) * pi / 180;  % 俯仰角转弧度
        rcs   = targets(t_idx, rcs_col);

        % 幅度：自由空间传播衰减 + RCS
        amp = sqrt(rcs) * (lambda / (4 * pi * R0))^2;

        % 方位TX空间相位
        phase_tx_az = exp(1j * 2 * pi * pos_tx_az(tx) * sin(theta_az) / lambda);

        % 跳过超出最大无模糊距离的目标
        tau_max = 2 * R0 / c;
        if tau_max >= cfg.Tchirp
            continue;
        end

        for k = 1:numLoops
            t_slow = ((k - 1) * Ntx_total + (tx - 1)) * Tp;
            R_cur = R0 + v * t_slow;

            fb = 2 * mu * R_cur / c;
            phase_fast = exp(1j * 2 * pi * fb * t_fast);
            phase_slow = exp(1j * 4 * pi * R_cur / lambda);

            for rx = 1:Nrx
                % RX在水平面排列，仅有方位相位，无俯仰相位
                phase_rx_az = exp(1j * 2 * pi * pos_rx_az(rx) * sin(theta_az) / lambda);

                if_signal(:, k, rx, tx) = if_signal(:, k, rx, tx) + ...
                    amp * phase_fast * phase_slow * phase_tx_az * phase_rx_az;
            end
        end
    end
end

% ------ 俯仰TX信号生成（TX 10~12）------
for tx_el = 1:Ntx_el
    tx_global = Ntx + tx_el;  % 全局TX索引（10~12）
    for t_idx = 1:numTargets
        R0    = targets(t_idx, 1);
        v     = targets(t_idx, 2);
        theta_az = targets(t_idx, 3) * pi / 180;
        theta_el = theta_el_targets(t_idx) * pi / 180;
        rcs   = targets(t_idx, rcs_col);

        amp = sqrt(rcs) * (lambda / (4 * pi * R0))^2;

        % 俯仰TX空间相位（俯仰方向）
        phase_tx_el = exp(1j * 2 * pi * pos_tx_el(tx_el) * sin(theta_el) / lambda);

        tau_max = 2 * R0 / c;
        if tau_max >= cfg.Tchirp
            continue;
        end

        for k = 1:numLoops
            t_slow = ((k - 1) * Ntx_total + (tx_global - 1)) * Tp;
            R_cur = R0 + v * t_slow;

            fb = 2 * mu * R_cur / c;
            phase_fast = exp(1j * 2 * pi * fb * t_fast);
            phase_slow = exp(1j * 4 * pi * R_cur / lambda);

            for rx = 1:Nrx
                % RX在水平面排列，仅有方位相位
                phase_rx_az = exp(1j * 2 * pi * pos_rx_az(rx) * sin(theta_az) / lambda);

                if_signal(:, k, rx, tx_global) = if_signal(:, k, rx, tx_global) + ...
                    amp * phase_fast * phase_slow * phase_tx_el * phase_rx_az;
            end
        end
    end
end

fprintf('  信号维度: [%s]\n', num2str(size(if_signal)));
fprintf('  信号功率: %.2e W\n', mean(abs(if_signal(:)).^2));
fprintf('信号生成完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_clutter.m]
%  步骤 3/12：杂波叠加
%  支持杂波分布: 'Rayleigh'(复高斯散斑) / 'Weibull'(ZMNL变换) / 'K'(SIRP模型)
%  杂波特性: 高斯多普勒谱塑形(频域滤波), CNR功率控制
%% ========================================================================
fprintf('[步骤 3/12] 杂波叠加...\n');

if cfg.enable_clutter
    % ------ 参数提取 ------
    clutter_dims = size(if_signal);
    clutter_Ns       = clutter_dims(1);
    clutter_numLoops = clutter_dims(2);
    clutter_Nrx      = clutter_dims(3);
    clutter_Ntx      = clutter_dims(4);
    clutter_dist     = cfg.clutter_dist;
    clutter_power    = cfg.clutter_power;
    clutter_shape    = cfg.clutter_shape;
    sigma_fd         = cfg.clutter_doppler_std;  % 杂波多普勒谱宽 (m/s)

    % ------ 生成杂波（逐通道） ------
    clutter = zeros(clutter_dims);

    for tx = 1:clutter_Ntx
        for rx = 1:clutter_Nrx
            % [原 generate_speckle 子函数] 生成复高斯散斑并施加高斯多普勒谱塑形
            speckle = (randn(clutter_Ns, clutter_numLoops) + 1j * randn(clutter_Ns, clutter_numLoops)) / sqrt(2);

            % 多普勒谱塑形（频域滤波）
            if sigma_fd > 0
                % [原 fftfreq 子函数] 计算 FFT 频率轴
                n_fft = clutter_numLoops;
                d_fft = 1 / cfg.Tp;
                if mod(n_fft, 2) == 0
                    fd_axis = [0:(n_fft/2-1), (-n_fft/2):(-1)]' / (n_fft * d_fft);
                else
                    fd_axis = [0:((n_fft-1)/2), (-(n_fft-1)/2):(-1)]' / (n_fft * d_fft);
                end

                % 高斯功率谱: S(f) = exp(-f^2 / (2*sigma_f^2))
                % sigma_fd (m/s) 转换为频率: sigma_f = 2*sigma_fd/lambda
                sigma_f = 2 * sigma_fd / lambda;
                H_doppler = exp(-fd_axis.^2 / (2 * sigma_f^2));
                H_doppler = H_doppler.';  % 转为行向量 [1 × numLoops]

                % 频域滤波（沿慢时间维）
                for n = 1:clutter_Ns
                    X = fft(speckle(n, :));
                    X = X .* H_doppler;
                    speckle(n, :) = ifft(X);
                end
            end

            % 按杂波分布调制幅度
            switch upper(clutter_dist)
                case 'RAYLEIGH'
                    % Rayleigh = 复高斯，幅度已为 Rayleigh 分布
                    clutter(:, :, rx, tx) = speckle;

                case 'WEIBULL'
                    % [原 apply_weibull_zmnl 子函数] ZMNL 变换：|高斯| → Weibull
                    amp_speckle = abs(speckle);
                    phase_speckle = angle(speckle);
                    % Rayleigh → 均匀分布
                    U = 1 - exp(-amp_speckle.^2 / 2);
                    % 均匀 → Weibull（逆 CDF）
                    w_amp = (-log(1 - U)).^(1/clutter_shape);
                    % 保持原始相位
                    clutter(:, :, rx, tx) = w_amp .* exp(1j * phase_speckle);

                case 'K'
                    % SIRP 模型：Gamma 纹理 × 复高斯散斑
                    nu = clutter_shape;  % 形状参数
                    texture = gamrnd(nu, 1/nu, 1, clutter_numLoops);
                    texture_3d = repmat(texture, clutter_Ns, 1);
                    clutter(:, :, rx, tx) = sqrt(texture_3d) .* speckle;

                otherwise
                    error('未知杂波分布类型: %s', clutter_dist);
            end
        end
    end

    % ------ CNR 功率控制 ------
    sig_power_clutter = mean(abs(if_signal(:)).^2);
    if sig_power_clutter > 0
        clutter_current = mean(abs(clutter(:)).^2);
        if clutter_current > 0
            scale = sqrt(clutter_power / clutter_current);
            clutter = clutter * scale;
        end
    end

    % ------ 叠加杂波 ------
    if_signal = if_signal + clutter;

    fprintf('  杂波分布: %s, 功率: %.2e W\n', cfg.clutter_dist, cfg.clutter_power);
else
    fprintf('  杂波已禁用\n');
end
fprintf('杂波叠加完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_noise.m]
%  步骤 4/12：噪声叠加
%  循环对称复高斯白噪声，实部和虚部独立同分布 N(0, noise_power/2)
%  噪声功率 = 信号功率 / 10^(SNR_dB/10)
%% ========================================================================
fprintf('[步骤 4/12] 噪声叠加...\n');

SNR_dB = cfg.SNR_dB;
noise_dims = size(if_signal);

% 计算信号功率
sig_power_noise = mean(abs(if_signal(:)).^2);

if sig_power_noise == 0
    % 纯信号为零时无法计算 SNR，使用单位功率
    noise_power = 1;
else
    noise_power = sig_power_noise / (10^(SNR_dB / 10));
end

% 生成复高斯白噪声（实部和虚部各方差为 noise_power/2）
sigma = sqrt(noise_power / 2);
noise = sigma * (randn(noise_dims) + 1j * randn(noise_dims));

% 叠加噪声
if_signal = if_signal + noise;

fprintf('  SNR: %d dB\n', cfg.SNR_dB);
fprintf('噪声叠加完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_range_fft.m]
%  步骤 5/12：Range-FFT 测距
%  快时间维 Hamming 窗 + FFT，实现测距
%% ========================================================================
fprintf('[步骤 5/12] Range-FFT 测距...\n');

Ns_range = cfg.Ns;
range_dims = size(if_signal);

% 构造 Hamming 窗
win_range = hamming(Ns_range);

% 将窗扩展到多维: if_signal 维度 [Ns, numLoops, Nrx, Ntx]
win_4d = reshape(win_range, [Ns_range, 1, 1, 1]);
if_signal_win = if_signal .* win_4d;

% 沿第1维做 FFT
range_fft_data = fft(if_signal_win, Ns_range, 1);

% 距离轴标定
cfg.range_axis = (0:Ns_range-1) * cfg.Rres;  % 单位：m

fprintf('  距离分辨率 = %.4f m\n', cfg.Rres);
fprintf('Range-FFT 完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_mti.m]
%  步骤 6/12：MTI 两脉冲对消
%  对消公式: y[n] = x[n] - x[n-1]
%  注意: MTI会抑制零速附近的低速目标，静态目标(v=0)须关闭MTI
%% ========================================================================
fprintf('[步骤 6/12] MTI 对消...\n');

if cfg.enable_MTI
    mti_dims = size(range_fft_data);
    mti_Ns = mti_dims(1);
    mti_numLoops = mti_dims(2);
    mti_Nrx = mti_dims(3);
    mti_Ntx = mti_dims(4);

    % 两脉冲对消: y[n] = x[n] - x[n-1], n = 2:N
    mti_data = range_fft_data(:, 2:mti_numLoops, :, :);

    for k = 2:mti_numLoops
        mti_data(:, k-1, :, :) = range_fft_data(:, k, :, :) - range_fft_data(:, k-1, :, :);
    end

    % 更新速度轴和分辨率
    numLoops_new = mti_numLoops - 1;
    cfg.numLoops_mti = numLoops_new;
    cfg.Vres_mti = cfg.lambda / (2 * numLoops_new * cfg.Tp);
    cfg.V_axis_mti = (-1) * ((-numLoops_new/2:numLoops_new/2-1) * cfg.Vres_mti);

    fprintf('  对消后慢时间长度: %d (原 %d)\n', ...
        size(mti_data, 2), size(range_fft_data, 2));
else
    mti_data = range_fft_data;
    fprintf('  MTI 已禁用\n');
end
fprintf('MTI 对消完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_doppler_fft.m]
%  步骤 7/12：Doppler-FFT 测速
%  慢时间维 Hamming 窗 + FFT + fftshift，实现测速
%  TDM-MIMO 有效 PRT = Ntx * Tperiod
%% ========================================================================
fprintf('[步骤 7/12] Doppler-FFT 测速...\n');

dop_dims = size(mti_data);
dop_Ns   = dop_dims(1);
dop_Ndop = dop_dims(2);
dop_Nrx  = dop_dims(3);
dop_Ntx  = dop_dims(4);

% 构造 Hamming 窗
win_dop = hamming(dop_Ndop);

% 加窗
win_4d_dop = reshape(win_dop, [1, dop_Ndop, 1, 1]);
mti_data_win = mti_data .* win_4d_dop;

% 沿第2维做 FFT
rd_data = fft(mti_data_win, dop_Ndop, 2);

% fftshift
rd_data = fftshift(rd_data, 2);

% TDM 速度轴标定
% TDM-MIMO 有效 PRT = Ntx * Tperiod
% 速度分辨率 = lambda / (2 * Ndop * Ntx * Tp)
Vres_tdm = cfg.lambda / (2 * dop_Ndop * dop_Ntx * cfg.Tp);
V_axis_tdm = (-dop_Ndop/2:dop_Ndop/2-1) * Vres_tdm;

cfg.Vres_tdm = Vres_tdm;
cfg.V_axis_tdm = V_axis_tdm;
cfg.Ndop = dop_Ndop;

fprintf('  TDM 速度分辨率 = %.4f m/s\n', cfg.Vres_tdm);
fprintf('Doppler-FFT 完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_virtual_array.m]
%  步骤 8/12：MIMO 虚拟阵列重组（方位 + 俯仰）
%  方位维: TDM多普勒相位补偿 + 重叠阵元相干平均(144→86)
%  俯仰维: 借用方位TX1(俯仰位置=0λ) + 3俯仰TX → 4元阵
%    每个TX对应所有RX做相干平均(16次平均, SNR增益≈12dB)
%% ========================================================================
fprintf('[步骤 8/12] MIMO 虚拟阵列重组（方位 + 俯仰）...\n');

va_dims = size(rd_data);
va_Ns   = va_dims(1);
va_Ndop = va_dims(2);
va_Nrx  = va_dims(3);
va_Ntx_total = va_dims(4);

va_lambda = cfg.lambda;
va_Ntx_az = cfg.Ntx;       % 方位TX数（9）
va_Ntx_el = cfg.Ntx_el;    % 俯仰TX数（3）

% 方位天线位置（单位：lambda）
va_pos_tx_az = cfg.az_tx_pos;
va_pos_rx_az = cfg.az_rx_pos;

% 俯仰TX位置（单位：lambda）
va_pos_tx_el = cfg.el_tx_pos(:)';

% ------ 方位虚拟阵列重组（TX 1~Ntx_az, 所有RX） ------
Nv_az = va_Ntx_az * va_Nrx;
voxel_map_az = zeros(Nv_az, 3);
virtual_az_lambda = zeros(1, Nv_az);

idx = 0;
for tx = 1:va_Ntx_az
    for rx = 1:va_Nrx
        idx = idx + 1;
        voxel_map_az(idx, 1) = tx;
        voxel_map_az(idx, 2) = rx;
        voxel_map_az(idx, 3) = va_pos_tx_az(tx) + va_pos_rx_az(rx);
        virtual_az_lambda(idx) = va_pos_tx_az(tx) + va_pos_rx_az(rx);
    end
end

% ------ TDM 多普勒相位补偿 ------
if isfield(cfg, 'V_axis_tdm')
    V_axis_va = cfg.V_axis_tdm;
else
    V_axis_va = cfg.V_axis;
end

rd_data_comp = rd_data;

for tx = 1:va_Ntx_total
    for d_idx = 1:va_Ndop
        v_est = V_axis_va(d_idx);
        dop_corr = exp(-1j * 4 * pi * v_est * (tx - 1) * cfg.Tp / va_lambda);
        rd_data_comp(:, d_idx, :, tx) = rd_data_comp(:, d_idx, :, tx) * dop_corr;
    end
end

% ------ 方位虚拟阵列去重叠 ------
[unique_az_lambda, ~, ic_az] = unique(virtual_az_lambda);
Nv_az_unique = length(unique_az_lambda);

rd_virtual = zeros(va_Ns, va_Ndop, Nv_az_unique);
for v_idx = 1:Nv_az_unique
    match_idx = find(ic_az == v_idx);
    accum = zeros(va_Ns, va_Ndop);
    for m = 1:length(match_idx)
        tx = voxel_map_az(match_idx(m), 1);
        rx = voxel_map_az(match_idx(m), 2);
        accum = accum + rd_data_comp(:, :, rx, tx);
    end
    rd_virtual(:, :, v_idx) = accum / length(match_idx);
end

% ------ 俯仰虚拟阵列（借用TX1 + TX10~12, 所有RX相干平均） ------
% 借用方位TX1（俯仰位置=0λ）+ 3个俯仰TX → 4元虚拟阵列
Nv_el_virtual = va_Ntx_el + 1;  % 4元（3俯仰TX + 1借用方位TX）
rd_el_virtual = zeros(va_Ns, va_Ndop, Nv_el_virtual);

% 第1个快拍：借用方位TX1（俯仰位置=0λ）
borrowed_tx = cfg.el_borrowed_tx;
accum = zeros(va_Ns, va_Ndop);
for rx = 1:va_Nrx
    accum = accum + rd_data_comp(:, :, rx, borrowed_tx);
end
rd_el_virtual(:, :, 1) = accum / va_Nrx;

% 第2~4个快拍：俯仰TX10~12（俯仰位置=0.5/2/3 λ）
for el_idx = 1:va_Ntx_el
    global_tx = va_Ntx_az + el_idx;
    accum = zeros(va_Ns, va_Ndop);
    for rx = 1:va_Nrx
        accum = accum + rd_data_comp(:, :, rx, global_tx);
    end
    rd_el_virtual(:, :, el_idx + 1) = accum / va_Nrx;
end

% 更新配置
cfg.Nv_unique = Nv_az_unique;
cfg.Nv_el_unique = Nv_el_virtual;
cfg.virtual_az_pos_unique = unique_az_lambda * va_lambda;
cfg.virtual_el_pos_unique = cfg.d_el_tx_pos(:)' * va_lambda;

fprintf('  方位虚拟阵元: %d (去重叠后 %d)\n', cfg.Nv, cfg.Nv_unique);
fprintf('  俯仰虚拟阵元: %d (去重叠后 %d)\n', cfg.Nv_el, cfg.Nv_el_unique);
fprintf('虚拟阵列重组完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_cfar.m]
%  步骤 9/12：二维 CA-CFAR 恒虚警检测
%  处理流程：
%    1. 所有虚拟阵元功率非相干积累
%    2. 计算门限系数 alpha
%    3. 二维滑窗 CA-CFAR 检测
%    4. 边界跳过处理
%% ========================================================================
fprintf('[步骤 9/12] 二维 CA-CFAR 检测...\n');

Pfa = cfg.cfar2_Pfa;
Tr  = cfg.cfar2_Tr;    % 距离维参考单元（单边）
Td  = cfg.cfar2_Td;    % 多普勒维参考单元（单边）
Gr  = cfg.cfar2_Gr;    % 距离维保护单元（单边）
Gd  = cfg.cfar2_Gd;    % 多普勒维保护单元（单边）

cfar_dims = size(rd_virtual);
Nr_cfar  = cfar_dims(1);
Nd_cfar  = cfar_dims(2);
Nv_cfar  = cfar_dims(3);

% 非相干积累
rd_power = zeros(Nr_cfar, Nd_cfar);
for v = 1:Nv_cfar
    rd_power = rd_power + abs(rd_virtual(:, :, v)).^2;
end
rd_power = rd_power / Nv_cfar;

% 计算门限系数
N_total_cfar = (2*Tr + 2*Gr + 1) * (2*Td + 2*Gd + 1);
N_guard_cfar = (2*Gr + 1) * (2*Gd + 1);
N_train_cfar = N_total_cfar - N_guard_cfar;
alpha_cfar = N_train_cfar * (Pfa^(-1/N_train_cfar) - 1);

% 二维 CA-CFAR 检测
det_mask = false(Nr_cfar, Nd_cfar);
det_snr  = zeros(Nr_cfar, Nd_cfar);

r_start = Tr + Gr + 1;
r_end   = Nr_cfar - Tr - Gr;
d_start = Td + Gd + 1;
d_end   = Nd_cfar - Td - Gd;

for d_idx = d_start:d_end
    for r_idx = r_start:r_end
        % 当前检测单元功率
        CUT = rd_power(r_idx, d_idx);

        % 构建参考区域
        ref_region = rd_power(r_idx-Tr-Gr : r_idx+Tr+Gr, ...
                              d_idx-Td-Gd : d_idx+Td+Gd);

        % 零化保护区域
        ref_copy = ref_region;
        r_center = Tr + Gr + 1;
        d_center = Td + Gd + 1;
        ref_copy(r_center-Gr:r_center+Gr, ...
                 d_center-Gd:d_center+Gd) = 0;

        % 计算训练单元均值
        ref_sum = sum(ref_copy(:));
        ref_mean = ref_sum / N_train_cfar;

        % 计算检测阈值
        threshold = ref_mean * alpha_cfar;

        % 检测判决
        if CUT > threshold
            det_mask(r_idx, d_idx) = true;
            if ref_mean > 0
                det_snr(r_idx, d_idx) = 10 * log10(CUT / ref_mean);
            end
        end
    end
end

num_det = sum(det_mask(:));

% 更新配置
cfg.cfar_alpha = alpha_cfar;
cfg.cfar_N_train = N_train_cfar;

fprintf('  检测点数: %d\n', num_det);
fprintf('  门限系数 alpha = %.2f\n', cfg.cfar_alpha);
fprintf('CFAR 检测完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_angle.m]
%  步骤 10/12：角度估计（方位 + 俯仰）
%  方位角: FFT / DBF / MUSIC (86元方位虚拟阵列)
%  俯仰角: DBF (4元俯仰虚拟阵列 [0,0.5,2,3]λ)
%% ========================================================================
fprintf('[步骤 10/12] 角度估计 (%s, 含俯仰)...\n', cfg.angle_method);

angle_dims = size(rd_virtual);
Nr_angle  = angle_dims(1);
Nd_angle  = angle_dims(2);

angle_lambda = cfg.lambda;
angle_method = cfg.angle_method;

% 方位虚拟阵元位置
if isfield(cfg, 'virtual_az_pos_unique')
    virtual_az_pos = cfg.virtual_az_pos_unique;
else
    virtual_az_pos = cfg.virtual_az_pos;
end

% 俯仰虚拟阵元位置
has_elevation = ~isempty(rd_el_virtual);
if has_elevation
    if isfield(cfg, 'virtual_el_pos_unique')
        virtual_el_pos = cfg.virtual_el_pos_unique;
    else
        virtual_el_pos = cfg.virtual_el_pos;
    end
end

% 距离轴与速度轴
range_axis = cfg.range_axis;
if isfield(cfg, 'V_axis_tdm')
    vel_axis = cfg.V_axis_tdm;
else
    vel_axis = cfg.V_axis;
end

% ------ 提取检测点索引 ------
[det_r, det_d] = find(det_mask);
num_det_angle = length(det_r);

if num_det_angle == 0
    estimates = struct('range', {}, 'velocity', {}, 'angle', {}, ...
                       'elevation', {}, 'snr_dB', {});
else
    % 使用动态列表收集估计结果（同 R-D 单元可能产生多个角度估计）
    est_list = struct('range', {}, 'velocity', {}, 'angle', {}, ...
                      'elevation', {}, 'snr_dB', {});
    est_count = 0;

    for i = 1:num_det_angle
        r_idx = det_r(i);
        d_idx = det_d(i);

        % 方位虚拟阵列快拍
        x_az = rd_virtual(r_idx, d_idx, :);
        x_az = x_az(:);

        R_est = range_axis(r_idx);
        V_est = vel_axis(d_idx);

        % SNR 估计
        sig_pow = abs(x_az' * x_az) / length(x_az);
        snr_dB_est = 10 * log10(sig_pow + eps);

        % ------ 方位角估计 ------
        switch upper(angle_method)
            case 'FFT'
                % [原 angle_fft_method 子函数] FFT 角度估计（单峰）
                Nfft = cfg.angle_fft_n;
                virtual_az_pos_fft = virtual_az_pos(:);
                X = fftshift(fft(x_az, Nfft));
                [~, peak_idx] = max(abs(X));
                sin_axis = (-Nfft/2:Nfft/2-1) / Nfft * 2;
                d_min = min(diff(sort(virtual_az_pos_fft)));
                sin_theta = sin_axis(peak_idx) * angle_lambda / (2 * d_min);

                % 抛物线插值精化
                if peak_idx > 1 && peak_idx < Nfft
                    y1 = abs(X(peak_idx - 1));
                    y2 = abs(X(peak_idx));
                    y3 = abs(X(peak_idx + 1));
                    delta = (y1 - y3) / (2 * (y1 - 2*y2 + y3) + eps);
                    sin_theta = sin_theta + delta * (2 / Nfft) * angle_lambda / (2 * d_min);
                end

                sin_theta = max(-1, min(1, sin_theta));
                az_angles = asind(sin_theta);

            case 'DBF'
                % [原 angle_dbf_method 子函数] DBF 角度估计（方位维）
                % 改进：多峰检测，支持同距离同速度多目标角度分辨
                scan_range_az = cfg.az_angle_range;
                P_az = length(scan_range_az);
                step_az = scan_range_az(2) - scan_range_az(1);
                virtual_az_pos_dbf = virtual_az_pos(:);
                spectrum_az = zeros(1, P_az);
                for p = 1:P_az
                    theta_rad = deg2rad(scan_range_az(p));
                    a = exp(1j * 2 * pi * virtual_az_pos_dbf * sin(theta_rad) / angle_lambda);
                    spectrum_az(p) = abs(a' * x_az)^2;
                end

                % 多峰检测：找所有局部峰值点
                spec_db = 10 * log10(spectrum_az + eps);
                spec_max_db = max(spec_db);
                peak_threshold_db = spec_max_db - 10;  % 峰值门限：低于最大峰 10dB 的不算
                az_angles = [];
                for p = 2:P_az-1
                    if spectrum_az(p) > spectrum_az(p-1) && spectrum_az(p) > spectrum_az(p+1) ...
                       && spec_db(p) > peak_threshold_db
                        % 抛物线插值精化峰值位置
                        y1 = spectrum_az(p-1);
                        y2 = spectrum_az(p);
                        y3 = spectrum_az(p+1);
                        delta = (y1 - y3) / (2 * (y1 - 2*y2 + y3) + eps);
                        az_angles(end+1) = scan_range_az(p) + delta * step_az;
                    end
                end
                % 边界峰值检测
                if spectrum_az(1) > spectrum_az(2) && spec_db(1) > peak_threshold_db
                    az_angles = [scan_range_az(1), az_angles];
                end
                if spectrum_az(end) > spectrum_az(end-1) && spec_db(end) > peak_threshold_db
                    az_angles = [az_angles, scan_range_az(end)];
                end
                % 若无峰值（极端情况），取全局最大值
                if isempty(az_angles)
                    [~, pk] = max(spectrum_az);
                    az_angles = scan_range_az(pk);
                end

            case 'MUSIC'
                % [原 angle_music_method 子函数] MUSIC 角度估计（多峰检测）
                N_music = length(x_az);
                M_music = cfg.music_nsig;
                virtual_az_pos_m = virtual_az_pos(:);
                Rxx = x_az * x_az';
                [V_music, D_music] = eig(Rxx);
                eigvals = real(diag(D_music));
                [eigvals, sort_idx] = sort(eigvals, 'ascend');
                V_music = V_music(:, sort_idx);
                Un = V_music(:, 1:N_music-M_music);
                az_scan = cfg.az_angle_range;
                P_m = length(az_scan);
                step_m = az_scan(2) - az_scan(1);
                pseudo_spectrum = zeros(1, P_m);
                for p = 1:P_m
                    theta_rad = deg2rad(az_scan(p));
                    a = exp(1j * 2 * pi * virtual_az_pos_m * sin(theta_rad) / angle_lambda);
                    pseudo_spectrum(p) = 1 / (abs(a' * Un * Un' * a) + eps);
                end
                pseudo_db = 10 * log10(pseudo_spectrum + eps);
                pseudo_max_db = max(pseudo_db);
                peak_threshold_m_db = pseudo_max_db - 10;
                az_angles = [];
                for p = 2:P_m-1
                    if pseudo_spectrum(p) > pseudo_spectrum(p-1) && pseudo_spectrum(p) > pseudo_spectrum(p+1) ...
                       && pseudo_db(p) > peak_threshold_m_db
                        y1 = pseudo_spectrum(p-1);
                        y2 = pseudo_spectrum(p);
                        y3 = pseudo_spectrum(p+1);
                        delta = (y1 - y3) / (2 * (y1 - 2*y2 + y3) + eps);
                        az_angles(end+1) = az_scan(p) + delta * step_m;
                    end
                end
                if pseudo_spectrum(1) > pseudo_spectrum(2) && pseudo_db(1) > peak_threshold_m_db
                    az_angles = [az_scan(1), az_angles];
                end
                if pseudo_spectrum(end) > pseudo_spectrum(end-1) && pseudo_db(end) > peak_threshold_m_db
                    az_angles = [az_angles, az_scan(end)];
                end
                if isempty(az_angles)
                    [~, pk] = max(pseudo_spectrum);
                    az_angles = az_scan(pk);
                end

            otherwise
                error('未知角度估计方法: %s', angle_method);
        end

        % ------ 俯仰角估计（固定使用 DBF，多峰检测） ------
        if has_elevation
            x_el = rd_el_virtual(r_idx, d_idx, :);
            x_el = x_el(:);

            scan_range_el = cfg.el_angle_range;
            P_el = length(scan_range_el);
            step_el = scan_range_el(2) - scan_range_el(1);
            virtual_el_pos_dbf = virtual_el_pos(:);
            spectrum_el = zeros(1, P_el);
            for p = 1:P_el
                theta_rad = deg2rad(scan_range_el(p));
                a = exp(1j * 2 * pi * virtual_el_pos_dbf * sin(theta_rad) / angle_lambda);
                spectrum_el(p) = abs(a' * x_el)^2;
            end

            % 俯仰维多峰检测
            % 注意：俯仰阵元仅4个 [0, 0.5, 2, 3]λ，稀疏阵栅瓣严重
            % 使用极严格门限(3dB) + 峰值数量限制，滤除栅瓣
            spec_el_db = 10 * log10(spectrum_el + eps);
            spec_el_max_db = max(spec_el_db);
            peak_threshold_el_db = spec_el_max_db - 10;   % 俯仰维用极严格门限减少虚假目标
            max_el_peaks = 4;  % 俯仰维最多保留4个峰
            el_angles = [];
            el_angles_db = [];  % 记录峰值dB用于排序
            for p = 2:P_el-1
                if spectrum_el(p) > spectrum_el(p-1) && spectrum_el(p) > spectrum_el(p+1) ...
                   && spec_el_db(p) > peak_threshold_el_db
                    y1 = spectrum_el(p-1);
                    y2 = spectrum_el(p);
                    y3 = spectrum_el(p+1);
                    delta = (y1 - y3) / (2 * (y1 - 2*y2 + y3) + eps);
                    el_angles(end+1) = scan_range_el(p) + delta * step_el;
                    el_angles_db(end+1) = spec_el_db(p);
                end
            end
            % 边界峰值
            if length(el_angles) < max_el_peaks && spectrum_el(1) > spectrum_el(2) ...
               && spec_el_db(1) > peak_threshold_el_db
                el_angles = [scan_range_el(1), el_angles];
                el_angles_db = [spec_el_db(1), el_angles_db];
            end
            if length(el_angles) < max_el_peaks && spectrum_el(end) > spectrum_el(end-1) ...
               && spec_el_db(end) > peak_threshold_el_db
                el_angles = [el_angles, scan_range_el(end)];
                el_angles_db = [el_angles_db, spec_el_db(end)];
            end
            % 限制峰值数量（按幅度排序取前N）
            if length(el_angles) > max_el_peaks
                [~, sort_idx] = sort(el_angles_db, 'descend');
                el_angles = el_angles(sort_idx(1:max_el_peaks));
            end
            if isempty(el_angles)
                [~, pk_el] = max(spectrum_el);
                if pk_el > 1 && pk_el < P_el
                    y1 = spectrum_el(pk_el-1);
                    y2 = spectrum_el(pk_el);
                    y3 = spectrum_el(pk_el+1);
                    delta = (y1 - y3) / (2 * (y1 - 2*y2 + y3) + eps);
                    el_angles = scan_range_el(pk_el) + delta * step_el;
                else
                    el_angles = scan_range_el(pk_el);
                end
            end
        else
            el_angles = 0;
        end

        % ------ 方位×俯仰交叉组合，生成估计列表 ------
        num_az_peaks = length(az_angles);
        num_el_peaks = length(el_angles);
        for ak = 1:num_az_peaks
            for ek = 1:num_el_peaks
                est_count = est_count + 1;
                est_list(est_count).range = R_est;
                est_list(est_count).velocity = V_est;
                est_list(est_count).angle = az_angles(ak);
                est_list(est_count).snr_dB = snr_dB_est;
                est_list(est_count).elevation = el_angles(ek);
            end
        end
    end

    estimates = est_list;
end

fprintf('  估计目标数: %d\n', length(estimates));
for i = 1:length(estimates)
    fprintf('    目标%d: R=%.3fm, V=%.3fm/s, Az=%.2f°, El=%.2f°, SNR=%.1fdB\n', ...
        i, estimates(i).range, estimates(i).velocity, ...
        estimates(i).angle, estimates(i).elevation, estimates(i).snr_dB);
end
fprintf('角度估计完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_pointcloud.m]
%  步骤 11/12：点云结构化输出与可视化（含俯仰角）
%  3D坐标转换:
%    x = R × cos(El) × sin(Az)    → 横向
%    y = R × cos(El) × cos(Az)    → 纵向
%    z = R × sin(El)               → 高度
%% ========================================================================
if cfg.enable_pointcloud
    fprintf('[步骤 11/12] 点云可视化...\n');

    num_det_pc = length(estimates);
    pointcloud = struct('range', cell(1, num_det_pc), ...
                        'velocity', cell(1, num_det_pc), ...
                        'angle', cell(1, num_det_pc), ...
                        'elevation', cell(1, num_det_pc), ...
                        'snr_dB', cell(1, num_det_pc), ...
                        'power', cell(1, num_det_pc), ...
                        'x', cell(1, num_det_pc), ...
                        'y', cell(1, num_det_pc), ...
                        'z', cell(1, num_det_pc));

    for i = 1:num_det_pc
        R = estimates(i).range;
        V = estimates(i).velocity;
        theta_az = estimates(i).angle;
        theta_el = estimates(i).elevation;
        snr = estimates(i).snr_dB;

        % 笛卡尔坐标转换（含俯仰角 → 3D坐标）
        az_rad = deg2rad(theta_az);
        el_rad = deg2rad(theta_el);
        x = R * cos(el_rad) * sin(az_rad);
        y = R * cos(el_rad) * cos(az_rad);
        z = R * sin(el_rad);

        power = 10^(snr / 10);

        pointcloud(i).range     = R;
        pointcloud(i).velocity  = V;
        pointcloud(i).angle     = theta_az;
        pointcloud(i).elevation = theta_el;
        pointcloud(i).snr_dB    = snr;
        pointcloud(i).power     = power;
        pointcloud(i).x         = x;
        pointcloud(i).y         = y;
        pointcloud(i).z         = z;
    end

    % ------ 可视化 ------
    fig_handles = [];

    % 提取数据用于绘图
    if num_det_pc > 0
        det_R     = [pointcloud.range];
        det_V     = [pointcloud.velocity];
        det_theta = [pointcloud.angle];
        det_el    = [pointcloud.elevation];
        det_x     = [pointcloud.x];
        det_y     = [pointcloud.y];
        det_z     = [pointcloud.z];
        det_snr   = [pointcloud.snr_dB];
    end

    % 真值数据
    has_targets = ~isempty(targets);
    if has_targets
        tgt_R  = targets(:, 1);
        tgt_V  = targets(:, 2);
        tgt_az = targets(:, 3);
        if size(targets, 2) >= 5
            tgt_el = targets(:, 4);
        else
            tgt_el = zeros(size(targets, 1), 1);
        end
        tgt_x = tgt_R .* cosd(tgt_el) .* sind(tgt_az);
        tgt_y = tgt_R .* cosd(tgt_el) .* cosd(tgt_az);
        tgt_z = tgt_R .* sind(tgt_el);
    end

    % ------ 图1：距离-多普勒图 ------
    fig1 = figure('Name', '距离-多普勒图', 'NumberTitle', 'off');
    if isfield(cfg, 'V_axis_tdm')
        V_axis_plot = cfg.V_axis_tdm;
    else
        V_axis_plot = cfg.V_axis;
    end

    if num_det_pc > 0
        scatter(det_R, det_V, 50, det_snr, 'filled', 'MarkerEdgeColor', 'w');
        colorbar;
        colormap(jet);
        caxis([min(det_snr)-5, max(det_snr)+5]);
    end

    if has_targets
        hold on;
        plot(tgt_R, tgt_V, 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
        hold off;
    end

    xlabel('距离 (m)');
    ylabel('速度 (m/s)');
    title('距离-多普勒图');
    grid on;
    fig_handles(end+1) = fig1;

    % ------ 图2：BEV 鸟瞰图 ------
    fig2 = figure('Name', 'BEV 鸟瞰图', 'NumberTitle', 'off');
    hold on;

    max_R = max(cfg.Rmax, 50);
    for r = 10:10:max_R
        theta_circle = linspace(0, 2*pi, 100);
        plot(r*cos(theta_circle), r*sin(theta_circle), 'k--', 'LineWidth', 0.5);
        text(r*0.7, r*0.7, sprintf('%dm', r), 'FontSize', 8, 'Color', [0.5 0.5 0.5]);
    end

    fov = 120;
    fov_rad = deg2rad(fov/2);
    max_line = max_R;
    plot([0, max_line*sin(fov_rad)], [0, max_line*cos(fov_rad)], 'k-', 'LineWidth', 0.5);
    plot([0, -max_line*sin(fov_rad)], [0, max_line*cos(fov_rad)], 'k-', 'LineWidth', 0.5);

    rectangle('Position', [-0.5, -1, 1, 2], 'FaceColor', 'g', 'EdgeColor', 'k');

    if num_det_pc > 0
        scatter(det_x, det_y, 40, det_snr, 'filled');
        colormap(jet);
        caxis([min(det_snr)-5, max(det_snr)+5]);
        colorbar;
    end

    if has_targets
        plot(tgt_x, tgt_y, 'm^', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    end

    hold off;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('BEV 鸟瞰图');
    grid on;
    fig_handles(end+1) = fig2;

    % ------ 图3：3D 点云（含俯仰角） ------
    if num_det_pc > 0
        fig3 = figure('Name', '3D 点云', 'NumberTitle', 'off');
        scatter3(det_x, det_y, det_z, 40, det_snr, 'filled');
        colormap(jet);
        caxis([min(det_snr)-5, max(det_snr)+5]);
        colorbar;
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('3D 点云（颜色=SNR）');
        grid on;
        if has_targets
            hold on;
            scatter3(tgt_x, tgt_y, tgt_z, 80, 'm', 'filled', 'Marker', '^');
            hold off;
        end
        fig_handles(end+1) = fig3;
    end

    fprintf('点云可视化完成\n\n');
else
    fig_handles = [];
    fprintf('[步骤 11/12] 点云可视化已跳过\n\n');
end

%% ========================================================================
%  [对应模块: fmcw_sim_verify.m]
%  步骤 12/12：仿真验证（方位 + 俯仰）
%  判定阈值: 距离 ≤ 1m, 速度 ≤ 1m/s, 方位角 ≤ 5°, 俯仰角 ≤ 5°
%% ========================================================================
fprintf('[步骤 12/12] 仿真验证...\n');

% 阈值设定
R_TOL   = 1.0;   % 距离容差 (m)
V_TOL   = 1.0;   % 速度容差 (m/s)
AZ_TOL  = 5.0;   % 方位角容差 (°)
EL_TOL  = 5.0;   % 俯仰角容差 (°)

% 目标矩阵兼容处理
num_targets = size(targets, 1);
if size(targets, 2) >= 5
    tgt_az = targets(:, 3);
    tgt_el = targets(:, 4);
else
    tgt_az = targets(:, 3);
    tgt_el = zeros(num_targets, 1);
end

num_det_verify = length(estimates);

% 初始化结果
verify_result = struct();
verify_result.matched = false(num_targets, 1);
verify_result.match_idx = zeros(num_targets, 1);
verify_result.errors = struct('range', cell(num_targets, 1), ...
                              'velocity', cell(num_targets, 1), ...
                              'angle', cell(num_targets, 1), ...
                              'elevation', cell(num_targets, 1));
verify_result.target_pass = false(num_targets, 1);

% 最近邻匹配
for t = 1:num_targets
    tgt_R     = targets(t, 1);
    tgt_V     = targets(t, 2);
    tgt_theta = tgt_az(t);
    tgt_phi   = tgt_el(t);

    best_dist = inf;
    best_idx  = 0;

    for d = 1:num_det_verify
        est_R     = estimates(d).range;
        est_V     = estimates(d).velocity;
        est_theta = estimates(d).angle;
        est_phi   = estimates(d).elevation;

        % 归一化欧几里得距离（含俯仰角）
        dist_R = abs(est_R - tgt_R) / R_TOL;
        dist_V = abs(est_V - tgt_V) / V_TOL;
        dist_A = abs(est_theta - tgt_theta) / AZ_TOL;
        dist_E = abs(est_phi - tgt_phi) / EL_TOL;
        dist_total = sqrt(dist_R^2 + dist_V^2 + dist_A^2 + dist_E^2);

        if dist_total < best_dist
            best_dist = dist_total;
            best_idx  = d;
        end
    end

    % 匹配判定
    if best_idx > 0 && best_dist < 2.0
        verify_result.matched(t) = true;
        verify_result.match_idx(t) = best_idx;

        est = estimates(best_idx);
        err_R = est.range - tgt_R;
        err_V = est.velocity - tgt_V;
        err_A = est.angle - tgt_theta;
        err_E = est.elevation - tgt_phi;

        verify_result.errors(t).range     = err_R;
        verify_result.errors(t).velocity  = err_V;
        verify_result.errors(t).angle     = err_A;
        verify_result.errors(t).elevation = err_E;

        % PASS/FAIL 判定
        if abs(err_R) <= R_TOL && abs(err_V) <= V_TOL && ...
           abs(err_A) <= AZ_TOL && abs(err_E) <= EL_TOL
            verify_result.target_pass(t) = true;
        end
    else
        verify_result.errors(t).range     = NaN;
        verify_result.errors(t).velocity  = NaN;
        verify_result.errors(t).angle     = NaN;
        verify_result.errors(t).elevation = NaN;
    end
end

% RMSE 统计
matched_idx = find(verify_result.matched);
num_matched = length(matched_idx);

if num_matched > 0
    err_R = cell2mat({verify_result.errors(matched_idx).range});
    err_V = cell2mat({verify_result.errors(matched_idx).velocity});
    err_A = cell2mat({verify_result.errors(matched_idx).angle});
    err_E = cell2mat({verify_result.errors(matched_idx).elevation});

    verify_result.rmse_range     = sqrt(mean(err_R.^2));
    verify_result.rmse_velocity  = sqrt(mean(err_V.^2));
    verify_result.rmse_angle     = sqrt(mean(err_A.^2));
    verify_result.rmse_elevation = sqrt(mean(err_E.^2));
else
    verify_result.rmse_range     = NaN;
    verify_result.rmse_velocity  = NaN;
    verify_result.rmse_angle     = NaN;
    verify_result.rmse_elevation = NaN;
end

% 总体判定
all_matched = all(verify_result.matched);
all_pass    = all(verify_result.target_pass);
verify_result.all_pass = all_matched && all_pass;

% 打印验证结果
fprintf('\n========== 仿真验证 ==========\n');
for t = 1:num_targets
    if verify_result.matched(t)
        e = verify_result.errors(t);
        status = 'PASS';
        if ~verify_result.target_pass(t)
            status = 'FAIL';
        end
        fprintf('目标%d: R_err=%.3fm, V_err=%.3fm/s, Az_err=%.2f°, El_err=%.2f° [%s]\n', ...
            t, e.range, e.velocity, e.angle, e.elevation, status);
    else
        fprintf('目标%d: 未匹配 [FAIL]\n', t);
    end
end

if num_matched > 0
    fprintf('RMSE: R=%.3fm, V=%.3fm/s, Az=%.2f°, El=%.2f°\n', ...
        verify_result.rmse_range, verify_result.rmse_velocity, ...
        verify_result.rmse_angle, verify_result.rmse_elevation);
end

if verify_result.all_pass
    fprintf('仿真验证: 通过\n');
else
    fprintf('仿真验证: 失败\n');
end
fprintf('================================\n');

fprintf('仿真验证完成\n\n');

%% ========================================================================
%  [对应模块: fmcw_sim_main.m — 自动保存所有图像]
%  自动保存所有图像为 PNG（可选）
%% ========================================================================
if isfield(cfg, 'save_fig') && cfg.save_fig
    save_dir = cfg.fig_dir;
    if isempty(save_dir)
        save_dir = pwd;
    end
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end

    fig_all = flip(findall(0, 'Type', 'figure'));
    if isempty(fig_all)
        fprintf('无打开的图形窗口，跳过图像保存\n\n');
    else
        fprintf('保存 %d 张图像到: %s\n', length(fig_all), save_dir);
        fig_names = cell(1, length(fig_all));
        for i = 1:length(fig_all)
            name = get(fig_all(i), 'Name');
            if isempty(name)
                name = sprintf('figure_%d', i);
            end
            name = strrep(name, ' ', '_');
            name = strrep(name, '-', '_');
            fig_names{i} = name;
        end
        warning('off', 'MATLAB:handle_graphics:exceptions:SceneNode');
        for i = 1:length(fig_all)
            fh = fig_all(i);
            if ~isvalid(fh)
                continue;
            end
            drawnow;
            fpath = fullfile(save_dir, sprintf('%02d_%s.png', i, fig_names{i}));
            print(fh, fpath, '-dpng', '-r150');
            fprintf('  [%d/%d] %s\n', i, length(fig_all), fpath);
        end
        warning('on', 'MATLAB:handle_graphics:exceptions:SceneNode');
        fprintf('图像保存完成\n\n');
    end
end

%% ========================================================================
%  [对应模块: fmcw_sim_main.m — 结果汇总]
%% ========================================================================
fprintf('============ 结果汇总 ============\n');
fprintf('MIMO 配置: %dAz-Tx × %dRx = %d方位阵元 (%d去重叠) + %dEl-Tx = %d俯仰阵元 (%d去重叠)\n', ...
    cfg.Ntx, cfg.Nrx, cfg.Nv, cfg.Nv_unique, cfg.Ntx_el, cfg.Nv_el, cfg.Nv_el_unique);
fprintf('目标数: %d, 检测点数: %d\n', size(targets,1), num_det);
fprintf('角度算法: %s\n', cfg.angle_method);
fprintf('杂波: %s, MTI: %s\n', ...
    mat2str(cfg.enable_clutter), mat2str(cfg.enable_MTI));

% 真值 vs 估计对比表
has_el = size(targets, 2) >= 5;
fprintf('\n--- 真值 vs 估计 ---\n');
if has_el
    fprintf('%-6s  %-32s  %-32s  %-8s\n', '目标', '真值 [R,V,Az,El,RCS]', '估计 [R,V,Az,El,SNR]', '状态');
else
    fprintf('%-6s  %-24s  %-24s  %-8s\n', '目标', '真值 [R,V,Az,RCS]', '估计 [R,V,Az,SNR]', '状态');
end
fprintf('%s\n', repmat('-', 1, 80));
for t = 1:size(targets, 1)
    if has_el
        tgt_str = sprintf('[%.2f, %.2f, %.1f, %.1f, %.1f]', ...
            targets(t,1), targets(t,2), targets(t,3), targets(t,4), targets(t,5));
    else
        tgt_str = sprintf('[%.2f, %.2f, %.1f, %.1f]', ...
            targets(t,1), targets(t,2), targets(t,3), targets(t,4));
    end
    if verify_result.matched(t)
        eidx = verify_result.match_idx(t);
        est = estimates(eidx);
        est_str = sprintf('[%.3f, %.3f, %.2f, %.2f, %.1f]', ...
            est.range, est.velocity, est.angle, est.elevation, est.snr_dB);
        if verify_result.target_pass(t)
            status_str = 'PASS';
        else
            status_str = 'FAIL';
        end
    else
        est_str = '--- 未匹配 ---';
        status_str = 'FAIL';
    end
    fprintf('  %-4d  %-32s  %-32s  %-8s\n', t, tgt_str, est_str, status_str);
end

% RMSE 统计
if ~isnan(verify_result.rmse_range)
    fprintf('\nRMSE: R=%.4f m, V=%.4f m/s, Az=%.3f°, El=%.3f°\n', ...
        verify_result.rmse_range, verify_result.rmse_velocity, ...
        verify_result.rmse_angle, verify_result.rmse_elevation);
end

% 估计结果详情
if num_det > 0
    fprintf('\n--- 估计结果详情 ---\n');
    for i = 1:length(estimates)
        fprintf('  检测点%d: R=%.4f m, V=%.4f m/s, Az=%.2f°, El=%.2f°, SNR=%.1f dB\n', ...
            i, estimates(i).range, estimates(i).velocity, ...
            estimates(i).angle, estimates(i).elevation, estimates(i).snr_dB);
    end
end

if verify_result.all_pass
    fprintf('\n验证结果: 通过\n');
else
    fprintf('\n验证结果: 失败\n');
end
fprintf('===================================\n');
