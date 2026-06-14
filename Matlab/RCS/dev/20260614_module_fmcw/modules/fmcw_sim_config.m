function cfg = fmcw_sim_config(varargin)
% fmcw_sim_config  集中管理所有 FMCW MIMO 雷达仿真参数
%
%   cfg = fmcw_sim_config()           使用默认参数（参考 AWR2243 级联配置）
%   cfg = fmcw_sim_config('fc',79e9)  通过名称-值对覆盖指定参数
%
%   返回结构体 cfg 包含：
%     物理常数、FMCW 波形参数、信号维度、MIMO 阵列配置、
%     处理开关、杂波参数、噪声参数、CFAR 参数、角度估计参数、导出参数
%
%   输入参数（名称-值对，可选）：
%     'fc'       - 载频 (Hz)，默认 77e9
%     'B'        - 扫频带宽 (Hz)，默认 gama*Tc 计算得出
%     'Tchirp'   - 有效 chirp 时间 (s)，默认 154e-6
%     'Tidle'    - 空闲时间 (s)，默认 340e-6
%     'gama'     - 调频斜率 (Hz/s)，默认 24e12
%     'fs'       - 采样率 (Hz)，默认 4e6
%     'Ns'       - chirp 内采样点数，默认 512
%     'Nchirp'   - 一帧总 chirp 数，默认 768（12轮询×64 chirp/轮询）
%     'Ntx'      - 发射天线数，默认 12
%     'Nrx'      - 接收天线数，默认 16
%     ... 以及其他可覆盖参数

    %% ====== 1. 物理常数 ======
    cfg.c = 3e8;  % 光速 (m/s)

    %% ====== 2. FMCW 波形参数（参考 AWR2243 data_process.m） ======
    cfg.fc     = 77e9;       % 载频 77GHz
    cfg.gama   = 24e12;      % 调频斜率 (Hz/s)
    cfg.fs     = 4e6;        % 采样率 (Hz)
    cfg.idle_t = 340e-6;     % 空闲时间 (s)
    cfg.start_t = 6e-6;      % 发射信号起始位置 (s)
    cfg.end_t  = 160e-6;     % 发射信号结束位置 (s)

    %% ====== 3. 信号维度 ======
    cfg.Ns     = 512;        % chirp 内采样点数（快时间）
    cfg.numLoops = 64;       % 一次发射轮询周期 chirp 数（慢时间）

    %% ====== 4. MIMO 阵列配置（4 芯片级联 AWR2243） ======
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

    %% ====== 5. 处理流程开关 ======
    cfg.enable_clutter    = true;   % 是否启用杂波
    cfg.enable_MTI        = true;   % 是否启用 MTI 对消
    cfg.enable_pointcloud = true;   % 是否启用点云可视化
    cfg.angle_method      = 'DBF';  % 角度估计方法：'FFT' / 'DBF' / 'MUSIC'

    % 图像保存参数（-batch 模式下用于查看图像）
    cfg.save_fig = false;            % 是否自动保存图像为 PNG
    cfg.fig_dir  = '';               % 图像保存目录（空则使用当前工作目录）

    %% ====== 6. 杂波参数 ======
    cfg.clutter_dist    = 'Rayleigh';  % 杂波幅度分布：'Rayleigh'/'Weibull'/'LogNormal'
    cfg.clutter_power   = 1e-15;       % 杂波功率 (W)，典型信号功率约 5.77e-15 W
    cfg.clutter_shape   = 1.5;         % Weibull 形状参数 k（仅 Weibull 有效）
    cfg.clutter_doppler_std = 0.5;     % 杂波多普勒谱宽 (m/s)

    %% ====== 7. 噪声参数 ======
    cfg.SNR_dB = 20;  % 信噪比 (dB)

    %% ====== 8. CFAR 参数（1D + 2D） ======
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

    %% ====== 9. 角度估计参数 ======
    cfg.az_angle_range = -60:0.1:60;   % 方位角扫描范围 (°)
    cfg.el_angle_range = -30:0.1:30;   % 俯仰角扫描范围 (°)
    cfg.angle_fft_n    = 256;           % 角度 FFT 点数
    cfg.music_nsig     = 2;             % MUSIC 信源数

    %% ====== 10. 名称-值对覆盖 ======
    if nargin > 0
        cfg = override_params(cfg, varargin{:});
    end

    %% ====== 11. 导出计算参数 ======
    cfg = compute_derived(cfg);

    %% ====== 12. 参数合法性校验 ======
    validate_params(cfg);
end

%% ========== 子函数：名称-值对覆盖 ==========
function cfg = override_params(cfg, varargin)
% override_params  使用名称-值对覆盖默认参数
%
%   参数：
%     cfg      - 默认配置结构体
%     varargin - 名称-值对，如 'fc', 79e9, 'fs', 5e6
%
%   返回：
%     cfg      - 覆盖后的配置结构体

    % 将 varargin 解析为 名称-值 对
    names  = varargin(1:2:end);
    values = varargin(2:2:end);

    for k = 1:length(names)
        field = names{k};
        if isfield(cfg, field)
            cfg.(field) = values{k};
        else
            error('fmcw_sim_config:unknownParam', ...
                  '未知参数 "%s"，请检查拼写', field);
        end
    end
end

%% ========== 子函数：计算导出参数 ==========
function cfg = compute_derived(cfg)
% compute_derived  根据基础参数自动计算导出参数
%
%   计算项：
%     lambda     - 波长
%     Tchirp     - 有效 chirp 时间
%     Tp         - 一个 chirp 周期（含空闲）
%     B          - 扫频带宽
%     mu         - 调频斜率（gama 的别名）
%     Rres       - 距离分辨率
%     Vres       - 速度分辨率
%     Rmax       - 最大无模糊距离
%     Vmax       - 最大无模糊速度
%     Nchirp_per_tx - 每 TX 的 chirp 数
%     R_axis     - 距离轴（cm）
%     V_axis     - 速度轴（m/s）
%     天线位置、虚拟阵元位置

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

    % 俯仰维天线位置（在下方虚拟阵列位置处统一设置）

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
    % 位置0来自借用的方位TX，位置0.5/2/3来自俯仰TX
    % 所有RX在水平面排列（俯仰维位置=0），仅提供SNR增益
    cfg.el_tx_pos = cfg.d_el_tx_pos;  % 俯仰虚拟阵列位置（含借用，4元）(单位：lambda)
    cfg.el_tx_pos_physical = cfg.d_el_tx_pos(2:end);  % 物理俯仰TX位置（3个，不含借用）(单位：lambda)
    cfg.el_rx_pos = cfg.d_el_rx_pos;  % 俯仰接收天线参考位置 (单位：lambda)

    % 俯仰虚拟阵元位置（去重叠，4个不同位置）
    cfg.virtual_el_pos = sort(unique(cfg.el_tx_pos)) * cfg.lambda;  % 去重叠 (单位：m)
    cfg.Nv_el_unique = length(cfg.virtual_el_pos);
end

%% ========== 子函数：参数合法性校验 ==========
function validate_params(cfg)
% validate_params  校验关键参数的合法性
%
%   校验项：
%     - Nchirp 必须能被 Ntx 整除（TDM-MIMO 时分条件）
%     - 带宽和采样率必须为正
%     - Tchirp 必须大于零
%     - angle_method 必须为合法选项

    % numLoops (每TX慢时间chirp数) 必须为正整数
    if cfg.numLoops <= 0 || mod(cfg.numLoops, 1) ~= 0
        error('fmcw_sim_config:invalidNumLoops', ...
              'numLoops(%d) 必须为正整数', cfg.numLoops);
    end

    % 带宽必须为正
    if cfg.B <= 0
        error('fmcw_sim_config:invalidBandwidth', ...
              '带宽 B 必须为正，当前值: %.2e Hz', cfg.B);
    end

    % 采样率必须为正
    if cfg.fs <= 0
        error('fmcw_sim_config:invalidSampleRate', ...
              '采样率 fs 必须为正，当前值: %.2e Hz', cfg.fs);
    end

    % Tchirp 必须大于零
    if cfg.Tchirp <= 0
        error('fmcw_sim_config:invalidTchirp', ...
              '有效 chirp 时间必须大于零，当前值: %.2e s', cfg.Tchirp);
    end

    % 角度估计方法合法性
    valid_methods = {'FFT', 'DBF', 'MUSIC'};
    if ~ismember(cfg.angle_method, valid_methods)
        error('fmcw_sim_config:invalidAngleMethod', ...
              'angle_method 必须为 %s 之一，当前值: %s', ...
              strjoin(valid_methods, '/'), cfg.angle_method);
    end
end
