function if_signal = fmcw_sim_signal(cfg, targets)
% fmcw_sim_signal  TDM-MIMO 差频（IF）信号生成
%
%   if_signal = fmcw_sim_signal(cfg, targets)
%
%   输入：
%     cfg     - fmcw_sim_config() 返回的参数结构体
%     targets - 目标矩阵 [N×4] 或 [N×5]
%               [R(m), v(m/s), θ_az(°), RCS(m²)] 或
%               [R(m), v(m/s), θ_az(°), θ_el(°), RCS(m²)]
%
%   输出：
%     if_signal - 差频信号数据立方体 [Ns × Nchirp_per_tx × Nrx × Ntx]
%
%   信号模型（去斜接收/Dechirp）：
%     快时间差频相位: exp(j·2π·fb·t_fast),  fb = 2·mu·R/c
%     TDM慢时间相位:  exp(j·4π·R(t_slow)/λ)
%     TX空间相位(方位): exp(j·2π·pos_tx_az·sin(θ_az)/λ)
%     TX空间相位(俯仰): exp(j·2π·pos_tx_el·sin(θ_el)/λ)
%     RX空间相位(方位): exp(j·2π·pos_rx_az·sin(θ_az)/λ)
%     RX空间相位(俯仰): exp(j·2π·pos_rx_el·sin(θ_el)/λ)
%     幅度:           sqrt(RCS) · (λ/(4πR))²  自由空间衰减

    %% ====== 输入校验 ======
    validate_inputs(cfg, targets);

    %% ====== 目标矩阵兼容处理 ======
    % 支持 N×4（无俯仰角）和 N×5（含俯仰角）两种格式
    if size(targets, 2) >= 5
        theta_el_targets = targets(:, 4);   % 俯仰角 (°)，第4列
        rcs_col = 5;                         % RCS 在第5列
    else
        theta_el_targets = zeros(size(targets, 1), 1);  % 俯仰角默认 0°
        rcs_col = 4;                         % RCS 在第4列
    end

    %% ====== 提取参数 ======
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
    numLoops = cfg.numLoops; % 每TX的chirp数 = Nchirp_per_tx

    % 方位天线位置（单位：m）
    pos_tx_az = cfg.az_tx_pos * lambda;
    pos_rx_az = cfg.az_rx_pos * lambda;

    % 俯仰天线位置（单位：m）
    % 信号生成使用物理俯仰TX位置（3个），不含借用的方位TX
    pos_tx_el = cfg.el_tx_pos_physical(:)' * lambda;  % [1 × Ntx_el]，物理俯仰TX位置
    pos_rx_el = cfg.el_rx_pos(:)' * lambda;  % [1 × Nrx_el]

    numTargets = size(targets, 1);

    % RX 在水平面排列，不提供俯仰维空间采样
    % 俯仰信息来自TX10~12的垂直间距 + 借用方位TX1（俯仰位置=0λ）

    %% ====== 快时间轴 ======
    t_fast = (0:Ns-1).' / fs;  % [Ns × 1]

    %% ====== 初始化输出 ======
    % 总 TX 数 = 方位 TX + 俯仰 TX = 9 + 3 = 12
    Ntx_total = Ntx + Ntx_el;
    if_signal = zeros(Ns, numLoops, Nrx, Ntx_total);

    %% ====== 方位TX信号生成（TX 1~9）======
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

    %% ====== 俯仰TX信号生成（TX 10~12）======
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
end

%% ========== 子函数：输入校验 ==========
function validate_inputs(cfg, targets)
% validate_inputs  校验输入参数合法性
%
%   参数：
%     cfg     - 配置结构体
%     targets - 目标矩阵

    % cfg 必须为结构体
    if ~isstruct(cfg)
        error('fmcw_sim_signal:invalidCfg', 'cfg 必须为结构体');
    end

    % 检查 cfg 必要字段
    req_fields = {'c', 'lambda', 'mu', 'fs', 'Ns', 'Ntx', 'Nrx', ...
                  'Tp', 'numLoops', 'az_tx_pos', 'az_rx_pos', 'Tchirp'};
    for i = 1:length(req_fields)
        if ~isfield(cfg, req_fields{i})
            error('fmcw_sim_signal:missingField', ...
                  'cfg 缺少必要字段: %s', req_fields{i});
        end
    end

    % targets 必须为 N×4 或 N×5 矩阵
    if ~isnumeric(targets) || size(targets, 2) < 4
        error('fmcw_sim_signal:invalidTargets', ...
              'targets 必须为 N×4 [R,v,θ_az,RCS] 或 N×5 [R,v,θ_az,θ_el,RCS] 矩阵');
    end

    % 距离必须为正
    if any(targets(:, 1) <= 0)
        error('fmcw_sim_signal:invalidRange', ...
              '目标距离必须为正值');
    end

    % RCS 必须为正（N×4格式在第4列，N×5格式在第5列）
    rcs_col = min(size(targets, 2), 4) + (size(targets, 2) >= 5);
    if any(targets(:, rcs_col) <= 0)
        error('fmcw_sim_signal:invalidRCS', ...
              '目标 RCS 必须为正值');
    end
end
