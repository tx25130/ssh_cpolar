function if_signal = fmcw_sim_clutter(cfg, if_signal)
% fmcw_sim_clutter  杂波生成与叠加
%
%   if_signal = fmcw_sim_clutter(cfg, if_signal)
%
%   输入：
%     cfg       - fmcw_sim_config() 返回的参数结构体
%     if_signal - 差频信号 [Ns × Nchirp_per_tx × Nrx × Ntx]
%
%   输出：
%     if_signal - 叠加杂波后的差频信号（维度不变）
%
%   支持杂波分布：
%     'Rayleigh'  - 复高斯散斑（默认）
%     'Weibull'   - ZMNL 变换，形状参数 cfg.clutter_shape
%     'K'         - Gamma 纹理 × 复高斯散斑（SIRP 模型）
%
%   杂波特性：
%     - 高斯多普勒谱塑形（频域滤波）
%     - CNR 功率控制

    %% ====== 开关控制 ======
    if ~cfg.enable_clutter
        return;
    end

    %% ====== 参数提取 ======
    dims      = size(if_signal);
    Ns        = dims(1);
    numLoops  = dims(2);
    Nrx       = dims(3);
    Ntx       = dims(4);
    lambda    = cfg.lambda;
    clutter_dist = cfg.clutter_dist;
    clutter_power = cfg.clutter_power;
    clutter_shape = cfg.clutter_shape;
    sigma_fd  = cfg.clutter_doppler_std;  % 杂波多普勒谱宽 (m/s)

    %% ====== 生成杂波（逐通道） ======
    clutter = zeros(dims);

    for tx = 1:Ntx
        for rx = 1:Nrx
            % 生成复高斯散斑分量
            speckle = generate_speckle(Ns, numLoops, lambda, sigma_fd, cfg);

            % 按杂波分布调制幅度
            switch upper(clutter_dist)
                case 'RAYLEIGH'
                    % Rayleigh = 复高斯，幅度已为 Rayleigh 分布
                    clutter(:, :, rx, tx) = speckle;

                case 'WEIBULL'
                    % ZMNL 变换：|高斯| → Weibull
                    clutter(:, :, rx, tx) = apply_weibull_zmnl(speckle, clutter_shape);

                case 'K'
                    % SIRP 模型：Gamma 纹理 × 复高斯散斑
                    nu = clutter_shape;  % 形状参数
                    texture = gamrnd(nu, 1/nu, 1, numLoops);
                    texture_3d = repmat(texture, Ns, 1);
                    clutter(:, :, rx, tx) = sqrt(texture_3d) .* speckle;

                otherwise
                    error('fmcw_sim_clutter:unknownDist', ...
                          '未知杂波分布类型: %s', clutter_dist);
            end
        end
    end

    %% ====== CNR 功率控制 ======
    sig_power = mean(abs(if_signal(:)).^2);
    if sig_power > 0
        clutter_current = mean(abs(clutter(:)).^2);
        if clutter_current > 0
            % 缩放使杂波功率达到目标值
            scale = sqrt(clutter_power / clutter_current);
            clutter = clutter * scale;
        end
    end

    %% ====== 叠加杂波 ======
    if_signal = if_signal + clutter;
end

%% ========== 子函数：复高斯散斑 + 多普勒谱塑形 ==========
function speckle = generate_speckle(Ns, numLoops, lambda, sigma_fd, cfg)
% generate_speckle  生成复高斯散斑并施加高斯多普勒谱塑形
%
%   参数：
%     Ns       - 快时间点数
%     numLoops - 慢时间 chirp 数
%     lambda   - 波长 (m)
%     sigma_fd - 杂波多普勒谱宽 (m/s)
%     cfg      - 配置结构体
%
%   返回：
%     speckle - [Ns × numLoops] 复高斯散斑（已塑形）

    % 生成独立复高斯噪声
    speckle = (randn(Ns, numLoops) + 1j * randn(Ns, numLoops)) / sqrt(2);

    % 多普勒谱塑形（频域滤波）
    if sigma_fd > 0
        % 多普勒频率轴
        fd_axis = fftfreq(numLoops, 1 / cfg.Tp);

        % 高斯功率谱: S(f) = exp(-f^2 / (2*sigma_f^2))
        % sigma_fd (m/s) 转换为频率: sigma_f = 2*sigma_fd/lambda
        sigma_f = 2 * sigma_fd / lambda;
        H_doppler = exp(-fd_axis.^2 / (2 * sigma_f^2));  % [numLoops × 1]
        H_doppler = H_doppler.';  % 转为行向量 [1 × numLoops]，匹配 fft 输出

        % 频域滤波（沿慢时间维）
        for n = 1:Ns
            X = fft(speckle(n, :));
            X = X .* H_doppler;
            speckle(n, :) = ifft(X);
        end
    end
end

%% ========== 子函数：Weibull ZMNL 变换 ==========
function out = apply_weibull_zmnl(speckle, k)
% apply_weibull_zmnl  通过 ZMNL 变换将 Rayleigh 幅度转为 Weibull 分布
%
%   参数：
%     speckle - 复高斯散斑 [Ns × numLoops]
%     k       - Weibull 形状参数
%
%   返回：
%     out - Weibull 分布杂波（保持原始相位）
%
%   ZMNL 变换流程：
%     1. 取 Rayleigh 幅度 |z|
%     2. 转为均匀分布 U = 1 - exp(-|z|^2/2)（CDF 逆变换）
%     3. 逆 Weibull CDF: W = (-ln(1-U))^(1/k)
%     4. 保持原始相位，用 W 替换 |z|

    amp = abs(speckle);
    phase = angle(speckle);

    % Rayleigh → 均匀分布
    U = 1 - exp(-amp.^2 / 2);

    % 均匀 → Weibull（逆 CDF）
    w_amp = (-log(1 - U)).^(1/k);

    % 保持原始相位
    out = w_amp .* exp(1j * phase);
end

%% ========== 子函数：FFT 频率轴 ==========
function f = fftfreq(n, d)
% fftfreq  计算 FFT 频率轴（类似 numpy.fft.fftfreq）
%
%   参数：
%     n - FFT 点数
%     d - 采样间隔
%
%   返回：
%     f - 频率轴 [n × 1]

    if mod(n, 2) == 0
        f = [0:(n/2-1), (-n/2):(-1)]' / (n * d);
    else
        f = [0:((n-1)/2), (-(n-1)/2):(-1)]' / (n * d);
    end
end
