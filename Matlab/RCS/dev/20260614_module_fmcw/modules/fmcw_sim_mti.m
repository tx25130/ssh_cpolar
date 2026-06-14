function [mti_data, cfg] = fmcw_sim_mti(cfg, range_fft)
% fmcw_sim_mti  MTI 两脉冲对消处理
%
%   [mti_data, cfg] = fmcw_sim_mti(cfg, range_fft)
%
%   输入：
%     cfg       - fmcw_sim_config() 返回的参数结构体
%     range_fft - Range-FFT 结果 [Ns × Nchirp_per_tx × Nrx × Ntx]
%
%   输出：
%     mti_data - MTI 对消后数据 [Ns × (Nchirp_per_tx-1) × Nrx × Ntx]
%     cfg      - 更新后的配置（含更新后的速度轴和分辨率）
%
%   处理流程：
%     1. 若 MTI 禁用，直接返回原数据
%     2. 沿慢时间维做两脉冲对消: y[n] = x[n] - x[n-1]
%     3. 更新速度轴和速度分辨率

    %% ====== 开关控制 ======
    if ~cfg.enable_MTI
        mti_data = range_fft;
        return;
    end

    %% ====== 参数提取 ======
    dims = size(range_fft);
    Ns = dims(1);
    numLoops = dims(2);
    Nrx = dims(3);
    Ntx = dims(4);

    %% ====== 两脉冲对消 ======
    % y[n] = x[n] - x[n-1], n = 2:N
    mti_data = range_fft(:, 2:numLoops, :, :);

    for k = 2:numLoops
        mti_data(:, k-1, :, :) = range_fft(:, k, :, :) - range_fft(:, k-1, :, :);
    end

    %% ====== 更新速度轴和分辨率 ======
    numLoops_new = numLoops - 1;
    cfg.numLoops_mti = numLoops_new;

    % MTI 后速度分辨率
    cfg.Vres_mti = cfg.lambda / (2 * numLoops_new * cfg.Tp);

    % MTI 后速度轴（用于后续 Doppler-FFT）
    cfg.V_axis_mti = (-1) * ((-numLoops_new/2:numLoops_new/2-1) * cfg.Vres_mti);
end
