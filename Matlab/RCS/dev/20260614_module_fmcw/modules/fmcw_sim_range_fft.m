function [range_fft, cfg] = fmcw_sim_range_fft(cfg, if_signal)
% fmcw_sim_range_fft  快时间维 Hamming 窗 + FFT
%
%   [range_fft, cfg] = fmcw_sim_range_fft(cfg, if_signal)
%
%   输入：
%     cfg       - fmcw_sim_config() 返回的参数结构体
%     if_signal - 差频信号 [Ns × Nchirp_per_tx × Nrx × Ntx]
%
%   输出：
%     range_fft - 距离 FFT 结果 [Ns × Nchirp_per_tx × Nrx × Ntx]
%     cfg       - 更新后的配置（含距离轴标定）
%
%   处理流程：
%     1. 构造 Hamming 窗
%     2. 沿快时间维（第1维）加窗
%     3. 沿快时间维做 FFT
%     4. 生成距离轴

    %% ====== 参数提取 ======
    Ns = cfg.Ns;
    dims = size(if_signal);

    %% ====== 构造 Hamming 窗 ======
    win = hamming(Ns);

    %% ====== 加窗 + FFT ======
    range_fft = zeros(dims);

    % 将窗扩展到多维
    % if_signal 维度: [Ns, numLoops, Nrx, Ntx]
    win_4d = reshape(win, [Ns, 1, 1, 1]);
    if_signal_win = if_signal .* win_4d;

    % 沿第1维做 FFT
    range_fft = fft(if_signal_win, Ns, 1);

    %% ====== 距离轴标定 ======
    cfg.range_axis = (0:Ns-1) * cfg.Rres;  % 单位：m
end
