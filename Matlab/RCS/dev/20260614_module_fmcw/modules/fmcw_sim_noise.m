function if_signal = fmcw_sim_noise(cfg, if_signal)
% fmcw_sim_noise  复高斯白噪声生成与叠加
%
%   if_signal = fmcw_sim_noise(cfg, if_signal)
%
%   输入：
%     cfg       - fmcw_sim_config() 返回的参数结构体
%     if_signal - 差频信号 [Ns × Nchirp_per_tx × Nrx × Ntx]
%
%   输出：
%     if_signal - 叠加噪声后的差频信号（维度不变）
%
%   噪声模型：
%     循环对称复高斯白噪声
%     实部和虚部独立同分布 N(0, noise_power/2)
%     噪声功率 = 信号功率 / 10^(SNR_dB/10)

    %% ====== 参数提取 ======
    SNR_dB = cfg.SNR_dB;
    dims   = size(if_signal);

    %% ====== 计算信号功率 ======
    sig_power = mean(abs(if_signal(:)).^2);

    if sig_power == 0
        % 纯信号为零时无法计算 SNR，使用单位功率
        noise_power = 1;
    else
        noise_power = sig_power / (10^(SNR_dB / 10));
    end

    %% ====== 生成复高斯白噪声 ======
    % 实部和虚部各方差为 noise_power/2
    sigma = sqrt(noise_power / 2);
    noise = sigma * (randn(dims) + 1j * randn(dims));

    %% ====== 叠加噪声 ======
    if_signal = if_signal + noise;
end
