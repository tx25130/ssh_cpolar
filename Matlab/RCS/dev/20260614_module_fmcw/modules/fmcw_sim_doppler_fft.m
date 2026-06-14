function [rd_data, cfg] = fmcw_sim_doppler_fft(cfg, mti_data)
% fmcw_sim_doppler_fft  慢时间维 Hamming 窗 + FFT + fftshift
%
%   [rd_data, cfg] = fmcw_sim_doppler_fft(cfg, mti_data)
%
%   输入：
%     cfg       - fmcw_sim_config() 返回的参数结构体
%     mti_data  - MTI 处理后数据 [Ns × Ndop × Nrx × Ntx]
%
%   输出：
%     rd_data - R-D 谱数据 [Ns × Ndop × Nrx × Ntx]
%     cfg     - 更新后的配置（含 TDM 速度轴标定）
%
%   处理流程：
%     1. 构造 Hamming 窗
%     2. 沿慢时间维（第2维）加窗
%     3. 沿慢时间维做 FFT
%     4. fftshift 使零速居中
%     5. 按 TDM 体制计算速度轴

    %% ====== 参数提取 ======
    dims = size(mti_data);
    Ns = dims(1);
    Ndop = dims(2);
    Nrx = dims(3);
    Ntx = dims(4);

    %% ====== 构造 Hamming 窗 ======
    win = hamming(Ndop);

    %% ====== 加窗 + FFT ======
    % 将窗扩展到多维
    win_4d = reshape(win, [1, Ndop, 1, 1]);
    mti_data_win = mti_data .* win_4d;

    % 沿第2维做 FFT
    rd_data = fft(mti_data_win, Ndop, 2);

    %% ====== fftshift ======
    rd_data = fftshift(rd_data, 2);

    %% ====== TDM 速度轴标定 ======
    % TDM-MIMO 有效 PRT = Ntx * Tperiod
    % 速度分辨率 = lambda / (2 * Ndop * Ntx * Tp)
    Vres_tdm = cfg.lambda / (2 * Ndop * Ntx * cfg.Tp);
    V_axis_tdm = (-Ndop/2:Ndop/2-1) * Vres_tdm;

    cfg.Vres_tdm = Vres_tdm;
    cfg.V_axis_tdm = V_axis_tdm;
    cfg.Ndop = Ndop;
end
