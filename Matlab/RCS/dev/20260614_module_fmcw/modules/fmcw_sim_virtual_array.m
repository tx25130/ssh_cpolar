function [rd_virtual, rd_el_virtual, cfg] = fmcw_sim_virtual_array(cfg, rd_data)
% fmcw_sim_virtual_array  MIMO 虚拟阵列重组（方位 + 俯仰）
%
%   [rd_virtual, rd_el_virtual, cfg] = fmcw_sim_virtual_array(cfg, rd_data)
%
%   输入：
%     cfg      - fmcw_sim_config() 返回的参数结构体
%     rd_data  - R-D 谱数据 [Ns × Ndop × Nrx × Ntx_total]
%
%   输出：
%     rd_virtual      - 方位虚拟阵列 R-D 数据 [Ns × Ndop × Nv_az_unique]
%     rd_el_virtual   - 俯仰虚拟阵列 R-D 数据 [Ns × Ndop × 4]（借用TX1 + 3俯仰TX）
%     cfg             - 更新后的配置
%
%   俯仰虚拟阵列说明：
%     AWR2243级联中，RX在水平面排列（无俯仰空间采样），
%     俯仰信息来自TX10~12的垂直间距 + 借用方位TX1（俯仰位置=0λ），
%     形成4元虚拟阵列 [0, 0.5, 2, 3] λ。
%     俯仰快拍 = 借用TX1 + 3个俯仰TX，每个TX对应所有RX的相干平均，
%     保留各TX间的俯仰相位差用于DBF角度估计。

    %% ====== 参数提取 ======
    dims = size(rd_data);
    Ns   = dims(1);
    Ndop = dims(2);
    Nrx  = dims(3);
    Ntx_total = dims(4);

    lambda = cfg.lambda;
    Ntx_az = cfg.Ntx;       % 方位TX数（9）
    Ntx_el = cfg.Ntx_el;    % 俯仰TX数（3）

    % 方位天线位置（单位：lambda）
    pos_tx_az = cfg.az_tx_pos;
    pos_rx_az = cfg.az_rx_pos;

    % 俯仰TX位置（单位：lambda）
    pos_tx_el = cfg.el_tx_pos(:)';

    %% ====== 方位虚拟阵列重组（TX 1~Ntx_az, 所有RX） ======
    Nv_az = Ntx_az * Nrx;
    % 方位虚拟阵列：[tx, rx, lambda]
    % 记录每个虚拟阵元对应的tx和rx，以及lambda
    voxel_map_az = zeros(Nv_az, 3);
    virtual_az_lambda = zeros(1, Nv_az);

    idx = 0;
    for tx = 1:Ntx_az
        for rx = 1:Nrx
            idx = idx + 1;
            voxel_map_az(idx, 1) = tx;
            voxel_map_az(idx, 2) = rx;
            voxel_map_az(idx, 3) = pos_tx_az(tx) + pos_rx_az(rx);
            virtual_az_lambda(idx) = pos_tx_az(tx) + pos_rx_az(rx);
        end
    end

    %% ====== TDM 多普勒相位补偿 ======
    if isfield(cfg, 'V_axis_tdm')
        V_axis = cfg.V_axis_tdm;
    else
        V_axis = cfg.V_axis;
    end

    rd_data_comp = rd_data;

    for tx = 1:Ntx_total
        for d_idx = 1:Ndop
            v_est = V_axis(d_idx);
            dop_corr = exp(-1j * 4 * pi * v_est * (tx - 1) * cfg.Tp / lambda);
            rd_data_comp(:, d_idx, :, tx) = rd_data_comp(:, d_idx, :, tx) * dop_corr;
        end
    end

    %% ====== 方位虚拟阵列去重叠 ======
    [unique_az_lambda, ~, ic_az] = unique(virtual_az_lambda);
    Nv_az_unique = length(unique_az_lambda);

    rd_virtual = zeros(Ns, Ndop, Nv_az_unique);
    for v_idx = 1:Nv_az_unique
        match_idx = find(ic_az == v_idx);
        accum = zeros(Ns, Ndop);
        for m = 1:length(match_idx)
            tx = voxel_map_az(match_idx(m), 1);
            rx = voxel_map_az(match_idx(m), 2);
            accum = accum + rd_data_comp(:, :, rx, tx);
        end
        rd_virtual(:, :, v_idx) = accum / length(match_idx);
    end

    %% ====== 俯仰虚拟阵列（借用TX1 + TX10~12, 所有RX相干平均） ======
    % 借用方位TX1（俯仰位置=0λ）+ 3个俯仰TX → 4元虚拟阵列
    % 保留各TX间的俯仰相位差（编码俯仰角信息）
    % 对每个TX，所有RX信号相干平均提高SNR
    Nv_el_virtual = Ntx_el + 1;  % 4元（3俯仰TX + 1借用方位TX）
    rd_el_virtual = zeros(Ns, Ndop, Nv_el_virtual);

    % 第1个快拍：借用方位TX1（俯仰位置=0λ）
    borrowed_tx = cfg.el_borrowed_tx;  % 借用的方位TX索引
    accum = zeros(Ns, Ndop);
    for rx = 1:Nrx
        accum = accum + rd_data_comp(:, :, rx, borrowed_tx);
    end
    rd_el_virtual(:, :, 1) = accum / Nrx;

    % 第2~4个快拍：俯仰TX10~12（俯仰位置=0.5/2/3 λ）
    for el_idx = 1:Ntx_el
        global_tx = Ntx_az + el_idx;
        accum = zeros(Ns, Ndop);
        for rx = 1:Nrx
            accum = accum + rd_data_comp(:, :, rx, global_tx);
        end
        rd_el_virtual(:, :, el_idx + 1) = accum / Nrx;
    end

    %% ====== 更新配置 ======
    cfg.Nv_unique = Nv_az_unique;
    cfg.Nv_el_unique = Nv_el_virtual;  % 俯仰快拍数 = 4（3俯仰TX + 1借用方位TX）
    cfg.virtual_az_pos_unique = unique_az_lambda * lambda;
    cfg.virtual_el_pos_unique = cfg.d_el_tx_pos(:)' * lambda;  % 俯仰4元阵位置
end
