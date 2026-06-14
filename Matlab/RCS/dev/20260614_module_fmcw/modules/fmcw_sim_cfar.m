function [det_mask, det_snr, cfg] = fmcw_sim_cfar(cfg, rd_virtual)
% fmcw_sim_cfar  二维 CA-CFAR 恒虚警检测
%
%   [det_mask, det_snr, cfg] = fmcw_sim_cfar(cfg, rd_virtual)
%
%   输入：
%     cfg         - fmcw_sim_config() 返回的参数结构体
%     rd_virtual  - 虚拟阵列 R-D 数据 [Ns × Ndop × Nv]
%
%   输出：
%     det_mask - 检测掩码 [Ns × Ndop]（布尔矩阵）
%     det_snr  - 检测 SNR [Ns × Ndop]
%     cfg      - 更新后的配置
%
%   处理流程：
%     1. 所有虚拟阵元功率非相干积累
%     2. 计算门限系数 alpha
%     3. 二维滑窗 CA-CFAR 检测
%     4. 边界跳过处理

    %% ====== 参数提取 ======
    Pfa = cfg.cfar2_Pfa;
    Tr  = cfg.cfar2_Tr;    % 距离维参考单元（单边）
    Td  = cfg.cfar2_Td;    % 多普勒维参考单元（单边）
    Gr  = cfg.cfar2_Gr;    % 距离维保护单元（单边）
    Gd  = cfg.cfar2_Gd;    % 多普勒维保护单元（单边）

    dims = size(rd_virtual);
    Nr  = dims(1);
    Nd  = dims(2);
    Nv  = dims(3);

    %% ====== 非相干积累 ======
    rd_power = zeros(Nr, Nd);
    for v = 1:Nv
        rd_power = rd_power + abs(rd_virtual(:, :, v)).^2;
    end
    rd_power = rd_power / Nv;

    %% ====== 计算门限系数 ======
    % 总单元数（含保护区域）
    N_total = (2*Tr + 2*Gr + 1) * (2*Td + 2*Gd + 1);
    % 保护单元数
    N_guard = (2*Gr + 1) * (2*Gd + 1);
    % 训练单元数
    N_train = N_total - N_guard;
    % CA-CFAR 门限系数
    alpha = N_train * (Pfa^(-1/N_train) - 1);

    %% ====== 二维 CA-CFAR 检测 ======
    det_mask = false(Nr, Nd);
    det_snr  = zeros(Nr, Nd);

    % 边界跳过范围
    r_start = Tr + Gr + 1;
    r_end   = Nr - Tr - Gr;
    d_start = Td + Gd + 1;
    d_end   = Nd - Td - Gd;

    for d_idx = d_start:d_end
        for r_idx = r_start:r_end
            % 当前检测单元功率
            CUT = rd_power(r_idx, d_idx);

            % 构建参考区域
            ref_region = rd_power(r_idx-Tr-Gr : r_idx+Tr+Gr, ...
                                  d_idx-Td-Gd : d_idx+Td+Gd);

            % 零化保护区域
            ref_copy = ref_region;
            r_center = Tr + Gr + 1;  % 中心在参考区域中的行索引
            d_center = Td + Gd + 1;  % 中心在参考区域中的列索引
            ref_copy(r_center-Gr:r_center+Gr, ...
                     d_center-Gd:d_center+Gd) = 0;

            % 计算训练单元均值（排除零化的保护区域）
            ref_sum = sum(ref_copy(:));
            ref_mean = ref_sum / N_train;

            % 计算检测阈值
            threshold = ref_mean * alpha;

            % 检测判决
            if CUT > threshold
                det_mask(r_idx, d_idx) = true;
                if ref_mean > 0
                    det_snr(r_idx, d_idx) = 10 * log10(CUT / ref_mean);
                end
            end
        end
    end

    %% ====== 更新配置 ======
    cfg.cfar_alpha = alpha;
    cfg.cfar_N_train = N_train;
end
