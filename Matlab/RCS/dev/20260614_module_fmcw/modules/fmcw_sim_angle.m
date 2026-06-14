function [estimates, cfg] = fmcw_sim_angle(cfg, rd_virtual, det_mask, rd_el_virtual)
% fmcw_sim_angle  角度估计模块（方位 + 俯仰）
%
%   [estimates, cfg] = fmcw_sim_angle(cfg, rd_virtual, det_mask, rd_el_virtual)
%
%   输入：
%     cfg            - fmcw_sim_config() 返回的参数结构体
%     rd_virtual     - 方位虚拟阵列 R-D 数据 [Ns × Ndop × Nv_az_unique]
%     det_mask       - CFAR 检测掩码 [Ns × Ndop]
%     rd_el_virtual  - 俯仰虚拟阵列 R-D 数据 [Ns × Ndop × Nv_el_unique]（可选）
%
%   输出：
%     estimates - 结构体数组，每点含 range, velocity, angle, elevation, snr_dB
%     cfg       - 更新后的配置

    %% ====== 参数提取 ======
    dims = size(rd_virtual);
    Nr  = dims(1);
    Nd  = dims(2);

    lambda = cfg.lambda;
    method = cfg.angle_method;

    % 方位虚拟阵元位置
    if isfield(cfg, 'virtual_az_pos_unique')
        virtual_az_pos = cfg.virtual_az_pos_unique;
    else
        virtual_az_pos = cfg.virtual_az_pos;
    end

    % 俯仰虚拟阵元位置
    has_elevation = (nargin >= 4) && ~isempty(rd_el_virtual);
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

    %% ====== 提取检测点索引 ======
    [det_r, det_d] = find(det_mask);
    num_det = length(det_r);

    if num_det == 0
        estimates = struct('range', {}, 'velocity', {}, 'angle', {}, ...
                           'elevation', {}, 'snr_dB', {});
        return;
    end

    %% ====== 逐检测点估计角度 ======
    estimates = struct('range', cell(1, num_det), ...
                       'velocity', cell(1, num_det), ...
                       'angle', cell(1, num_det), ...
                       'elevation', cell(1, num_det), ...
                       'snr_dB', cell(1, num_det));

    for i = 1:num_det
        r_idx = det_r(i);
        d_idx = det_d(i);

        % 方位虚拟阵列快拍
        x_az = rd_virtual(r_idx, d_idx, :);
        x_az = x_az(:);

        R_est = range_axis(r_idx);
        V_est = vel_axis(d_idx);

        % SNR 估计
        sig_pow = abs(x_az' * x_az) / length(x_az);
        estimates(i).range = R_est;
        estimates(i).velocity = V_est;
        estimates(i).snr_dB = 10 * log10(sig_pow + eps);

        % 方位角估计
        switch upper(method)
            case 'FFT'
                estimates(i).angle = angle_fft_method(x_az, virtual_az_pos, lambda, cfg);
            case 'DBF'
                estimates(i).angle = angle_dbf_method(x_az, virtual_az_pos, lambda, cfg);
            case 'MUSIC'
                estimates(i).angle = angle_music_method(x_az, virtual_az_pos, lambda, cfg);
            otherwise
                error('fmcw_sim_angle:unknownMethod', ...
                      '未知角度估计方法: %s', method);
        end

        % 俯仰角估计
        if has_elevation
            x_el = rd_el_virtual(r_idx, d_idx, :);
            x_el = x_el(:);
            estimates(i).elevation = angle_dbf_method(x_el, virtual_el_pos, lambda, cfg, 'el');
        else
            estimates(i).elevation = 0;
        end
    end
end

%% ========== 子函数：FFT 角度估计 ==========
function theta_est = angle_fft_method(x_snap, virtual_pos, lambda, cfg)
    Nfft = cfg.angle_fft_n;
    virtual_pos = virtual_pos(:);
    X = fftshift(fft(x_snap, Nfft));
    [~, peak_idx] = max(abs(X));
    sin_axis = (-Nfft/2:Nfft/2-1) / Nfft * 2;
    d_min = min(diff(sort(virtual_pos)));
    sin_theta = sin_axis(peak_idx) * lambda / (2 * d_min);

    % 抛物线插值精化
    if peak_idx > 1 && peak_idx < Nfft
        y1 = abs(X(peak_idx - 1));
        y2 = abs(X(peak_idx));
        y3 = abs(X(peak_idx + 1));
        delta = (y1 - y3) / (2 * (y1 - 2*y2 + y3) + eps);
        sin_theta = sin_theta + delta * (2 / Nfft) * lambda / (2 * d_min);
    end

    sin_theta = max(-1, min(1, sin_theta));
    theta_est = asind(sin_theta);
end

%% ========== 子函数：DBF 角度估计 ==========
function theta_est = angle_dbf_method(x_snap, virtual_pos, lambda, cfg, dim_type)
    % dim_type: 'az'(默认) 或 'el'
    if nargin < 5
        dim_type = 'az';
    end

    if strcmp(dim_type, 'el')
        scan_range = cfg.el_angle_range;
    else
        scan_range = cfg.az_angle_range;
    end

    P = length(scan_range);
    virtual_pos = virtual_pos(:);
    spectrum = zeros(1, P);
    for p = 1:P
        theta_rad = deg2rad(scan_range(p));
        a = exp(1j * 2 * pi * virtual_pos * sin(theta_rad) / lambda);
        spectrum(p) = abs(a' * x_snap)^2;
    end

    [~, peak_idx] = max(spectrum);
    if peak_idx > 1 && peak_idx < P
        y1 = spectrum(peak_idx - 1);
        y2 = spectrum(peak_idx);
        y3 = spectrum(peak_idx + 1);
        delta = (y1 - y3) / (2 * (y1 - 2*y2 + y3) + eps);
        theta_est = scan_range(peak_idx) + delta * (scan_range(2) - scan_range(1));
    else
        theta_est = scan_range(peak_idx);
    end
end

%% ========== 子函数：MUSIC 角度估计 ==========
function theta_est = angle_music_method(x_snap, virtual_pos, lambda, cfg)
    N = length(x_snap);
    M = cfg.music_nsig;
    virtual_pos = virtual_pos(:);
    Rxx = x_snap * x_snap';
    [V, D] = eig(Rxx);
    eigvals = real(diag(D));
    [eigvals, sort_idx] = sort(eigvals, 'ascend');
    V = V(:, sort_idx);
    Un = V(:, 1:N-M);
    az_scan = cfg.az_angle_range;
    P = length(az_scan);
    pseudo_spectrum = zeros(1, P);
    for p = 1:P
        theta_rad = deg2rad(az_scan(p));
        a = exp(1j * 2 * pi * virtual_pos * sin(theta_rad) / lambda);
        pseudo_spectrum(p) = 1 / (abs(a' * Un * Un' * a) + eps);
    end
    [~, peak_idx] = max(pseudo_spectrum);
    theta_est = az_scan(peak_idx);
end
