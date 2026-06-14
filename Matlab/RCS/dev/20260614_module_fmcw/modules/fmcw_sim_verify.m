function verify_result = fmcw_sim_verify(targets, estimates, cfg)
% fmcw_sim_verify  仿真验证模块（方位 + 俯仰）
%
%   verify_result = fmcw_sim_verify(targets, estimates, cfg)
%
%   输入：
%     targets   - 目标真值矩阵 [N×4] 或 [N×5]
%                 [R, v, θ_az, RCS] 或 [R, v, θ_az, θ_el, RCS]
%     estimates - 角度估计结果结构体数组（含 elevation 字段）
%     cfg       - 配置结构体
%
%   输出：
%     verify_result - 结构体，含 matched, errors, rmse, pass_fail

    %% ====== 阈值设定 ======
    R_TOL   = 1.0;   % 距离容差 (m)
    V_TOL   = 1.0;   % 速度容差 (m/s)
    AZ_TOL  = 5.0;   % 方位角容差 (°)
    EL_TOL  = 5.0;   % 俯仰角容差 (°)

    %% ====== 目标矩阵兼容处理 ======
    num_targets = size(targets, 1);
    if size(targets, 2) >= 5
        tgt_az = targets(:, 3);  % 方位角真值
        tgt_el = targets(:, 4);  % 俯仰角真值
    else
        tgt_az = targets(:, 3);
        tgt_el = zeros(num_targets, 1);  % 无俯仰角时默认0°
    end

    num_det = length(estimates);

    %% ====== 初始化结果 ======
    verify_result = struct();
    verify_result.matched = false(num_targets, 1);
    verify_result.match_idx = zeros(num_targets, 1);
    verify_result.errors = struct('range', cell(num_targets, 1), ...
                                  'velocity', cell(num_targets, 1), ...
                                  'angle', cell(num_targets, 1), ...
                                  'elevation', cell(num_targets, 1));
    verify_result.target_pass = false(num_targets, 1);

    %% ====== 最近邻匹配 ======
    for t = 1:num_targets
        tgt_R     = targets(t, 1);
        tgt_V     = targets(t, 2);
        tgt_theta = tgt_az(t);
        tgt_phi   = tgt_el(t);

        best_dist = inf;
        best_idx  = 0;

        for d = 1:num_det
            est_R     = estimates(d).range;
            est_V     = estimates(d).velocity;
            est_theta = estimates(d).angle;
            est_phi   = estimates(d).elevation;

            % 归一化欧几里得距离（含俯仰角）
            dist_R = abs(est_R - tgt_R) / R_TOL;
            dist_V = abs(est_V - tgt_V) / V_TOL;
            dist_A = abs(est_theta - tgt_theta) / AZ_TOL;
            dist_E = abs(est_phi - tgt_phi) / EL_TOL;
            dist_total = sqrt(dist_R^2 + dist_V^2 + dist_A^2 + dist_E^2);

            if dist_total < best_dist
                best_dist = dist_total;
                best_idx  = d;
            end
        end

        % 匹配判定
        if best_idx > 0 && best_dist < 2.0
            verify_result.matched(t) = true;
            verify_result.match_idx(t) = best_idx;

            est = estimates(best_idx);
            err_R = est.range - tgt_R;
            err_V = est.velocity - tgt_V;
            err_A = est.angle - tgt_theta;
            err_E = est.elevation - tgt_phi;

            verify_result.errors(t).range     = err_R;
            verify_result.errors(t).velocity  = err_V;
            verify_result.errors(t).angle     = err_A;
            verify_result.errors(t).elevation = err_E;

            % PASS/FAIL 判定
            if abs(err_R) <= R_TOL && abs(err_V) <= V_TOL && ...
               abs(err_A) <= AZ_TOL && abs(err_E) <= EL_TOL
                verify_result.target_pass(t) = true;
            end
        else
            verify_result.errors(t).range     = NaN;
            verify_result.errors(t).velocity  = NaN;
            verify_result.errors(t).angle     = NaN;
            verify_result.errors(t).elevation = NaN;
        end
    end

    %% ====== RMSE 统计 ======
    matched_idx = find(verify_result.matched);
    num_matched = length(matched_idx);

    if num_matched > 0
        err_R = cell2mat({verify_result.errors(matched_idx).range});
        err_V = cell2mat({verify_result.errors(matched_idx).velocity});
        err_A = cell2mat({verify_result.errors(matched_idx).angle});
        err_E = cell2mat({verify_result.errors(matched_idx).elevation});

        verify_result.rmse_range     = sqrt(mean(err_R.^2));
        verify_result.rmse_velocity  = sqrt(mean(err_V.^2));
        verify_result.rmse_angle     = sqrt(mean(err_A.^2));
        verify_result.rmse_elevation = sqrt(mean(err_E.^2));
    else
        verify_result.rmse_range     = NaN;
        verify_result.rmse_velocity  = NaN;
        verify_result.rmse_angle     = NaN;
        verify_result.rmse_elevation = NaN;
    end

    %% ====== 总体判定 ======
    all_matched = all(verify_result.matched);
    all_pass    = all(verify_result.target_pass);
    verify_result.all_pass = all_matched && all_pass;

    %% ====== 打印验证结果 ======
    fprintf('\n========== 仿真验证 ==========\n');
    for t = 1:num_targets
        if verify_result.matched(t)
            e = verify_result.errors(t);
            status = 'PASS';
            if ~verify_result.target_pass(t)
                status = 'FAIL';
            end
            fprintf('目标%d: R_err=%.3fm, V_err=%.3fm/s, Az_err=%.2f°, El_err=%.2f° [%s]\n', ...
                t, e.range, e.velocity, e.angle, e.elevation, status);
        else
            fprintf('目标%d: 未匹配 [FAIL]\n', t);
        end
    end

    if num_matched > 0
        fprintf('RMSE: R=%.3fm, V=%.3fm/s, Az=%.2f°, El=%.2f°\n', ...
            verify_result.rmse_range, verify_result.rmse_velocity, ...
            verify_result.rmse_angle, verify_result.rmse_elevation);
    end

    if verify_result.all_pass
        fprintf('仿真验证: 通过\n');
    else
        fprintf('仿真验证: 失败\n');
    end
    fprintf('================================\n');
end
