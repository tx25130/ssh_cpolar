%% surface_scattering_model.m
% =========================================================================
% 复现论文: "Road-Condition Recognition Using 24-GHz Automotive Radar"
%   Ville V. Viikari, Timo Varpula, and Mikko Kantanen
%   IEEE Trans. Intell. Transp. Syst., 2009
%
% 功能: 基于标准一阶 Bragg SPM（小扰动模型）计算干燥/湿润
%       沥青表面的后向散射系数，复现论文 Figure 12 的理论曲线
%
% 公式说明:
%   论文 IV-A 节给出 SPM 公式(9)-(18)。其中公式(9)的 |I^n_pp| 按
%   量纲分析应为 |I^n_pp|^2（后向散射系数为功率比），标准 SPM
%   (Fung 1994, Eq.2.137) 使用平方形式。本文统一使用标准一阶
%   Bragg SPM（n=1），该模型在微波遥感领域广泛验证。
%
%   对于 HH 极化，F_pp 修正项（论文公式14）在非磁性高介电常数
%   表面的大入射角处发散，因此不计 F_pp 项。
%
% 参考文献:
%   [21] A. K. Fung, Microwave Scattering and Emission Models
%        and Their Applications. Boston: Artech House, 1994.
%   Ulaby, Moore & Fung, Microwave Remote Sensing, Vol.II, 1982.
%
% 环境要求: MATLAB R2020a+, Signal Processing Toolbox
% =========================================================================
clear; close all; clc;

%% ========================================================================
%  1. 雷达系统参数（论文 Section V）
% =========================================================================
freq = 24e9;           % 雷达工作频率 (Hz)
c0   = 3e8;            % 光速 (m/s)
lambda = c0 / freq;    % 波长 (m)，24 GHz → 12.5 mm
k = 2 * pi / lambda;   % 波数 (rad/m)

%% ========================================================================
%  2. 沥青表面参数（论文 Table I）
%     双尺度高斯粗糙面，大尺度(L1) + 小尺度(L2)互不相关
%     参数由论文将模型拟合到实验室测量数据得到
% =========================================================================
% --- 粗糙度参数（原始单位 mm，见 Table I）---
gamma1_mm = 0.6545;    % 大尺度 RMS 高度 (mm)
L1_mm = 14.1840;       % 大尺度自相关长度 (mm)
gamma2_mm = 0.2018;    % 小尺度 RMS 高度 (mm)
L2_mm = 3.7632;        % 小尺度自相关长度 (mm)

% --- 转换为国际单位制 ---
gamma1 = gamma1_mm * 1e-3;  % 大尺度 RMS 高度 (m)
L1 = L1_mm * 1e-3;          % 大尺度相关长度 (m)
gamma2 = gamma2_mm * 1e-3;  % 小尺度 RMS 高度 (m)
L2 = L2_mm * 1e-3;          % 小尺度相关长度 (m)

%% ========================================================================
%  3. 材料电磁参数
%
%     干燥沥青: ε_r = 2.6
%       来源: 论文引用的参考文献[22][23]，
%       沥青在微波频段相对介电常数为 2.6，损耗角正切很小
%
%     湿润沥青: ε_r = 33
%       来源: 论文[24]，Meissner & Wentz (2004)
%       24 GHz、20°C 时纯水的相对介电常数为 33 - j36
%       由于试验中水层厚度(数毫米)远大于趋肤深度(0.77 mm)，
%       湿润沥青的等效介电常数近似取为水的介电常数
%
%     磁导率: μ_r = 1（所有材料为非磁性）
%
%     【选择依据】论文 IV-B 节明确给出上述参数和理由
% =========================================================================
epsilon_r_dry = 2.6;    % 干燥沥青相对介电常数
epsilon_r_wet = 33;     % 湿润沥青相对介电常数

% 注意: 材料损耗(ε_r 的虚部 j36)在本模型中未计入。
% 论文指出趋肤深度 0.77 mm << 水层厚度，损耗影响仅限于
% 透射波的衰减，对表面散射的 Bragg 共振效应影响较小。
% 标准 SPM 通常假设无耗或低耗介质。

mu_r = 1;               % 相对磁导率（非磁性）

%% ========================================================================
%  4. 入射角网格
% =========================================================================
phi_deg = (0:0.5:80).';  % 入射角 (deg)，相对表面法线
phi = deg2rad(phi_deg);  % 入射角 (rad)

%% ========================================================================
%  5. 计算后向散射系数
% =========================================================================

% 湿润沥青（论文 Figure 12 的目标）
[sigma_vv_wet, sigma_hh_wet] = bragg_spm_2scale( ...
    k, phi, gamma1, gamma2, L1, L2, epsilon_r_wet);

% 干燥沥青（参考对比）
[sigma_vv_dry, sigma_hh_dry] = bragg_spm_2scale( ...
    k, phi, gamma1, gamma2, L1, L2, epsilon_r_dry);

%% ========================================================================
%  6. 绘图：主图 + 极化比子图
% =========================================================================

% --- Figure 1: 后向散射系数（复现论文 Figure 12）---
h_fig1 = figure('Position', [100, 100, 900, 700], 'Color', 'w');

% 子图 (a): 后向散射系数 vs 入射角
subplot(2, 1, 1);
plot(phi_deg, 10*log10(sigma_vv_wet), 'b-', 'LineWidth', 2.5); hold on;
plot(phi_deg, 10*log10(sigma_hh_wet), 'r-', 'LineWidth', 2.5);
plot(phi_deg, 10*log10(sigma_vv_dry), 'b--', 'LineWidth', 1.5);
plot(phi_deg, 10*log10(sigma_hh_dry), 'r--', 'LineWidth', 1.5);
xlabel('Incidence angle \phi (deg)', 'FontSize', 13);
ylabel('\sigma^0 (dB)', 'FontSize', 13);
title(['SPM Backscattering Coefficients at ', ...
    num2str(freq/1e9), ' GHz'], 'FontSize', 14, 'FontWeight', 'bold');
legend({...
    ['\sigma^0_{vv} wet (\epsilon_r=', num2str(epsilon_r_wet), ')'], ...
    ['\sigma^0_{hh} wet (\epsilon_r=', num2str(epsilon_r_wet), ')'], ...
    ['\sigma^0_{vv} dry (\epsilon_r=', num2str(epsilon_r_dry), ')'], ...
    ['\sigma^0_{hh} dry (\epsilon_r=', num2str(epsilon_r_dry), ')']}, ...
    'Location', 'southwest', 'FontSize', 11);
grid on; set(gca, 'FontSize', 12, 'GridAlpha', 0.3);
xlim([0, 80]); ylim([-55, 10]);

% 子图 (b): 极化比 σ_vv/σ_hh（论文 Figure 13 的参考）
subplot(2, 1, 2);
ratio_wet = 10*log10(sigma_vv_wet ./ sigma_hh_wet);
ratio_dry = 10*log10(sigma_vv_dry ./ sigma_hh_dry);

plot(phi_deg, ratio_wet, 'b-', 'LineWidth', 2.5); hold on;
plot(phi_deg, ratio_dry, 'b--', 'LineWidth', 1.5);

% 标注论文关键数据点
plot(70, ratio_wet(phi_deg == 70), 'ro', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'r');
text(72, ratio_wet(phi_deg == 70) + 1, ...
    [num2str(round(ratio_wet(phi_deg == 70), 1)), ' dB @ 70°'], ...
    'FontSize', 11, 'Color', 'r');

xlabel('Incidence angle \phi (deg)', 'FontSize', 13);
ylabel('\sigma^0_{vv} / \sigma^0_{hh} (dB)', 'FontSize', 13);
title('Polarization Ratio', 'FontSize', 14, 'FontWeight', 'bold');
legend({['Wet (\epsilon_r=', num2str(epsilon_r_wet), ')'], ...
        ['Dry (\epsilon_r=', num2str(epsilon_r_dry), ')']}, ...
    'Location', 'northwest', 'FontSize', 11);
grid on; set(gca, 'FontSize', 12, 'GridAlpha', 0.3);
xlim([0, 80]);

% 表面参数标注
sgtitle(sprintf(['Surface parameters: \\gamma_1=%.4g mm, ', ...
    'L_1=%.4g mm, \\gamma_2=%.4g mm, L_2=%.4g mm'], ...
    gamma1_mm, L1_mm, gamma2_mm, L2_mm), ...
    'FontSize', 11);

% 保存图像
saveas(h_fig1, fullfile(fileparts(mfilename('fullpath')), ...
    'fig_backscattering_coefficients.png'));

%% ========================================================================
%  7. 数据输出
% =========================================================================

fprintf('\n========== 湿润沥青 (ε_r=%d) 后向散射系数 ==========\n', ...
    epsilon_r_wet);
fprintf('  φ(°)    σ°_vv(dB)    σ°_hh(dB)    ratio(dB)\n');
fprintf('-----------------------------------------------\n');
check_angles = [0, 10, 20, 30, 40, 50, 60, 65, 70, 75, 80];
for i = 1:length(check_angles)
    idx = find(abs(phi_deg - check_angles(i)) < 0.1, 1);
    if ~isempty(idx)
        fprintf('  %2d      %8.2f      %8.2f      %8.2f\n', ...
            check_angles(i), ...
            10*log10(sigma_vv_wet(idx)), ...
            10*log10(sigma_hh_wet(idx)), ...
            10*log10(sigma_vv_wet(idx)/sigma_hh_wet(idx)));
    end
end

fprintf('\n========== 干燥沥青 (ε_r=%.1f) 后向散射系数 ==========\n', ...
    epsilon_r_dry);
fprintf('  φ(°)    σ°_vv(dB)    σ°_hh(dB)    ratio(dB)\n');
fprintf('-----------------------------------------------\n');
for i = 1:length(check_angles)
    idx = find(abs(phi_deg - check_angles(i)) < 0.1, 1);
    if ~isempty(idx)
        fprintf('  %2d      %8.2f      %8.2f      %8.2f\n', ...
            check_angles(i), ...
            10*log10(sigma_vv_dry(idx)), ...
            10*log10(sigma_hh_dry(idx)), ...
            10*log10(sigma_vv_dry()/sigma_hh_dry(idx)));
    end
end

%% ========================================================================
%  8. 与论文实测数据的对比验证
% =========================================================================
fprintf('\n========== 验证：与论文 Figure 2-3 实测数据对比 ==========\n');
fprintf('  在 65° 入射角:\n');
fprintf('  湿润沥青 σ°_vv:  模型 %.1f dB vs 实测 ≈ -20~-22 dB\n', ...
    10*log10(sigma_vv_wet(phi_deg == 65)));
fprintf('  湿润沥青 σ°_hh:  模型 %.1f dB vs 实测 ≈ -35~-38 dB\n', ...
    10*log10(sigma_hh_wet(phi_deg == 65)));
fprintf('  在 70° 极化比:   模型 %.1f dB vs 论文预测 ~17 dB\n', ...
    10*log10(sigma_vv_wet(phi_deg == 70)/sigma_hh_wet(phi_deg == 70)));

%% ========================================================================
%  辅助函数：标准一阶 Bragg SPM
% =========================================================================

function [sigma_vv, sigma_hh] = bragg_spm_2scale( ...
    k, phi, gamma1, gamma2, L1, L2, epsilon_r)
% BRAGG_SPM_2SCALE  标准一阶 Bragg SPM 后向散射系数
%
% 公式:
%   σ⁰_pp = 8·k⁴·γ²·cos⁴φ·|α_pp|²·W_s(K)·exp(-2k_z²γ²)
%
%   W_s(K) = a·(L₁²/2)·exp(-(KL₁)²/4) + b·(L₂²/2)·exp(-(KL₂)²/4)
%
%   α_vv = (ε_r-1)[sin²φ - ε_r(1+sin²φ)] / [ε_r cosφ + √(ε_r-sin²φ)]²
%   α_hh = (ε_r-1) / [cosφ + √(ε_r-sin²φ)]²
%
% 输入:
%   k          - 波数 (rad/m)
%   phi        - 入射角 (rad)，列向量
%   gamma1, L1 - 大尺度 RMS 高度 (m), 相关长度 (m)
%   gamma2, L2 - 小尺度 RMS 高度 (m), 相关长度 (m)
%   epsilon_r  - 相对介电常数（标量）
%
% 输出:
%   sigma_vv   - VV 极化后向散射系数（线性值，无量纲）
%   sigma_hh   - HH 极化后向散射系数（线性值，无量纲）
%
% 参考文献:
%   Fung (1994), Eqs. 2.112-2.137
%   Ulaby et al. (1982), Microwave Remote Sensing, Vol.II
% =========================================================================

% --- 总 RMS 高度与双尺度权重 ---
gamma_sq = gamma1^2 + gamma2^2;  % 总方差 (m²)
gamma = sqrt(gamma_sq);          % 总 RMS 高度 (m)
a = gamma1^2 / gamma_sq;         % 大尺度归一化权重
b = gamma2^2 / gamma_sq;         % 小尺度归一化权重

% --- 角度相关量 ---
cos_phi = cos(phi);
sin_phi = sin(phi);
sin2_phi = sin_phi.^2;

k_z = k * cos_phi;  % 垂直波数 (rad/m)
k_x = k * sin_phi;  % 水平波数 (rad/m)

% 后向散射的空间波数 K = -2k_x（论文公式 9 的 W^(n)(-2k_x)）
K_val = -2 * k_x;

% --- SPM 指数衰减因子（论文公式 9）---
% exp(-2k_z²γ²): 由表面粗糙度引起的散射功率衰减
exp_factor = exp(-2 * k_z.^2 * gamma_sq);

% --- Fresnel 反射系数（论文公式 15-16）---
% 注意: 对非磁性材料(μ_r=1)，R_hh ≠ R_vv
sqrt_term = sqrt(epsilon_r - sin2_phi);
R_hh = (epsilon_r * cos_phi - sqrt_term) ./ ...
       (epsilon_r * cos_phi + sqrt_term);
R_vv = (cos_phi - sqrt_term) ./ ...
       (cos_phi + sqrt_term);

% --- 标准 Bragg 极化系数 α_pp ---
% 来源: Fung (1994); Ulaby et al. (1982, Vol.II, p.957)
eps_minus_1 = epsilon_r - 1;

% VV 极化 Bragg 系数
% α_vv = (ε_r-1)[sin²φ - ε_r(1+sin²φ)] / [ε_r cosφ + √(ε_r-sin²φ)]²
denom_vv = epsilon_r * cos_phi + sqrt_term;
alpha_vv = eps_minus_1 .* (sin2_phi - epsilon_r * (1 + sin2_phi)) ...
           ./ (denom_vv.^2);

% HH 极化 Bragg 系数
% α_hh = (ε_r-1) / [cosφ + √(ε_r-sin²φ)]²
denom_hh = cos_phi + sqrt_term;
alpha_hh = eps_minus_1 ./ (denom_hh.^2);

% --- 双尺度表面功率谱 W_s(K) = W^(1)(K) ---
% 论文公式(17-18) n=1 时:
% W^(1)(K) = Σ_{m=0}^{1} nC1_m a^(1-m) b^m (L²/2) exp(-(KL)²/4)
% m=0: L = L₁ (大尺度); m=1: L = L₂ (小尺度)
W_s = a * (L1^2 / 2) .* exp(-(K_val * L1).^2 / 4) + ...
      b * (L2^2 / 2) .* exp(-(K_val * L2).^2 / 4);

% --- 一阶 Bragg SPM 后向散射系数 ---
% σ⁰_pp = (k²/2)·e^(-2k_z²γ²)·|I¹_pp|²·W^(1)/1!
% 其中 |I¹_pp|² = 4k_z²γ²·|f_pp|²（取 n=1 的主导项）
% 展开为: σ⁰_pp = 8k⁴γ²·cos⁴φ·|α_pp|²·W_s·exp(-2k_z²γ²)
prefactor = 8 * k^4 * gamma_sq * cos_phi.^4;

sigma_vv = prefactor .* abs(alpha_vv).^2 .* W_s .* exp_factor;
sigma_hh = prefactor .* abs(alpha_hh).^2 .* W_s .* exp_factor;

% --- SPM 有效性校验（论文公式 19）---
% 条件: (k·γ) · (k·L_eq) < 1.2·√ε_r
% 等效相关长度: 双尺度 RMS 加权
L_eq = sqrt(gamma1^2 * L1^2 + gamma2^2 * L2^2) / gamma;
validity = (k * gamma) * (k * L_eq);
limit = 1.2 * sqrt(epsilon_r);

persistent already_checked
if isempty(already_checked)
    already_checked = true;
    if validity < limit
        fprintf(['[OK] SPM validity: (kγ)*(kL_eq) = %.3f < ', ...
                 '1.2√ε_r = %.3f\n'], validity, limit);
    else
        warning(['SPM validity marginal: (kγ)*(kL_eq) = %.3f, ', ...
                 'limit = %.3f'], validity, limit);
    end
end

end
