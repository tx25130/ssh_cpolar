



%% CPU-串行和并行对比
clc;clear;close all;

%% 启动并行池
% 多核CPU启动-并行
p = parpool("Processes"); %集群计算
% p = parpool("Threads"); %数据交换


%% 等待用户确认
next = input('输入 1 继续执行，输入 0 退出: ');

if next == 1
    %% 测试1: 常规 for 循环
    a = randn(1, 1000);
    tic
    % 修复: for 循环需要指定范围 1:1000，而非单个数字
    for i = 1:1000
        a(i) = a(i)^2;
    end
    cpu_time = toc;
    fprintf('常规 for 循环时间: %.10f 秒\n', cpu_time);

    %% 测试2: 并行 parfor 循环
    a = randn(1, 1000);
    tic
    % 修复: parfor 同样需要指定范围 1:1000
    parfor i = 1:1000
        a(i) = a(i)^2;
    end
    par_time = toc;  % 修复: 变量名改为 par_time（原 gpu_time 命名错误）
    fprintf('并行 parfor 时间: %.10f 秒\n', par_time);
    
    %% 性能对比
    if par_time > 0
        speedup = cpu_time / par_time;
        fprintf('加速比: %.2fx\n', speedup);
    end
else
    disp('用户取消执行');
end

%% 清理并行池
delete(p);
disp('并行池已关闭');
