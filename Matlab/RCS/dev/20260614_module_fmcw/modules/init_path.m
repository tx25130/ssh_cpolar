function init_path()
% init_path  将模块目录添加到 MATLAB 搜索路径
%   init_path() 将当前脚本所在目录（modules/）添加到 MATLAB 搜索路径，
%   使所有模块函数可被主脚本直接调用。
%
%   用法：
%     cd('Matlab_1\RCS\dev\20260614\modules');
%     init_path();

    % 获取当前脚本所在目录
    modules_dir = fileparts(mfilename('fullpath'));

    % 检查是否已在路径中
    paths = path();
    if contains(paths, modules_dir)
        fprintf('模块目录已在搜索路径中: %s\n', modules_dir);
    else
        addpath(modules_dir);
        fprintf('已添加模块目录到搜索路径: %s\n', modules_dir);
    end
end
