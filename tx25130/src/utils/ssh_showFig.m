

%% 保存所有图像
% 在脚本末尾加入

%  遍历所有打开的图形窗口，逐个保存为 PNG 文件
%  saveas 将图形按屏幕分辨率导出，便于快速查看
%  如需更高画质（如论文发表），可将 saveas 替换为 exportgraphics(gcf, ...)
figHandles = flip(findall(0, 'Type', 'figure'));
for i = 1:length(figHandles)
    figure(figHandles(i));
    saveas(gcf, sprintf('figure_%d.png', i));
end
disp('图像已保存');