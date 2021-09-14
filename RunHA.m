% 《自动驾驶决策规划技术理论与实践》书籍配套代码
%  Copyright (C) 2021 Bai Li
%  2021.08.07
% ==============================================================================
%  第9章.多体拖挂车的决策规划方法
% ==============================================================================
%  备注：部分代码不开源，本代码只用于辅助理解基于采样搜索方法的多体车辆路径生成过程
%  基于该部分代码的研究成果建议引用以下参考文献：
%  a) Bai Li, Tankut Acarman, Youmin Zhang, et al., “Tractor-trailer
%  vehicle trajectory planning in narrow environments with a progressively
%  constrained optimal control approach,” IEEE Transactions on Intelligent
%  Vehicles, 5(3), 414-425, 2020.
%  b) Hangjie Cen, Bai Li, Tankut Acarman, et al., "Optimization-based
%  maneuver planning for a tractor-trailer vehicle in complex environments
%  with safe travel corridors", 2021 32nd IEEE Intelligent Vehicles
%  Symposium (IV), 974-979, 2021.
% ==============================================================================

clear all;
close all;
clc;

LoadCase();
InitializeParams();
DrawBackgroundforHA();
SearchGuidingPath();
SearchTrajViaHybridAstar();