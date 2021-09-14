function LoadCase()
global params
params.x1_init = -13;
params.y1_init = -23;
params.theta1_init = 0;
params.theta2_init = 0;
params.theta3_init = 0;
params.theta4_init = 0;
params.x1_end_norm = 22;
params.y1_end_norm = -23;
params.theta1_end_norm = 0;
params.theta2_end_norm = 0;
params.theta3_end_norm = 0;
params.theta4_end_norm = 0;

obstacle_cell = cell(1, 3);
elem.x = [-10 -10 -50 -50 -10];
elem.y = [-20 30 30 -20 -20];
obstacle_cell{1, 1} = elem;
elem.x = [-2 -4 -4 -2 -2];
elem.y = [0 0 -50 -50 0];
obstacle_cell{1, 2} = elem;
elem.x = [4 50 50 4 4];
elem.y = [-20 -20 30 30 -20];
obstacle_cell{1, 3} = elem;
elem.x = [-4 -2 -2 -4 -4];
elem.y = [5 5 25 25 5];
obstacle_cell{1, 4} = elem;
params.obs = obstacle_cell;
params.Nobs = 4;
end