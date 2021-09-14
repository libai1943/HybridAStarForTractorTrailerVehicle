function InitializeParams()
global params
params.xmin = -25;
params.xmax = 25;
params.ymin = -25;
params.ymax = 25;
params.xhorizon = params.xmax - params.xmin;
params.yhorizon = params.ymax - params.ymin;

params.L_tractor_front_hang = 0.25;
params.L_tractor_wheelbase = 1.5;
params.L_tractor_rear_hang = 0.25;
params.L_half_width = 1;
params.vehicle_width = 2 * params.L_half_width;
%params.M = [1.5, 1.5, 1.5, 1.5];
params.M = [0, 0, 0, 0];
params.L = [3.0, 3.0, 3.0, 3.0];
params.L_trailer_front_hang = 1;
params.L_trailer_rear_hang = 1;
params.tractor_vehicle_length = params.L_tractor_wheelbase + params.L_tractor_front_hang + params.L_tractor_rear_hang;
params.trailer_vehicle_length = params.L_trailer_front_hang + params.L_trailer_rear_hang;
params.radius = max(sqrt(2) * params.L_half_width, hypot(params.L_half_width, 0.5 * params.tractor_vehicle_length));

params.v_max = 2.5;
params.phy_max = 0.7;
params.a_max = 0.25;
params.j_max = 0.1;
params.w_max = 0.5;

[params.x2_init, params.y2_init, params.x3_init, params.y3_init, params.x4_init, params.y4_init] = MapFrom6DimTo4Dim(params.x1_init, params.y1_init, params.theta1_init, params.theta2_init, params.theta3_init, params.theta4_init);
[params.x2_end_norm, params.y2_end_norm, params.x3_end_norm, params.y3_end_norm, params.x4_end_norm, params.y4_end_norm] = MapFrom6DimTo4Dim(params.x1_end_norm, params.y1_end_norm, params.theta1_end_norm, params.theta2_end_norm, params.theta3_end_norm, params.theta4_end_norm);

params.dx = 0.5;
params.dy = 0.5;
params.nx = ceil(params.xhorizon / params.dx) + 1;
params.ny = ceil(params.yhorizon / params.dy) + 1;


params.nfe = 200;
params.unit_step = 0.1;
params.max_step = 5;

CreateDilatedCostmap();
params.dialated_xy_grids_cell = CreateDilatedGrids();

params.colorpool = [237,28,36; 0,162,232; 34,177,76; 255,127,39]./255;

params.max_iter = 3;
params.max_cpu = 5;
params.trust_region_scale = 2;



% 增加部分
params.dtheta = 0.5;
params.ntheta = ceil(2 * pi / params.dtheta) + 1;
params.num_phy_ha = 9;
params.min_turning_radius = params.L_tractor_wheelbase / tan(params.phy_max);
params.ha_penalty_on_v_change = 0.1;
params.ha_penalty_on_reversing = 0.05;
params.ha_penalty_on_phy_change = 0.01;
params.weight_biased_from_reference_line = 0.01;
params.ha_simu_unit_duration = 3;
params.enable_information_based_route = 1;
params.max_iter_for_HA = 416;
params.w_gh = 2;
params.control_nv = 4;
end

function [x2, y2, x3, y3, x4, y4] = MapFrom6DimTo4Dim(x1, y1, theta1, theta2, theta3, theta4)
global params
x2 = x1 - params.L(2) * cos(theta2) - params.M(1) * cos(theta1);
y2 = y1 - params.L(2) * sin(theta2) - params.M(1) * sin(theta1);
x3 = x2 - params.L(3) * cos(theta3) - params.M(2) * cos(theta2);
y3 = y2 - params.L(3) * sin(theta3) - params.M(2) * sin(theta2);
x4 = x3 - params.L(4) * cos(theta4) - params.M(3) * cos(theta3);
y4 = y3 - params.L(4) * sin(theta4) - params.M(3) * sin(theta3);
end

function CreateDilatedCostmap()
global params
xmin = params.xmin;
ymin = params.ymin;
resolution_x = params.dx;
resolution_y = params.dy;
costmap = zeros(params.nx, params.ny);
params.potential_field = zeros(params.nx, params.ny);

for ii = 1 : size(params.obs, 2)
    vx = params.obs{1, ii}.x;
    vy = params.obs{1, ii}.y;
    x_lb = min(vx);
    x_ub = max(vx);
    y_lb = min(vy);
    y_ub = max(vy);
    
    [Nmin_x, Nmin_y] = ConvertXYToIndex(x_lb, y_lb);
    [Nmax_x, Nmax_y] = ConvertXYToIndex(x_ub, y_ub);
    for jj = Nmin_x : Nmax_x
        for kk = Nmin_y : Nmax_y
            if (costmap(jj, kk) == 1)
                continue;
            end
            cur_x = xmin + (jj - 1) * resolution_x;
            cur_y = ymin + (kk - 1) * resolution_y;
            if (inpolygon(cur_x, cur_y, vx, vy) == 1)
                costmap(jj, kk) = 1;
                UpdatePotentialField(jj, kk);
            end
        end
    end
end

length_unit = 0.5 * (resolution_x + resolution_y);
basic_elem = strel('disk', 1 + ceil(params.radius / length_unit));
params.dialated_map = imdilate(costmap, basic_elem);
basic_elem = strel('disk', 1);
params.basic_map = costmap;
params.potential_field = params.potential_field ./ max(max(params.potential_field));
end

function [ind1,ind2] = ConvertXYToIndex(x, y)
global params
ind1 = ceil((x - params.xmin) / params.dx) + 1;
ind2 = ceil((y - params.ymin) / params.dy) + 1;
ind1 = max(1, min(ind1, params.nx));
ind2 = max(1, min(ind2, params.ny));
end

function dialated_xy_grids_cell = CreateDilatedGrids()
global params
[ii, jj] = find(params.dialated_map == 1);
dialated_xy_grids_cell.x = params.xmin + (ii-1) .* params.dx;
dialated_xy_grids_cell.y = params.ymin + (jj-1) .* params.dy;
end

function UpdatePotentialField(jj, kk)
global params
ind_x_lb = max(min(jj - 5, params.nx), 1);
ind_x_ub = max(min(jj + 5, params.nx), 1);
ind_y_lb = max(min(kk - 5, params.ny), 1);
ind_y_ub = max(min(kk + 5, params.ny), 1);
for i = ind_x_lb : ind_x_ub
    for j = ind_y_lb : ind_y_ub
        distance = hypot(i - jj, j - kk);
        params.potential_field(i,j) = params.potential_field(i,j) + exp(-0.1 * distance);
    end
end
end