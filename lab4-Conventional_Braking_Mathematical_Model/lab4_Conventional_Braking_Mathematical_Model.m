function braking_simulation()
    clear; close all; clc;
    
    %% Define System Parameters
    params = define_parameters();
    
    %% Geometry Calculations
    geo = compute_geometry(params);
    
    %% Force and Pressure Calculations
    forces = compute_forces(params, geo);
    
    %% Run Braking Simulation
    results = run_simulation(params, forces, geo);
    
    %% Plot Results
    plot_results(results);
end

%% Function Definitions
function params = define_parameters()
    params.F_D = 150; % Driver force (N)
    params.v_0 = 120 / 3.6; % Initial velocity (m/s)
    params.grad = 0; % Road gradient (%)
    params.mu_max = 0.8;
    params.mu_s = 0.65;
    params.v_a = 0 / 3.6; % Air speed (m/s)
    params.i_pedal = 4;
    params.p_atm = 1; % Atmospheric pressure (bar)
    params.p_manifold = 0.45; % Manifold pressure (bar)
    params.D_servo = 238; params.d_servo = 75;
    params.d_master = 18;
    params.d_f_disk = 55; params.Do_f_disk = 200; params.Di_f_disk = 160; params.mu_f_disk = 0.3;
    params.d_r_disk = 50; params.Do_r_disk = 220; params.Di_r_disk = 180; params.mu_r_disk = 0.2813;
    params.d_r_drum = 20; params.D_r_drum = 200; params.C_r_drum = 1.75;
    params.tire_w = 195; params.tire_AR = 65; params.rim = 15;
    params.m = 2000; params.CG_H = 600; params.track = 1425; params.wheel_base = 2540;
    params.n_f = 2; params.n_r = 2; params.REAR = 'disk';
    params.Af = 2; params.fr = 0.007; params.cd = 0.4;
end

function geo = compute_geometry(params)
    geo.A_servo = area_calc(params.D_servo, params.d_servo);
    geo.A_master = area_calc(params.d_master);
    geo.A_f_disk = area_calc(params.d_f_disk);
    geo.A_r_disk = area_calc(params.d_r_disk);
    geo.A_r_drum = area_calc(params.d_r_drum);
    geo.R_mean_f_disk = 1e-3 * mean_radius(params.Do_f_disk, params.Di_f_disk);
    geo.R_mean_r_disk = 1e-3 * mean_radius(params.Do_r_disk, params.Di_r_disk);
    geo.R_tire = ((params.tire_w * params.tire_AR / 100) + (params.rim * 25.4 / 2)) * 1e-3;
end

function forces = compute_forces(params, geo)
    F_pedal = params.F_D * params.i_pedal;
    F_servo = (params.p_atm - params.p_manifold) * geo.A_servo / 10;
    P_line = (F_pedal + F_servo) / geo.A_master;
    
    forces.F_fric_f_disk = P_line * geo.A_f_disk * params.mu_f_disk * 2;
    forces.F_fric_r_disk = P_line * geo.A_r_disk * params.mu_r_disk * 2;
    forces.F_fric_r_drum = P_line * geo.A_r_drum * params.C_r_drum * 2;
    
    forces.T_fric_f_disk = forces.F_fric_f_disk * geo.R_mean_f_disk;
    forces.T_fric_r_disk = forces.F_fric_r_disk * geo.R_mean_r_disk;
    forces.T_fric_r_drum = forces.F_fric_r_drum * params.D_r_drum / 200;
    
    rear = strcmp(params.REAR, 'disk');
    if ~rear && ~strcmp(params.REAR, 'drum')
        error('Select rear brakes as either disk or drum');
    end
    
    forces.T_r = params.n_r * ((forces.T_fric_r_disk * rear) + forces.T_fric_r_drum * (1 - rear));
    forces.T_f = params.n_f * forces.T_fric_f_disk;
    forces.F_braking = (forces.T_r + forces.T_f) / geo.R_tire;
end

function results = run_simulation(params, forces, geo)
    h = 0.0001; T = 10;
    V = params.v_0; X = 0; t = 0;
    results.time = []; results.acc = []; results.vel = []; results.dist = [];
    
    while t < T
        vr = V - params.v_a;
        R_air = 0.5 * 1.225 * params.cd * params.Af * (vr^2);
        R_rolling = params.fr * params.m * 9.81 * cos(atan(params.grad / 100));
        R_grad = sin(atan(params.grad / 100)) * params.m * 9.81;
        R_total = 0.5 * (sign(V) + 1) * (R_air + R_rolling) + R_grad;
        ddx = -(R_total + 0.5 * (sign(V) + 1) * forces.F_braking) / params.m;
        dx = (ddx * h) + V;
        x = dx * h; X = X + x; V = dx;
        
        results.time = [results.time; t];
        results.acc = [results.acc; ddx];
        results.vel = [results.vel; dx];
        results.dist = [results.dist; X];
        
        if V <= 0, break; end
        t = t + h;
    end
end

function plot_results(results)
    figure; plot(results.time, results.acc);
    xlabel('Time (s)'); ylabel('Acceleration (m/s^2)'); title('Acceleration vs Time');
    
    figure; plot(results.time, results.vel);
    xlabel('Time (s)'); ylabel('Velocity (m/s)'); title('Velocity vs Time');
    
    figure; plot(results.time, results.dist);
    xlabel('Time (s)'); ylabel('Distance (m)'); title('Distance vs Time');
end

%% Helper Functions
function A = area_calc(D, d)
    if nargin == 2
        A = pi * (D^2 - d^2) / 4;
    else
        A = pi * D^2 / 4;
    end
end

function R_mean = mean_radius(D_outer, D_inner)
    R_mean = (D_outer + D_inner) / 4;
end
