function BrakingSystemSimulation()
    % Main function for vehicle braking system simulation
    % This single file contains all required functions
    
    % Clear workspace and figures
    clear
    close all
    clc
    
    % Initialize system parameters
    params = initializeParameters();
    
    % Calculate geometric properties
    geometry = calculateGeometry(params);
    
    % Calculate forces and pressures
    forces = calculateForces(params, geometry);
    
    % Run the braking simulation
    results = runBrakingSimulation(params, geometry, forces);
    
    % Plot and display results
    plotResults(results);
    displayFinalValues(results);
    
    % Nested helper functions
    function params = initializeParameters()
        % Initialize all system parameters
        
        % Driver parameters
        params.F_D = 500;       % Driver force on brake pedal (N)
        params.v_0 = 120/3.6;   % Initial velocity (m/s)
        
        % Road parameters
        params.grad = 0;        % Road gradient (%)
        params.mu_max = 0.8;    % Maximum adhesion limit
        params.mu_s = 0.65;     % Skidding adhesion limit
        params.v_a = 0;         % Air speed (m/s)
        
        % Pedal parameters
        params.i_pedal = 4;     % Pedal amplification ratio
        
        % Servo parameters
        params.p_atm = 1;       % Atmospheric pressure (bar)
        params.p_manifold = 0.60; % Intake manifold pressure (bar)
        params.D_servo = 260;   % Outer diameter of servo (mm)
        params.d_servo = 60;    % Inner diameter of servo (mm)
        params.factor = 1;      % Boost ratio
        
        % Master cylinder parameters
        params.d_master = 10;   % Master cylinder diameter (mm)
        
        % Front disk parameters
        params.d_f_disk = 55;   % Piston diameter (mm)
        params.Do_f_disk = 200; % Outer disk diameter (mm)
        params.Di_f_disk = 160; % Inner disk diameter (mm)
        params.mu_f_disk = 0.3; % Friction coefficient
        params.n = 2;           % Number of friction surfaces
        
        % Rear disk parameters
        params.d_r_disk = 50;   % Piston diameter (mm)
        params.Do_r_disk = 220; % Outer disk diameter (mm)
        params.Di_r_disk = 180; % Inner disk diameter (mm)
        params.mu_r_disk = 0.2813; % Friction coefficient
        
        % Rear drum parameters
        params.d_r_drum = 20;   % Piston diameter (mm)
        params.D_r_drum = 200;  % Outer drum diameter (mm)
        params.C_r_drum = 1.75; % Brake factor
        
        % Tire parameters
        params.tire_w = 195;    % Tire width (mm)
        params.tire_AR = 65;    % Aspect ratio (%)
        params.rim = 15;        % Rim diameter (inch)
        params.m_tire = 20;     % Tire mass (kg)
        
        % Vehicle parameters
        params.m = 1200;        % Total mass (kg)
        params.CG = 45;         % CG location from front axle (%)
        params.CG_H = 600;      % CG height (mm)
        params.CG_L = 50;       % CG location from left side (%)
        params.track = 1425;    % Wheel track (mm)
        params.wheel_base = 2540; % Wheel base (mm)
        params.n_f = 2;         % Number of front wheels
        params.n_r = 2;         % Number of rear wheels
        params.REAR = 'disk';   % Rear brake type ('disk' or 'drum')
        params.Af = 2;          % Frontal area (m²)
        
        % Resistance parameters
        params.fr = 0.007;      % Rolling resistance coefficient
        params.cd = 0.4;        % Drag coefficient
        
        % Simulation parameters
        params.h = 0.0001;      % Time step (s)
        params.T = 60;          % Total simulation time (s)
    end

    function geometry = calculateGeometry(params)
        % Calculate all geometric properties
        
        geometry.A_servo = Area_F(params.D_servo, params.d_servo); % mm²
        geometry.A_master = Area_F(params.d_master); % mm²
        geometry.A_f_disk = Area_F(params.d_f_disk); % mm²
        geometry.A_r_disk = Area_F(params.d_r_disk); % mm²
        geometry.A_r_drum = Area_F(params.d_r_drum); % mm²
        
        geometry.R_mean_f_disk = (1e-3) * R_mean_F(params.Do_f_disk, params.Di_f_disk); % m
        geometry.R_mean_r_disk = (1e-3) * R_mean_F(params.Do_r_disk, params.Di_r_disk); % m
        
        geometry.R_tire = ((params.tire_w * params.tire_AR / 100) + ...
                         (params.rim * 25.4 / 2)) * 1e-3; % m
        geometry.I_tire = 0.5 * params.m_tire * (geometry.R_tire^2);
        geometry.w_0 = params.v_0/geometry.R_tire;
    end

    function forces = calculateForces(params, geometry)
        % Calculate initial forces and pressures
        
        forces.F_pedal = params.F_D * params.i_pedal; % N
        forces.F_servo = params.factor * (params.p_atm - params.p_manifold) * ...
                        geometry.A_servo / 10; % N
        forces.P_line = (forces.F_pedal + forces.F_servo) / geometry.A_master; % MPa
        
        % Friction forces
        forces.F_fric_f_disk = forces.P_line * geometry.A_f_disk * ...
                              params.mu_f_disk * params.n; % N
        forces.F_fric_r_disk = forces.P_line * geometry.A_r_disk * ...
                              params.mu_r_disk * params.n; % N
        forces.F_fric_r_drum = forces.P_line * geometry.A_r_drum * ...
                              params.C_r_drum * 2; % N
        
        % Friction torques
        forces.T_fric_f_disk = forces.F_fric_f_disk * geometry.R_mean_f_disk; % N.m
        forces.T_fric_r_disk = forces.F_fric_r_disk * geometry.R_mean_r_disk; % N.m
        forces.T_fric_r_drum = forces.F_fric_r_drum * params.D_r_drum / 200; % N.m
        
        % Check rear brake type
        rear = strcmp(params.REAR, 'disk');
        rear_check = strcmp(params.REAR, 'drum');
        if rear == 0 && rear_check == 0
            error('Please select rear brakes as "disk" or "drum"');
        end
        
        % Total braking torques
        forces.T_r = params.n_r * ((forces.T_fric_r_disk * rear) + ...
                    forces.T_fric_r_drum * (1 - rear));
        forces.T_f = params.n_f * forces.T_fric_f_disk; % N.m
        
        % Resistance forces
        forces.R_rolling = params.fr * params.m * 9.81 * ...
                          cos(atan(params.grad / 100)); % N
        forces.R_grad = sin(atan(params.grad / 100)) * params.m * 9.81; % N
    end

    function results = runBrakingSimulation(params, geometry, forces)
        % Run the time-stepping braking simulation
        
        % Initialize variables
        V = params.v_0;
        W_f = geometry.w_0;
        W_r = geometry.w_0;
        N = 0;
        X = 0;
        t = 0;
        reg = 1;
        
        % Preallocate arrays for results
        maxSteps = ceil(params.T / params.h);
        results.time = zeros(maxSteps, 1);
        results.acc = zeros(maxSteps, 1);
        results.vel = zeros(maxSteps, 1);
        results.dist = zeros(maxSteps, 1);
        results.slipping_r = zeros(maxSteps, 1);
        results.slipping_f = zeros(maxSteps, 1);
        results.vel_w = zeros(maxSteps, 1);
        results.acc_w = zeros(maxSteps, 1);
        results.muu = zeros(maxSteps, 1);
        results.regg = zeros(maxSteps, 1);
        results.P_outt = zeros(maxSteps, 1);
        
        % Main simulation loop
        while t < params.T
            N = N + 1;
            
            % Relative speed
            vr = V - params.v_a;
            
            % Air resistance
            R_air = 0.5 * 1.225 * params.cd * params.Af * (vr^2); % N
            
            % Total resistance force
            R_total = 0.5 * sign(V) * (R_air + forces.R_rolling) + forces.R_grad;
            
            % Slipping calculation
            s_f = slipping(V, W_f * geometry.R_tire);
            s_r = slipping(V, W_r * geometry.R_tire);
            mu_f = mu(s_f);
            mu_r = mu(s_r);
            
            % ABS regulation
            reg = ABS(s_r);
            
            % Braking force calculation
            F_braking = (mu_r*(1-(params.CG/100)) + mu_f*(params.CG/100)) * params.m * 9.81;
            
            % Acceleration calculation
            ddx = -(R_total + 0.5 * sign(V) * F_braking) / params.m; % m/s²
            ddq_f = ((params.m * 9.81 * mu_f * (1 - (params.CG / 100))) - ...
                    ((forces.T_f*reg) / params.n_f)) / geometry.I_tire;
            ddq_r = ((params.m * 9.81 * mu_r * (params.CG / 100)) - ...
                    ((forces.T_r*reg) / params.n_r)) / geometry.I_tire;
            
            % Velocity update
            dx = (ddx * params.h) + V;
            dq_r = max(0, (ddq_r * params.h) + W_r);
            dq_f = max(0, (ddq_f * params.h) + W_f);
            
            % Pressure output
            P_out = reg * forces.P_line;
            
            % Position update
            x = dx * params.h;
            X = X + x;
            
            % Update velocity
            V = max(dx, 0);
            W_r = max(dq_r, 0);
            W_f = max(dq_f, 0);
            
            % Store results
            results.time(N) = t;
            results.acc(N) = ddx;
            results.vel(N) = dx;
            results.dist(N) = X;
            results.slipping_r(N) = s_r;
            results.slipping_f(N) = s_f;
            results.vel_w(N) = dq_r * geometry.R_tire;
            results.acc_w(N) = ddq_r * geometry.R_tire;
            results.muu(N) = mu_r;
            results.regg(N) = reg;
            results.P_outt(N) = P_out;
            
            % Break condition if vehicle stops
            if V <= 0
                break;
            end
            
            % Update time
            t = t + params.h;
        end
        
        % Trim unused array elements
        results.time = results.time(1:N);
        results.acc = results.acc(1:N);
        results.vel = results.vel(1:N);
        results.dist = results.dist(1:N);
        results.slipping_r = results.slipping_r(1:N);
        results.slipping_f = results.slipping_f(1:N);
        results.vel_w = results.vel_w(1:N);
        results.acc_w = results.acc_w(1:N);
        results.muu = results.muu(1:N);
        results.regg = results.regg(1:N);
        results.P_outt = results.P_outt(1:N);
    end

    function plotResults(results)
        % Plot all simulation results
        
        figure;
        plot(results.time, results.dist);
        xlabel('Time (s)'); ylabel('Distance (m)');
        title('Distance vs Time');
        grid on;
        
        figure;
        plot(results.time, results.slipping_f);
        xlabel('Time (s)'); ylabel('Slipping front');
        title('Front Wheel Slipping vs Time');
        grid on;
        
        figure;
        plot(results.time, results.slipping_r);
        xlabel('Time (s)'); ylabel('Slipping rear');
        title('Rear Wheel Slipping vs Time');
        grid on;
        
        figure;
        plot(results.time, results.muu);
        xlabel('Time (s)'); ylabel('Friction coefficient (μ)');
        title('Friction Coefficient vs Time');
        grid on;
        
        figure;
        plot(results.time, results.regg);
        xlabel('Time (s)'); ylabel('ABS Regulation Factor');
        title('ABS Regulation vs Time');
        grid on;
        
        figure;
        plot(results.time, results.P_outt);
        xlabel('Time (s)'); ylabel('Brake Pressure (MPa)');
        title('Brake Pressure vs Time');
        grid on;
        
        figure;
        plot(results.time, results.vel_w, 'r', 'LineWidth', 2);
        hold on;
        plot(results.time, results.vel, 'b', 'LineWidth', 2);
        hold off;
        xlabel('Time (s)'); ylabel('Velocity (m/s)');
        title('Vehicle and Wheel Velocities vs Time');
        legend('Wheel Velocity', 'Vehicle Velocity');
        grid on;
        
        figure;
        plot(results.time, results.acc_w, 'r', 'LineWidth', 2);
        hold on;
        plot(results.time, results.acc, 'b', 'LineWidth', 2);
        hold off;
        xlabel('Time (s)'); ylabel('Acceleration (m/s²)');
        title('Vehicle and Wheel Accelerations vs Time');
        legend('Wheel Acceleration', 'Vehicle Acceleration');
        grid on;
    end

    function displayFinalValues(results)
        % Display final simulation values
        
        fprintf('Final Simulation Results:\n');
        fprintf('------------------------\n');
        fprintf('Stopping time: %.3f s\n', results.time(end));
        fprintf('Final acceleration: %.3f m/s²\n', results.acc(end));
        fprintf('Final velocity: %.3f m/s\n', results.vel(end));
        fprintf('Total stopping distance: %.3f m\n', results.dist(end));
    end

    % Helper functions
    function mu_val = mu(slip)
        % Calculate friction coefficient based on slip
        
        mu_max = 0.8;    % Maximum friction coefficient
        slip_peak = 0.2; % Slip at max friction
        
        if slip <= slip_peak
            mu_val = mu_max * (slip / slip_peak); % Linear increase
        else
            mu_val = 0.5; % Constant value after peak
        end
    end

    function R_mean = R_mean_F(D_outer, D_inner)
        % Calculate mean radius
        
        R_mean = (D_outer + D_inner) / 4; % Mean of outer and inner radii
    end

    function slip = slipping(V_car, V_wheel)
        % Compute slip ratio while handling edge cases
        
        if V_car == 0  % Avoid division by zero
            if V_wheel == 0
                slip = 0;  % No motion, no slip
            else
                slip = 1;  % Wheel is spinning, car is stopped
            end
        else
            slip = 1 - (V_wheel / V_car);
        end
        
        % Ensure slip is in valid range [0,1]
        slip = max(0, min(slip, 1));
    end

    function reg = ABS(slipping)
        % ABS regulation logic
        
        if slipping < 0.13
            reg = 1.2;  % Increase force slightly
        elseif slipping >= 0.20 
            reg = 0.4;  % Reduce force significantly
        else
            reg = 1;    % Normal operation
        end
    end

    function A = Area_F(D, d)
        % Calculate circular or annular area
        
        if nargin == 2
            % Annular area (for cases with inner and outer diameters)
            A = pi * (D^2 - d^2) / 4;
        else
            % Full circular area (for cases with only one diameter)
            A = pi * (D^2) / 4;
        end
    end
end