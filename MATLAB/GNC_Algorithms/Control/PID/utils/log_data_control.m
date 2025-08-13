function data = log_data_control(data, step, t, x_ukf, telemetry, config, control_type, varargin)
%LOG_DATA_CONTROL Log data for control tests
%   Inputs:
%       data         - Data structure to update
%       step         - Current time step
%       t            - Current time
%       x_ukf        - UKF state vector
%       telemetry    - PX4 telemetry data
%       config       - Configuration structure
%       control_type - 'position' or 'cascade'
%       varargin     - Additional controller outputs
%                      For 'position': vel_cmd_x, vel_cmd_y, vel_cmd_z
%                      For 'cascade': vel_cmd_x, vel_cmd_y, vel_cmd_z, accel_cmd_x, accel_cmd_y, accel_cmd_z
    
    % Common logging
    data.time(step) = t;
    
    % UKF state
    data.ukf_pos_x(step) = x_ukf(1);
    data.ukf_pos_y(step) = x_ukf(2);
    data.ukf_pos_z(step) = x_ukf(3);
    data.ukf_vel_x(step) = x_ukf(4);
    data.ukf_vel_y(step) = x_ukf(5);
    data.ukf_vel_z(step) = x_ukf(6);
    
    % PX4 reference (if available)
    if ~isempty(telemetry) && isfield(telemetry, 'local_position')
        data.px4_pos_x(step) = telemetry.local_position.x;
        data.px4_pos_y(step) = telemetry.local_position.y;
        data.px4_pos_z(step) = telemetry.local_position.z;
    end
    
    % Controller-specific logging
    switch control_type
        case 'position'
            if length(varargin) >= 3
                data.vel_cmd_x(step) = varargin{1};
                data.vel_cmd_y(step) = varargin{2};
                data.vel_cmd_z(step) = varargin{3};
            end
            
        case 'cascade'
            if length(varargin) >= 6
                data.vel_cmd_x(step) = varargin{1};
                data.vel_cmd_y(step) = varargin{2};
                data.vel_cmd_z(step) = varargin{3};
                data.accel_cmd_x(step) = varargin{4};
                data.accel_cmd_y(step) = varargin{5};
                data.accel_cmd_z(step) = varargin{6};
            end
         case 'mpc'
            % Check if enough arguments for control commands are provided
            if length(varargin) >= 6
                % Convert CasADi outputs to double before logging
                data.vel_cmd_x(step)   = full(varargin{1});
                data.vel_cmd_y(step)   = full(varargin{2});
                data.vel_cmd_z(step)   = full(varargin{3});
                data.accel_cmd_x(step) = full(varargin{4});
                data.accel_cmd_y(step) = full(varargin{5});
                data.accel_cmd_z(step) = full(varargin{6});
            end
            
            % MPC-specific logging (solver stats, etc.)
            if length(varargin) >= 7 && ~isempty(varargin{7}) % mpc_state
                mpc_state = varargin{7};
                if isfield(mpc_state, 'solve_time')
                    data.mpc_solve_time(step) = mpc_state.solve_time * 1000; % Convert to ms
                end
                if isfield(mpc_state, 'iterations')
                    data.mpc_iterations(step) = mpc_state.iterations;
                end
                if isfield(mpc_state, 'solver_stats') && isfield(mpc_state.solver_stats, 'return_status')
                    data.mpc_status{step} = mpc_state.solver_stats.return_status;
                end
            end
    end
end