function success = px4_precision_land(client, config, target_x, target_y)
%PX4_PRECISION_LAND Complete precision landing sequence
%   Usage:
%       px4_precision_land(client, config)        % Land at current position
%       px4_precision_land(client, config, x, y)  % Land at specific coordinates
    
    % If no coordinates given, land at current position
    if nargin < 3
        success = px4_land_here_now(client, config);
        return;
    end
    
    % Navigate to target
    success = px4_move_to_target(client, config, target_x, target_y);
    if ~success, return; end
    
    % Initiate landing
    success = px4_initiate_landing(client, config);
    if ~success, return; end
    
    % Wait for completion
    for i = 1:30
        [landed, ~] = px4_check_landing_status(client, config);
        if landed
            success = true;
            return;
        end
        pause(1);
    end
    
    success = false; % Timeout
end