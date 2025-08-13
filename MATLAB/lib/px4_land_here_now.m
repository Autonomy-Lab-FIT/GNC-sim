function success = px4_land_here_now(client, config)
%PX4_LAND_HERE_NOW Immediate landing at current position
%   No navigation - just land where you are
    
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