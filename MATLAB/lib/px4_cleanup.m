function px4_cleanup(client)
%PX4_CLEANUP Clean up connection resources
%   Safely closes the TCP client connection
%
%   Input:
%       client - TCP client object to clean up
    fprintf('Cleanup (stub)\n');
    try
        if exist('client', 'var') && isvalid(client)
            clear client;
        end
    catch
        % Ignore cleanup errors
    end
end
