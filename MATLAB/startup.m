% startup.m - Automatic GNC system configuration
% Place this file in: MATLAB/startup.m
%
% This file automatically runs when MATLAB starts from the MATLAB directory.
% It configures all necessary paths for the GNC system.

fprintf('=== GNC System Auto-Configuration ===\n');

try
    % Check if we're in the right directory
    if exist('setup_gnc_paths.m', 'file') == 2
        setup_gnc_paths();
        fprintf('GNC system paths configured successfully!\n');
        fprintf('All modules ready: lib, GNC_Algorithms, examples\n');
    else
        fprintf('setup_gnc_paths.m not found in current directory\n');
        fprintf('Please ensure you start MATLAB from the MATLAB directory\n');
    end
    
catch ME
    fprintf('Error during GNC setup: %s\n', ME.message);
    fprintf('Please run setup_gnc_paths() manually if needed\n');
end

fprintf('=========================================\n\n');

% Clear temporary variables
clear;