function setup_gnc_paths()
%SETUP_GNC_PATHS Add all GNC algorithm folders to MATLAB path
%   Adds all MATLAB subdirectories to the search path for GNC system.
%   Run once at the beginning of any session.
%
%   Usage: setup_gnc_paths();

    % Find MATLAB directory
    function_dir = fileparts(mfilename('fullpath'));
    current_dir = function_dir;
    matlab_dir = '';
    
    % Search up directory tree for MATLAB folder
    for i = 1:5
        if exist(fullfile(current_dir, 'lib'), 'dir') || ...
           exist(fullfile(current_dir, 'examples'), 'dir') || ...
           contains(current_dir, 'MATLAB')
            matlab_dir = current_dir;
            break;
        end
        parent_dir = fileparts(current_dir);
        if strcmp(parent_dir, current_dir)
            break;
        end
        current_dir = parent_dir;
    end
    
    % Fallback: assume we're in GNC_Algorithms
    if isempty(matlab_dir)
        potential_matlab = fileparts(function_dir);
        if exist(fullfile(potential_matlab, 'lib'), 'dir')
            matlab_dir = potential_matlab;
        else
            matlab_dir = pwd;
        end
    end
    
    % Add all subdirectories recursively
    addpath(genpath(matlab_dir));
    
    % Remove unwanted paths (logs, build folders, etc.)
    remove_paths = {'.git', 'build', 'install', 'log', 'logs'};
    for i = 1:length(remove_paths)
        rmpath(genpath(fullfile(matlab_dir, '**', remove_paths{i})));
    end
    
    fprintf('GNC paths setup complete. Base: %s\n', matlab_dir);
end