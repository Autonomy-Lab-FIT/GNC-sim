function merged_config = merge_config(varargin)
%MERGE_CONFIG Merge multiple configuration structures
%   Combines multiple configuration structures into one
%   Later configurations override earlier ones for overlapping fields
%
%   Usage:
%       config = merge_config(sys_config, ukf_config);
%       config = merge_config(sys_config, ukf_config, custom_overrides);
%
%   Inputs:
%       varargin - Variable number of configuration structures
%
%   Output:
%       merged_config - Combined configuration structure

    if nargin == 0
        merged_config = struct();
        return;
    end
    
    % Start with the first configuration
    merged_config = varargin{1};
    
    % Validate first input is a structure
    if ~isstruct(merged_config)
        error('All inputs to merge_config must be structures');
    end
    
    % Merge each additional configuration
    for i = 2:nargin
        config_to_merge = varargin{i};
        
        % Validate input is a structure
        if ~isstruct(config_to_merge)
            error('All inputs to merge_config must be structures');
        end
        
        % Get all field names from the configuration to merge
        field_names = fieldnames(config_to_merge);
        
        % Copy each field to the merged configuration
        for j = 1:length(field_names)
            field_name = field_names{j};
            merged_config.(field_name) = config_to_merge.(field_name);
        end
    end
    
    % Add merge metadata
    merged_config.merge_info = struct();
    merged_config.merge_info.num_configs_merged = nargin;
    
end