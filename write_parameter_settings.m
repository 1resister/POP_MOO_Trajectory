function settings = write_parameter_settings(cfg)
%WRITE_PARAMETER_SETTINGS Export the current configuration as a Chinese CSV.
if nargin < 1 || isempty(cfg)
    cfg = trajectory_config();
end
if ~exist(cfg.output.csv,'dir')
    mkdir(cfg.output.csv);
end
settings = build_parameter_settings_table(cfg);
writetable(settings,fullfile(cfg.output.csv,'parameter_settings.csv'), ...
    'Encoding','UTF-8');
end
