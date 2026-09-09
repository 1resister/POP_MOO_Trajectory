function cfg = trajectory_config()
%TRAJECTORY_CONFIG Central configuration for the POP-controlled OCP.
% All dimensional quantities use millimetres and seconds.

cfg.project_root = fileparts(mfilename('fullpath'));
cfg.Ts = 0.001;                         % interpolation period [s]

cfg.path.type = 'square';
cfg.path.square_side = 20.0;            % square side [mm]
cfg.path.closed = true;
cfg.path.csv_file = '';

cfg.geometry.line_tolerance = 0.05;     % line-core tolerance [mm]
cfg.geometry.corner_tolerance = 0.10;   % corner tolerance [mm]
cfg.geometry.transition_length = 2.0;   % transition length each side [mm]
cfg.geometry.overlap_extension = 0.10;  % tangential corner overlap [mm]

cfg.limits.Vmax = 500;                  % [mm/s]
cfg.limits.Amax = 10000;                % [mm/s^2]
cfg.limits.Jmax = 200000;               % [mm/s^3]
cfg.limits.SnapMax = 7e6;               % [mm/s^4]
cfg.limits.CrackleMax = 2e11;           % [mm/s^5]
cfg.limits.POPMax = 2e11;               % [mm/s^6]

cfg.boundary.zero_velocity = true;
cfg.boundary.zero_acceleration = true;
cfg.boundary.zero_jerk = true;
cfg.boundary.zero_snap = true;
cfg.boundary.zero_crackle = true;
cfg.boundary.zero_endpoint_pop = true;

% NLP variables are dimensionless after division by these scales.
cfg.scale.position = cfg.path.square_side; % [mm]
cfg.scale.velocity = cfg.limits.Vmax;       % [mm/s]
cfg.scale.acceleration = cfg.limits.Amax;   % [mm/s^2]
cfg.scale.jerk = cfg.limits.Jmax;           % [mm/s^3]
cfg.scale.snap = cfg.limits.SnapMax;         % [mm/s^4]
cfg.scale.crackle = cfg.limits.CrackleMax;   % [mm/s^5]
cfg.scale.pop = cfg.limits.POPMax;           % [mm/s^6]

cfg.optimization.time_slack = 0.02;
cfg.optimization.initial_segment_time = 0.45; % slow-guess duration [s]
cfg.optimization.initial_growth = 1.15;
cfg.optimization.max_initial_attempts = 8;
cfg.optimization.continuation_factor = 0.92;
cfg.optimization.min_segment_samples = 12;
cfg.optimization.neighborhood_radius = 2;
cfg.optimization.max_coordinate_passes = 120;
% Eleven evenly spaced weights provide a denser Pareto front while keeping
% the default Stage B run practical. Increase further when needed.
cfg.optimization.pareto_weights = 0:0.1:1;
cfg.optimization.secondary_sample_offsets = [];
cfg.optimization.run_optional_two_mode = false;
% Optimize one no-suppression/suppression pair at every integer machining
% time in [Nmin, floor((1+time_slack)*Nmin)].
cfg.optimization.time_sweep_enable = true;
cfg.optimization.time_sweep_lambda_vibration = 1.0;
cfg.optimization.time_sweep_resume = true;
cfg.optimization.time_sweep_boundary_cpu_multiplier = 2.0;

cfg.straightness.enable = true;
cfg.straightness.hard_enable = false;
cfg.straightness.core_velocity_tolerance = 1e-7; % normalized tolerance

cfg.resonance.enable = true;
cfg.resonance.mode(1).enable = true;
cfg.resonance.mode(1).frequency = 50;    % [Hz]
cfg.resonance.mode(1).zeta = 0.02;       % damping ratio [-]
cfg.resonance.mode(1).gain = 1.0;        % static gain [-]
cfg.resonance.mode(2).enable = false;
cfg.resonance.mode(2).frequency = 35;    % [Hz]
cfg.resonance.mode(2).zeta = 0.02;
cfg.resonance.mode(2).gain = 1.0;

cfg.frequency.bandwidth = 1.0;           % half bandwidth [Hz]
cfg.frequency.maximum_plot_frequency = 100; % [Hz]

cfg.objective.vibration_energy_weight = 1.0;
cfg.objective.vibration_peak_weight = 0.1;
cfg.objective.acceleration_rms_weight = 0.2;
cfg.objective.acceleration_peak_weight = 0.1;
cfg.objective.jerk_rms_weight = 0.2;
cfg.objective.jerk_peak_weight = 0.1;
% Explicit target-frequency PSD-band term.  The weight controls how far
% the optimizer pushes below the hard minimum reduction.
cfg.objective.resonance_band_weight = 1.0;
cfg.objective.resonance_band_hard_enable = true;
cfg.objective.resonance_band_min_reduction = 0.05; % at least 5% vs no suppression
cfg.objective.feas_pop_weight = 1e-8;
cfg.objective.feas_crackle_weight = 1e-8;
cfg.objective.feas_snap_weight = 1e-8;

cfg.solver.ipopt.max_iter = 3000;
cfg.solver.ipopt.max_cpu_time = 180;      % [s] per fixed-N solve
cfg.solver.ipopt.tol = 1e-6;
cfg.solver.ipopt.acceptable_tol = 1e-5;
cfg.solver.ipopt.acceptable_iter = 8;
cfg.solver.ipopt.print_level = 0;
cfg.solver.expand = true;
cfg.solver.hard_bound_margin = 0;        % optional numerical interior margin [-]

cfg.validation.dynamics_abs_tol = 2e-6;
cfg.validation.boundary_abs_tol = 2e-5;
cfg.validation.relative_limit_tol = 1e-4;
cfg.validation.geometry_abs_tol = 2e-4;  % [mm]
cfg.validation.time_abs_tol = 1e-12;     % [s]
cfg.validation.backward_velocity_tol = 1e-4; % [mm/s]
cfg.validation.nan_fail = true;
cfg.validation.resonance_band_relative_tol = 1e-3;

cfg.plot.visible = 'off';
cfg.plot.resolution = 180;               % saved PNG dpi
cfg.output.figures = fullfile(cfg.project_root, 'output', 'figures');
cfg.output.csv = fullfile(cfg.project_root, 'output', 'csv');
cfg.output.mat = fullfile(cfg.project_root, 'output', 'mat');
end
