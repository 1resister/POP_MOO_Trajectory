function results = run_square_pop_moo(cfg)
%RUN_SQUARE_POP_MOO Main entry point for all required square experiments.
root=fileparts(mfilename('fullpath')); addpath(genpath(root));
if nargin<1, cfg=trajectory_config(); end
fprintf('============================================================\n');
fprintf('POP-Controlled Multi-Objective Trajectory Optimisation\n');
fprintf('============================================================\n');
fprintf('Path:          Closed square %.3f mm x %.3f mm\n',cfg.path.square_side,cfg.path.square_side);
fprintf('Interpolation: Ts = %.3f ms\n',1e3*cfg.Ts);
fprintf('Geometry:      line %.3f mm, corner %.3f mm, transition %.3f mm\n', ...
    cfg.geometry.line_tolerance,cfg.geometry.corner_tolerance,cfg.geometry.transition_length);
fprintf('Limits: V %.6g, A %.6g, J %.6g, Snap %.6g, Crackle %.6g, POP %.6g\n', ...
    cfg.limits.Vmax,cfg.limits.Amax,cfg.limits.Jmax,cfg.limits.SnapMax, ...
    cfg.limits.CrackleMax,cfg.limits.POPMax);
results=run_all_cases(cfg);
fprintf('\nMinimum feasible N: %d samples\nTmin: %.3f s\n', ...
    results.solution_min_time.N,results.solution_min_time.T);
disp(results.comparison_table);
fprintf('All output written to:\n  figures: %s\n  csv:     %s\n  mat:     %s\n', ...
    cfg.output.figures,cfg.output.csv,cfg.output.mat);
end
