function results = run_unit_tests(cfg)
%RUN_UNIT_TESTS Run deterministic project tests without test-framework setup.
if nargin<1, cfg=[]; end
tests = {@test_pop_discretization,@test_augmented_resonance, ...
    @test_dynamic_tolerance,@test_contour_error,@test_square_path, ...
    @test_resonance_model,@test_high_order_smoothstep, ...
    @test_boundary_conditions,@test_nvec_history_format, ...
    @test_vibration_objective_jerk,@test_resonance_band_objective, ...
    @test_time_sweep_frequency_analysis,@test_parameter_settings_table, ...
    @test_time_vibration_pareto};
results = table('Size',[numel(tests),3], ...
    'VariableTypes',{'string','logical','string'}, ...
    'VariableNames',{'Test','Passed','Message'});
for i=1:numel(tests)
    testName=func2str(tests{i});
    results.Test(i)=testName;
    try
        if strcmp(testName,'test_resonance_model') && ~isempty(cfg)
            test_resonance_model(cfg);
        elseif strcmp(testName,'test_parameter_settings_table') && ~isempty(cfg)
            test_parameter_settings_table(cfg);
        else
            tests{i}();
        end
        results.Passed(i)=true; results.Message(i)="PASS";
    catch ME
        results.Passed(i)=false; results.Message(i)=string(ME.message);
        fprintf(2,'FAIL %s: %s\n',testName,ME.message);
    end
end
if ~all(results.Passed)
    error('POP_MOO:UnitTestsFailed','%d unit test(s) failed.',sum(~results.Passed));
end
fprintf('All %d unit tests passed.\n',height(results));
end
