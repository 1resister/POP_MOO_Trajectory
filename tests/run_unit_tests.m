function results = run_unit_tests()
%RUN_UNIT_TESTS Run deterministic project tests without test-framework setup.
tests = {@test_pop_discretization,@test_augmented_resonance, ...
    @test_dynamic_tolerance,@test_contour_error,@test_square_path, ...
    @test_resonance_model,@test_high_order_smoothstep, ...
    @test_boundary_conditions,@test_nvec_history_format};
results = table('Size',[numel(tests),3], ...
    'VariableTypes',{'string','logical','string'}, ...
    'VariableNames',{'Test','Passed','Message'});
for i=1:numel(tests)
    results.Test(i)=func2str(tests{i});
    try
        tests{i}(); results.Passed(i)=true; results.Message(i)="PASS";
    catch ME
        results.Passed(i)=false; results.Message(i)=string(ME.message);
        fprintf(2,'FAIL %s: %s\n',func2str(tests{i}),ME.message);
    end
end
if ~all(results.Passed)
    error('POP_MOO:UnitTestsFailed','%d unit test(s) failed.',sum(~results.Passed));
end
fprintf('All %d unit tests passed.\n',height(results));
end
