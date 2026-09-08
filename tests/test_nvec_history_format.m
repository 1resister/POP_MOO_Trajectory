function test_nvec_history_format()
%TEST_NVEC_HISTORY_FORMAT Guard against struct2table numeric-vector expansion.
t=table([1 2 3 4;5 6 7 8],'VariableNames',{'Nvec'});
t=normalize_nvec_history(t);
assert(iscell(t.Nvec) && isequal(size(t.Nvec),[2 1]));
assert(isequal(t.Nvec{1},[1 2 3 4]));
assert(isequal(t.Nvec{2},[5 6 7 8]));
t2=normalize_nvec_history(t);
assert(isequal(t2.Nvec,t.Nvec),'Cell-form Nvec history must be idempotent.');
fprintf('PASS test_nvec_history_format\n');
end
