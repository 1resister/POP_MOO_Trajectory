function Nvec = stage_b_integer_allocation(solutionMin,path,targetN)
%STAGE_B_INTEGER_ALLOCATION Deterministic segment allocation at one time.
Nmin=solutionMin.N;
if targetN<Nmin || targetN~=round(targetN)
    error('POP_MOO:BadStageBTime', ...
        'Target N must be an integer no smaller than Nmin=%d.',Nmin);
end
Nvec=solutionMin.Nvec;
extra=targetN-Nmin;
% Add each available sample to the segment with the shortest normalized
% duration.  Calling this function independently for every target N keeps
% the complete time sweep deterministic and nested.
for k=1:extra
    [~,segmentIndex]=min(Nvec./path.lengths(:).');
    Nvec(segmentIndex)=Nvec(segmentIndex)+1;
end
end
