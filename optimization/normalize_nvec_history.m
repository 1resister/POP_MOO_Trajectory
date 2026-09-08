function history = normalize_nvec_history(history)
%NORMALIZE_NVEC_HISTORY Store each Nvec table row as one row-vector cell.
% MATLAB releases may expand equal-sized numeric vectors from struct2table
% into an N-by-nSegment numeric variable. Downstream code uses one cell per
% candidate so that both fixed and potentially variable vector sizes work.
if isempty(history) || ~ismember('Nvec',history.Properties.VariableNames)
    return
end
if isnumeric(history.Nvec)
    values=history.Nvec;
    history.Nvec=mat2cell(values,ones(size(values,1),1),size(values,2));
elseif iscell(history.Nvec)
    values=history.Nvec;
    if size(values,2)==1
        history.Nvec=values;
    else
        combined=cell(size(values,1),1);
        for row=1:size(values,1)
            combined{row}=reshape(cell2mat(values(row,:)),1,[]);
        end
        history.Nvec=combined;
    end
else
    error('POP_MOO:BadNvecHistory','Unsupported Nvec history type: %s.',class(history.Nvec));
end
end
