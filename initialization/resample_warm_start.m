function warm = resample_warm_start(previous,Nvec,cfg)
%RESAMPLE_WARM_START Segment-aware deterministic interpolation of X and U.
Nvec=round(Nvec(:).'); model=previous.model;
warm.Nvec=Nvec; warm.N=sum(Nvec); warm.T=warm.N*cfg.Ts;
warm.Xaxis=zeros(model.n_axis_state,warm.N+1,2);
warm.U=zeros(2,warm.N);
oldStart=0; newStart=0;
for i=1:numel(Nvec)
    No=previous.Nvec(i); Nn=Nvec(i);
    xo=linspace(0,1,No+1); xn=linspace(0,1,Nn+1);
    uo=((0:No-1)+0.5)/No; un=((0:Nn-1)+0.5)/Nn;
    for axis=1:2
        for state=1:model.n_axis_state
            warm.Xaxis(state,newStart+(1:Nn+1),axis)=interp1(xo, ...
                previous.Xaxis(state,oldStart+(1:No+1),axis),xn,'pchip');
        end
        warm.U(axis,newStart+(1:Nn))=interp1(uo, ...
            previous.U(axis,oldStart+(1:No)),un,'linear','extrap');
    end
    oldStart=oldStart+No; newStart=newStart+Nn;
end
warm.time=(0:warm.N)*cfg.Ts;
end
