function corner = calculate_corner_sagitta(solution,path,cfg)
%CALCULATE_CORNER_SAGITTA Maximum deviation in scheduled +/-2 mm windows.
n=path.n_segments; cursor=[0,cumsum(solution.Nvec)];
template=struct('index',0,'entry_time',0,'exit_time',0,'minimum_speed',0, ...
    'maximum_speed',0,'max_error',0,'max_vx',0,'max_vy',0, ...
    'max_acceleration',0,'max_jerk',0,'max_snap',0,'max_modal_response',0);
corner=repmat(template,n,1);
contour=calculate_contour_error(solution,path,cfg);
for j=1:n
    if j==1
        Nin=solution.Nvec(end); Nout=solution.Nvec(1);
        kin=max(1,solution.N-floor(cfg.geometry.transition_length/path.lengths(end)*Nin)+1):solution.N+1;
        kout=1:min(solution.N+1,ceil(cfg.geometry.transition_length/path.lengths(1)*Nout)+1);
        idx=unique([kin,kout]);
    else
        boundary=cursor(j)+1;
        Nin=solution.Nvec(j-1); Nout=solution.Nvec(j);
        left=ceil(cfg.geometry.transition_length/path.lengths(j-1)*Nin);
        right=ceil(cfg.geometry.transition_length/path.lengths(j)*Nout);
        idx=max(1,boundary-left):min(solution.N+1,boundary+right);
    end
    speed=vecnorm(solution.velocity(:,idx),2,1);
    corner(j).index=j;
    if j==1
        corner(j).entry_time=solution.time(kin(1));
        corner(j).exit_time=solution.T;
    else
        corner(j).entry_time=solution.time(idx(1));
        corner(j).exit_time=solution.time(idx(end));
    end
    corner(j).minimum_speed=min(speed); corner(j).maximum_speed=max(speed);
    corner(j).max_error=max(contour.error(idx));
    corner(j).max_vx=max(abs(solution.velocity(1,idx)));
    corner(j).max_vy=max(abs(solution.velocity(2,idx)));
    corner(j).max_acceleration=max(vecnorm(solution.acceleration(:,idx),2,1));
    corner(j).max_jerk=max(vecnorm(solution.jerk(:,idx),2,1));
    corner(j).max_snap=max(vecnorm(solution.snap(:,idx),2,1));
    if ~isempty(solution.modal)
        corner(j).max_modal_response=max(abs(solution.modal(:,:,idx)),[],'all');
    end
end
end
