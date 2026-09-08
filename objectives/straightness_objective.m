function value = straightness_objective(varargin)
%STRAIGHTNESS_OBJECTIVE Dimensionless transverse-velocity integral.
% Numeric form: straightness_objective(solution,path,cfg).
% Symbolic form: straightness_objective(vx,vy,path,Nvec,cfg).
if nargin==3
    solution=varargin{1}; path=varargin{2}; cfg=varargin{3};
    vx=solution.velocity(1,:); vy=solution.velocity(2,:); Nvec=solution.Nvec;
else
    vx=varargin{1}; vy=varargin{2}; path=varargin{3};
    Nvec=varargin{4}; cfg=varargin{5};
end
value=0; cursor=0;
for i=1:path.n_segments
    Ni=Nvec(i); idx=cursor+(1:Ni+1);
    nominal=linspace(0,path.lengths(i),Ni+1);
    d=min(nominal,path.lengths(i)-nominal);
    xi=min(1,max(0,d/cfg.geometry.transition_length));
    weight=3*xi.^2-2*xi.^3; % 0 at corner, 1 in line core
    n=path.segment(i).normal(1:2);
    vperp=(n(1)*vx(idx)+n(2)*vy(idx))/cfg.limits.Vmax;
    value=value+cfg.Ts*sum(weight.*(vperp.^2));
    cursor=cursor+Ni;
end
end
