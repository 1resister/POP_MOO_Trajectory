function options = configure_ipopt(cfg)
%CONFIGURE_IPOPT One authoritative IPOPT configuration.
options.expand = cfg.solver.expand;
options.print_time = false;
options.ipopt.max_iter = cfg.solver.ipopt.max_iter;
options.ipopt.max_cpu_time = cfg.solver.ipopt.max_cpu_time;
options.ipopt.tol = cfg.solver.ipopt.tol;
options.ipopt.acceptable_tol = cfg.solver.ipopt.acceptable_tol;
options.ipopt.acceptable_iter = cfg.solver.ipopt.acceptable_iter;
options.ipopt.print_level = cfg.solver.ipopt.print_level;
options.ipopt.sb = 'yes';
options.ipopt.warm_start_init_point = 'yes';
options.ipopt.warm_start_bound_push = 1e-7;
options.ipopt.warm_start_mult_bound_push = 1e-7;
options.ipopt.mu_init = 1e-4;
end
