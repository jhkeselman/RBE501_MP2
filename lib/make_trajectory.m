function traj = make_trajectory(type, params)
    % MAKE_TRAJECTORY Generates cubic or quintic trajectories.
    %   TRAJ = MAKE_TRAJECTORY(TYPE, PARAMS) returns a trajectory of the given type (either cubic or quintic) for the given parameters.
    % Make cubic trajectory
    if strcmp(type, 'cubic')
        ti = params.t(1);
        tf = params.t(2);
        % Matrix to calculate trajectory coefficients
        tm = [1 ti ti^2 ti^3;
              0 1 2*ti 3*ti^2;
              1 tf tf^2 tf^3;
              0 1 2*tf 3*tf^2];
        a = tm\[params.q(1); params.v(1); params.q(2); params.v(2)];
        a0 = a(1); a1 = a(2); a2 = a(3); a3 = a(4);
        traj.t = [];
        traj.q = [];
        traj.v = [];
        traj.a = [];
        % Number of steps based on initial and final time and timestep
        d = (tf - ti) / params.time_step;
        ct = ti;
        % Calculates position, velocity, acceleration at each timestep
        for i = 1:d+1
            traj.t(i) = ct;
            traj.q(i) = a0 + a1*ct + a2*ct^2 + a3*ct^3;
            traj.v(i) = a1 + 2*a2*ct + 3*a3*ct^2;
            traj.a(i) = 2*a2 + 6*a3*ct;
            ct = ct + params.time_step;
        end
    end
    % Make quintic trajectory
    if strcmp(type, 'quintic')
        ti = params.t(1);
        tf = params.t(2);
        % Matrix to calculate trajectory coefficients
        tm = [1 ti ti^2 ti^3 ti^4 ti^5;
              0 1 2*ti 3*ti^2 4*ti^3 5*ti^4;
              0 0 2 6*ti 12*ti^2 20*ti^3;
              1 tf tf^2 tf^3 tf^4 tf^5;
              0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
              0 0 2 6*tf 12*tf^2 20*tf^3];
        a = tm\[params.q(1); params.v(1); params.a(1); params.q(2); params.v(2); params.a(2)];
        a0 = a(1);, a1 = a(2);, a2 = a(3);, a3 = a(4);, a4 = a(5); a5 = a(6);
        traj.t = [];
        traj.q = [];
        traj.v = [];
        traj.a = [];
        % Number of steps based on initial and final time and timestep
        d = (tf - ti) / params.time_step;
        ct = ti;
        % Calculates position, velocity, acceleration at each timestep
        for i = 1:d+1
            traj.t(i) = ct;
            traj.q(i) = a0 + a1*ct + a2*ct^2 + a3*ct^3 + a4*ct^4 + a5*ct^5;
            traj.v(i) = a1 + 2*a2*ct + 3*a3*ct^2 + 4*a4*ct^3 + 5*a5*ct^4;
            traj.a(i) = 2*a2 + 6*a3*ct + 12*a4*ct^2 + 20*a5*ct^3;
            ct = ct + params.time_step;
        end
    end
    
end