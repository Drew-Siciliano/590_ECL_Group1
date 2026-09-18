function Z0_guess = generateReference(X0,r_target,N_nodes,u_max)

    % Optimization Z is [Tf;x;y;theta;vr;vl]

    % Make Sure all dimensions are ok
    X0 = X0(:);
    r_target = r_target(:);

    pos0 = X0(1:2);
    theta0 = X0(3);

    delta_pos = r_target-pos0;
    distance = norm(delta_pos,2);

    % Heading from the initial position to the target
    theta_line = atan2(delta_pos(2),delta_pos(1));

    % Choose the equivalent angle closest to theta0
    delta_theta = wrapToPi(theta_line-theta0);

    theta_line = theta0+delta_theta;

    % Reference speed
    v_guess = 0.9*u_max;
    Tf_guess = distance/v_guess;

    alpha = linspace(0,1,N_nodes);

    % Straight-line position reference
    x_guess = pos0(1)+alpha*delta_pos(1);
    y_guess = pos0(2)+alpha*delta_pos(2);

    % Reference Heading
    theta_guess = theta_line*ones(1,N_nodes);
    theta_guess(1) = theta0;

    % Reference control
    vr_guess = v_guess*ones(1,N_nodes);
    vl_guess = v_guess*ones(1,N_nodes);

    Z_nodes_guess = [x_guess;
                     y_guess;
                     theta_guess;
                     vr_guess;
                     vl_guess];

    Z0_guess = [Tf_guess;
                Z_nodes_guess(:)];
end