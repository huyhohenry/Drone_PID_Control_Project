
import math


def pid_thrust(target_elevation, drone_elevation, tau_p=0, tau_d=0, tau_i=0, data: dict() = {}):
    '''
    Student code for Thrust PID control. Drone's starting x, y position is (0, 0).

    Args:
    target_elevation: The target elevation that the drone has to achieve
    drone_elevation: The drone's elevation at the current time step
    tau_p: Proportional gain
    tau_i: Integral gain
    tau_d: Differential gain
    data: Dictionary that you can use to pass values across calls.
        Reserved keys:
            max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.

    Returns:
        Tuple of thrust, data
        thrust - The calculated thrust using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.
            Reserved keys:
                max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.
    '''

    if data.get("time_step") is None:
        data["time_step"] = 0
        data["cte"] = target_elevation - drone_elevation
        data["diff_cte"] = 0
        data["int_cte"] = 0
        data["thrust"] = 0
    else:
        data["diff_cte"] = (target_elevation - drone_elevation) - data.get("cte")
        data["cte"] = target_elevation - drone_elevation
        data["int_cte"] += target_elevation - drone_elevation
        # if drone_elevation <= target_elevation * .7:
        #     data["thrust"] = 50
        # else:
        data["thrust"] = tau_p * data.get("cte") + tau_d * data.get("diff_cte") + tau_i * data.get("int_cte")

    # while data.get("max_rpm_reached") is False:
    thrust = data.get("thrust")
    # thrust = data["time_step"]

    # thrust = 22
    data["time_step"] += 1
    # print("time:", data.get("time_step")," drone elev:", drone_elevation, "target elev:",target_elevation,"tau_p:",tau_p,"cte:",data.get("cte"),"tau_d:",tau_d,"diff_cte:",data.get("diff_cte"),"tau_i:",tau_i, "int_cte:",data.get("int_cte")," thrust:", data.get("thrust"))

    return thrust, data


def pid_roll(target_x, drone_x, tau_p=0, tau_d=0, tau_i=0, data: dict() = {}):
    '''
    Student code for PD control for roll. Drone's starting x,y position is 0, 0.

    Args:
    target_x: The target horizontal displacement that the drone has to achieve
    drone_x: The drone's x position at this time step
    tau_p: Proportional gain, supplied by the test suite
    tau_i: Integral gain, supplied by the test suite
    tau_d: Differential gain, supplied by the test suite
    data: Dictionary that you can use to pass values across calls.

    Returns:
        Tuple of roll, data
        roll - The calculated roll using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.

    '''
    if data.get("time_step") is None:
        data["time_step"] = 0
        data["cte"] = target_x - drone_x
        data["diff_cte"] = 0
        data["int_cte"] = 0
        data["roll"] = 0
    else:
        data["diff_cte"] = (target_x - drone_x) - data.get("cte")
        data["cte"] = target_x - drone_x
        data["int_cte"] += target_x - drone_x
        # if drone_elevation <= target_elevation * .7:
        #     data["thrust"] = 50
        # else:
        data["roll"] = tau_p * data.get("cte") + tau_d * data.get("diff_cte") + tau_i * data.get("int_cte")

    roll = data.get("roll")

    data["time_step"] += 1

    return roll, data


def find_parameters_thrust(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you can focus on
    tuning gain values for Thrust test cases only.

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''

    # Initialize a list to contain your gain values that you want to tune
    params = [20., 700., 0.01]
    # params = [100., 1760., 0.0001]
    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
        thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = hover_error + max(0, drone_max_velocity - 0.80 * max_allowed_velocity) * 1 + 1 * max(0,
                                                                                                    total_oscillations - max_allowed_oscillations)

    # Implement your code to use twiddle to tune the params and find the best_error
    dp = [1.0, 1.0, 1.0]
    # TODO: twiddle loop here
    n = 0
    tol = 0.000001
    while sum(dp) > tol:
        for i in range(len(params)):
            params[i] += dp[i]
            thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
            # drone = DroneSimulator()
            # hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                thrust_params, roll_params, VISUALIZE=VISUALIZE)
            # print('hover err: ', hover_error)
            # print('best err: ', best_error)
            new_error = hover_error + max(0, drone_max_velocity - 0.999 * max_allowed_velocity) * 1 + 1 * max(0,
                                                                                                                   total_oscillations - max_allowed_oscillations)
            if new_error < best_error:
                best_error = new_error
                dp[i] *= 1.1

                # print('been here')
            else:
                params[i] -= 2.0 * dp[i]
                thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                    thrust_params, roll_params, VISUALIZE=VISUALIZE)
                new_error = hover_error + max(0, drone_max_velocity - 0.999 * max_allowed_velocity) * 1 + 1 * max(
                    0, total_oscillations - max_allowed_oscillations)
                if new_error < best_error:
                    best_error = new_error
                    dp[i] *= 1.1
                else:
                    params[i] += dp[i]
                    dp[i] *= 0.9
        # print(params)
        n += 1

    # Return the dict of gain values that give the best error.
    # params = [20., 700., 0.00001]
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    print('hover_error', hover_error, 'max_allowed_velocity', max_allowed_velocity, 'drone_max_velocity', drone_max_velocity,'max_allowed_oscillations', max_allowed_oscillations, 'total_oscillations', total_oscillations)
    return thrust_params, roll_params


def find_parameters_with_int(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you can focus on
    tuning gain values for Thrust test case with Integral error

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''

    # Initialize a list to contain your gain values that you want to tune
    params = [20., 700., 0.01]
    # params = [100., 1760., 0.0001]
    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
        thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = hover_error + max(0, drone_max_velocity - 0.90 * max_allowed_velocity) * 1000 + 100 * max(0,
                                                                                                           total_oscillations - max_allowed_oscillations)

    # Implement your code to use twiddle to tune the params and find the best_error
    dp = [1.0, 1.0, 1.0]
    # TODO: twiddle loop here
    n = 0
    tol = 0.000001
    while sum(dp) > tol:
        for i in range(len(params)):
            params[i] += dp[i]
            thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
            # drone = DroneSimulator()
            # hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                thrust_params, roll_params, VISUALIZE=VISUALIZE)
            # print('hover err: ', hover_error)
            # print('best err: ', best_error)
            new_error = hover_error + max(0, drone_max_velocity - 0.90 * max_allowed_velocity) * 1000 + 1000 * max(0,
                                                                                                                   total_oscillations - max_allowed_oscillations)
            if new_error < best_error:
                best_error = new_error
                dp[i] *= 1.1

                # print('been here')
            else:
                params[i] -= 2.0 * dp[i]
                thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                    thrust_params, roll_params, VISUALIZE=VISUALIZE)
                new_error = hover_error + max(0, drone_max_velocity - 0.90 * max_allowed_velocity) * 1000 + 1000 * max(
                    0,
                    total_oscillations - max_allowed_oscillations)
                if new_error < best_error:
                    best_error = new_error
                    dp[i] *= 1.1
                else:
                    params[i] += dp[i]
                    dp[i] *= 0.9
        # print(params)
        n += 1

    # Return the dict of gain values that give the best error.
    # params = [20., 700., 0.00001]
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
    print('hover_error', hover_error, 'max_allowed_velocity', max_allowed_velocity, 'drone_max_velocity', drone_max_velocity,'max_allowed_oscillations', max_allowed_oscillations, 'total_oscillations', total_oscillations)
    return thrust_params, roll_params


def find_parameters_with_roll(run_callback, tune='both', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you will
    find gain values for Thrust as well as Roll PID controllers.

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''


    # Initialize a list to contain your gain values that you want to tune
    params = [20., 700., 0.01, -20., -700., -.01]
    # params = [20., 700., 0.01, -20., -500., -.01]
    # params = [1., 1760., 0.0001]
    # Create dicts to pass the parameters to run_callback
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    # If tuning roll, then also initialize gain values for roll PID controller
    roll_params = {'tau_p': params[3], 'tau_d': params[4], 'tau_i': params[5]}

    # Call run_callback, passing in the dicts of thrust and roll gain values
    hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
        thrust_params, roll_params, VISUALIZE=VISUALIZE)

    # Calculate best_error from above returned values
    best_error = hover_error + max(0, drone_max_velocity - 0.99 * max_allowed_velocity) * 1 + 1 * max(0,
                                                                                                           total_oscillations - max_allowed_oscillations)

    # Implement your code to use twiddle to tune the params and find the best_error
    dp = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    # TODO: twiddle loop here
    n = 0
    tol = 0.000001
    while sum(dp) > tol:
        for i in range(len(params)):
            params[i] += dp[i]
            thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
            roll_params = {'tau_p': params[3], 'tau_d': params[4], 'tau_i': params[5]}
            # drone = DroneSimulator()
            # hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(thrust_params, roll_params, VISUALIZE=VISUALIZE)
            hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                thrust_params, roll_params, VISUALIZE=VISUALIZE)
            # print('hover err: ', hover_error)
            # print('best err: ', best_error)
            new_error = hover_error + max(0, drone_max_velocity - 0.99 * max_allowed_velocity) * 1 + 1 * max(0,
                                                                                                                   total_oscillations - max_allowed_oscillations)
            if new_error < best_error:
                best_error = new_error
                dp[i] *= 1.1

                # print('been here')
            else:
                params[i] -= 2.0 * dp[i]
                thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
                roll_params = {'tau_p': params[3], 'tau_d': params[4], 'tau_i': params[5]}
                hover_error, max_allowed_velocity, drone_max_velocity, max_allowed_oscillations, total_oscillations = run_callback(
                    thrust_params, roll_params, VISUALIZE=VISUALIZE)
                new_error = hover_error + max(0, drone_max_velocity - 0.99 * max_allowed_velocity) * 1 + 1 * max(
                    0,
                    total_oscillations - max_allowed_oscillations)
                if new_error < best_error:
                    best_error = new_error
                    dp[i] *= 1.1
                else:
                    params[i] += dp[i]
                    dp[i] *= 0.9
        # print(params)
        n += 1

    # Return the dict of gain values that give the best error.
    # params = [20., 700., 0.00001]
    thrust_params = {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}
    roll_params = {'tau_p': params[3], 'tau_d': params[4], 'tau_i': params[5]}
    print('hover_error', hover_error, 'max_allowed_velocity', max_allowed_velocity, 'drone_max_velocity', drone_max_velocity,'max_allowed_oscillations', max_allowed_oscillations, 'total_oscillations', total_oscillations)
    return thrust_params, roll_params

