function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
    pendulum.angle_previous = pendulum.angle
    pendulum.angle_dot_previous = pendulum.angle_dot
    var acc = pendulum_acceleration(pendulum, gravity)
    pendulum.angle = pendulum.angle_previous + pendulum.angle_dot_previous * dt
    pendulum.angle_dot = pendulum.angle_dot_previous + acc * dt
    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration
    var angle = pendulum.angle
    var acc = pendulum_acceleration(pendulum, gravity)

    pendulum.angle = 2 * pendulum.angle - pendulum.angle_previous + acc * Math.pow(dt, 2)
    pendulum.angle_dot = (pendulum.angle - pendulum.angle_previous) / (2 * dt);
    pendulum.angle_previous = angle 
    

    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment
    pendulum.angle_previous = pendulum.angle
    var acc = pendulum_acceleration(pendulum, gravity)

    pendulum.angle = pendulum.angle_previous + pendulum.angle_dot * dt + 0.5 * Math.pow(dt, 2) * acc;
    pendulum.angle_dot = pendulum.angle_dot + 0.5 * dt * (acc + pendulum_acceleration(pendulum, gravity));

    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
    var s = 4.0
    var a21 = 0.5
    var a32 = 0.5
    var a43 = 1.0
    var c1 = 0.0
    var c2 = 0.5
    var c3 = 0.5
    var c4 = 1.0
    var b1 = 1.0/6
    var b2 = 1.0/3
    var b3 = 1.0/3
    var b4 = 1.0/6

    pendulum.angle_previous = pendulum.angle;
    pendulum.angle_dot_previous = pendulum.angle_dot;
    pendulum.angle_dot_dot_previous = pendulum_acceleration( pendulum, gravity );

    k1_angle = pendulum.angle_previous
    k1_angle_dot = pendulum.angle_dot_previous
    pendulum.angle = k1_angle
    k1_angle_dot_dot = pendulum_acceleration(pendulum, gravity)
    k2_angle = k1_angle + a21 * k1_angle_dot * dt
    k2_angle_dot = k1_angle_dot + a21 * k1_angle_dot_dot * dt
    pendulum.angle = k2_angle
    k2_angle_dot_dot = pendulum_acceleration(pendulum, gravity)
    k3_angle = k1_angle + a32 *  k2_angle_dot * dt
    k3_angle_dot = k1_angle_dot + a32 * k2_angle_dot_dot * dt
    pendulum.angle = k3_angle
    k3_angle_dot_dot = pendulum_acceleration(pendulum, gravity)
    k4_angle = k1_angle + a43 *  k3_angle_dot * dt
    k4_angle_dot = k1_angle_dot + a43 * k3_angle_dot_dot * dt
    pendulum.angle = k4_angle
    k4_angle_dot_dot = pendulum_acceleration(pendulum, gravity)

    pendulum.angle = pendulum.angle_previous + dt * (b1 * k1_angle_dot + b2 * k2_angle_dot + b3 * k3_angle_dot + b4 * k4_angle_dot)
    pendulum.angle_dot = pendulum.angle_dot_previous + dt * (b1 * k1_angle_dot_dot + b2 * k2_angle_dot_dot + b3 * k3_angle_dot_dot + b4 * k4_angle_dot_dot)
    pendulum.angle_dot_dot = pendulum_acceleration( pendulum, gravity )
    } 
    else {
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = (pendulum.angle+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot = (pendulum.angle-pendulum.angle_previous)/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion
    return  - (gravity / pendulum.length) * Math.sin(pendulum.angle) + pendulum.control / (pendulum.mass * Math.pow(pendulum.length,2)) 
    
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time
    pendulum.angle_previous = pendulum.angle
    pendulum.angle = pendulum.angle + pendulum.angle_dot * dt + pendulum_acceleration(pendulum, gravity) * Math.pow(dt,2) * 0.5
    t = t + dt


    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    pendulum.servo = {kp:500, kd:200, ki:5};  // no control
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error
    accumulated_error += pendulum.desired - pendulum.angle;
    pendulum.control = (pendulum.desired - pendulum.angle) * pendulum.servo.kp + dt * accumulated_error * pendulum.servo.ki + (pendulum.angle_previous - pendulum.angle) * pendulum.servo.kd / dt;
    return [pendulum, accumulated_error];
}