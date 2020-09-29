function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
    pendulum.angle_previous = pendulum.angle
    pendulum.angle_dot_previous = pendulum.angle_dot
    var acc = pendulum_acceleration(pendulum, gravity)
    pendulum.angle[0] = pendulum.angle_previous[0] + pendulum.angle_dot_previous[0] * dt
    pendulum.angle[1] = pendulum.angle_previous[1] + pendulum.angle_dot_previous[1] * dt
    pendulum.angle_dot[0] = pendulum.angle_dot_previous[0] + acc[0] * dt
    pendulum.angle_dot[1] = pendulum.angle_dot_previous[1] + acc[1] * dt
    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration
    var angle = pendulum.angle
    var acc = pendulum_acceleration(pendulum, gravity)

    pendulum.angle[0] = 2 * pendulum.angle[0] - pendulum.angle_previous[0] + acc[0] * Math.pow(dt, 2)
    pendulum.angle[1] = 2 * pendulum.angle[1] - pendulum.angle_previous[1] + acc[1] * Math.pow(dt, 2)
    pendulum.angle_dot[0] = (pendulum.angle[0] - pendulum.angle_previous[0]) / (2 * dt);
    pendulum.angle_dot[1] = (pendulum.angle[1] - pendulum.angle_previous[1]) / (2 * dt);
    pendulum.angle_previous = angle 
    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment

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
    pendulum.angle_dot_dot_previous = pendulum_acceleration(pendulum, gravity);

    k1_angle = pendulum.angle_previous
    k1_angle_dot = pendulum.angle_dot_previous
    pendulum.angle = k1_angle
    k1_angle_dot_dot = pendulum_acceleration(pendulum, gravity)
    k2_angle = [k1_angle[0] + a21 * k1_angle_dot[0] * dt, k1_angle[1] + a21 * k1_angle_dot[1] * dt];
    k2_angle_dot = [ k1_angle_dot[0] + a21 * dt * k1_angle_dot_dot[0], k1_angle_dot[1] + a21 * dt * k1_angle_dot_dot[1] ];
    pendulum.angle = [ k2_angle[0], k2_angle[1] ];
    k2_angle_dot_dot = pendulum_acceleration ( pendulum, gravity );

    k3_angle_dot = [ k1_angle_dot[0] + a32 * dt * k2_angle_dot_dot[0], k1_angle_dot[1] + a32 * dt * k2_angle_dot_dot[1] ];
    k3_angle = [ k1_angle[0] + a32 * dt * k2_angle_dot[0], k1_angle[1] + a32 * dt * k2_angle_dot[1] ];
    pendulum.angle = [ k3_angle[0], k3_angle[1] ];
    k3_angle_dot_dot = pendulum_acceleration ( pendulum, gravity );

    k4_angle_dot = [ k1_angle_dot[0] + a43 * dt * k3_angle_dot_dot[0], k1_angle_dot[1] + a43 * dt * k3_angle_dot_dot[1] ];
    k4_angle = [ k1_angle[0] + a43 * dt * k3_angle_dot[0], k1_angle[1] + a43 * dt * k3_angle_dot[1] ];
    pendulum.angle = [ k4_angle[0], k4_angle[1] ];
    k4_angle_dot_dot = pendulum_acceleration ( pendulum, gravity );
    sum_angle_dot_dot = [ 0.0, 0.0 ];
    sum_angle_dot = [ 0.0, 0.0 ];
    sum_angle_dot_dot[0] = b1 * k1_angle_dot_dot[0] + b2 * k2_angle_dot_dot[0] + b3 * k3_angle_dot_dot[0] + b4 * k4_angle_dot_dot[0];
    sum_angle_dot_dot[1] = b1 * k1_angle_dot_dot[1] + b2 * k2_angle_dot_dot[1] + b3 * k3_angle_dot_dot[1] + b4 * k4_angle_dot_dot[1];
    sum_angle_dot[0] = b1 * k1_angle_dot[0] + b2 * k2_angle_dot[0] + b3 * k3_angle_dot[0] + b4 * k4_angle_dot[0];
    sum_angle_dot[1] = b1 * k1_angle_dot[1] + b2 * k2_angle_dot[1] + b3 * k3_angle_dot[1] + b4 * k4_angle_dot[1];
    pendulum.angle_dot = [ pendulum.angle_dot_previous[0] + dt * sum_angle_dot_dot[0], pendulum.angle_dot_previous[1] + dt * sum_angle_dot_dot[1] ];
    pendulum.angle = [ pendulum.angle_previous[0] +  dt * sum_angle_dot[0], pendulum.angle_previous[1] +  dt * sum_angle_dot[1] ];
    pendulum.angle_dot_dot = pendulum_acceleration  ( pendulum, gravity );

    } 
    else {
        pendulum.angle_previous[0] = pendulum.angle[0];
        pendulum.angle[0] = (pendulum.angle[0]+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot[0] = (pendulum.angle[0]-pendulum.angle_previous[0])/dt;
        pendulum.angle_previous[1] = pendulum.angle[1];
        pendulum.angle[1] = (pendulum.angle[1]-Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot[1] = (pendulum.angle[1]-pendulum.angle_previous[1])/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    var A1 = (pendulum.mass[0]+pendulum.mass[1]) * pendulum.length[0]
    var B1 = pendulum.mass[1] * pendulum.length[1] * Math.cos(pendulum.angle[0] - pendulum.angle[1])
    var C1 = pendulum.mass[1] * pendulum.length[1] * Math.pow(pendulum.angle_dot[1],2) * Math.sin(pendulum.angle[0] - pendulum.angle[1]) + (pendulum.mass[0] + pendulum.mass[1]) * gravity * Math.sin(pendulum.angle[0]) - pendulum.control[0] / pendulum.length[0]
    var A2 = pendulum.length[0] * Math.cos(pendulum.angle[0] - pendulum.angle[1])
    var B2 = pendulum.length[1]
    var C2 = -pendulum.length[0] * Math.pow(pendulum.angle_dot[0],2) * Math.sin(pendulum.angle[0] - pendulum.angle[1]) + gravity * Math.sin(pendulum.angle[1]) - pendulum.control[1] / (pendulum.mass[1] * pendulum.length[1]);
    var a1 = (B1*C2-B2*C1)/(A1*B2-B1*A2);
    var a2 = (C1*A2-C2*A1)/(A1*B2-B1*A2);

    return [a1, a2]    
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time
    pendulum.angle_previous = pendulum.angle
    pendulum.angle[0] = pendulum.angle[0] + pendulum.angle_dot[0] * dt + pendulum_acceleration(pendulum, gravity)[0] * Math.pow(dt,2) * 0.5
    pendulum.angle[1] = pendulum.angle[1] + pendulum.angle_dot[1] * dt + pendulum_acceleration(pendulum, gravity)[1] * Math.pow(dt,2) * 0.5
    t = t + dt
    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    pendulum.servo = {kp:[0,0], kd:[0,0], ki:[0,0]};  // no control
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error

    return [pendulum, accumulated_error];
}