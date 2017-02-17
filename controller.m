function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
tol = 1e-6;

% Position Controller/Trajectory Follower Parameters
Kdx = 8; Kpx = 16;
Kdy = 8; Kpy = 16;
Kdz = 20; Kpz = 100;

dr = (des_state.pos-state.pos);
if norm(des_state.vel) >= tol
    t_hat = des_state.vel/norm(des_state.vel);
else
    t_hat = des_state.vel;
end
if norm(des_state.acc) >= tol
    n_hat = des_state.acc/norm(des_state.acc);
else
    n_hat = des_state.acc;
end
b_hat = cross(t_hat,n_hat);

ep = (dr'*n_hat)*n_hat + (dr'*b_hat)*b_hat + (dr'*t_hat)*t_hat;
ev = des_state.vel-state.vel;

rddot_des = des_state.acc + [Kdx;Kdy;Kdz].*ev + [Kpx;Kpy;Kpz].*ep;

% Attitude Controller Parameters
Kpphi = 74.9948; Kdphi = 0.2750;
Kptheta = 18.5600; Kdtheta = 0.1392;
Kppsi = 22.4264; Kdpsi = 0.1869;

p_des = 0; q_des = 0; r_des = des_state.yawdot;
psi_des = des_state.yaw;
phi_des = 1/params.gravity*(rddot_des(1)*sin(psi_des)-rddot_des(2)*cos(psi_des)); 
theta_des = 1/params.gravity*(rddot_des(1)*cos(psi_des)+rddot_des(2)*sin(psi_des)); 

% Thrust
F = params.mass*(params.gravity+rddot_des(3));

% Moment
M = [Kpphi*(phi_des-state.rot(1))+Kdphi*(p_des-state.omega(1));
      Kptheta*(theta_des-state.rot(2))+Kdtheta*(q_des-state.omega(2));
      Kppsi*(psi_des-state.rot(3))+Kdpsi*(r_des-state.omega(3))];
% M = zeros(3,1);

% =================== Your code ends here ===================

end
