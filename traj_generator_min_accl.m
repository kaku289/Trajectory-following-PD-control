function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
%


%% Fill in your code here

persistent waypoints0 traj_time d0 timeStamps alpha Tc
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    Tc = 10; % Completion time in s
    d0 = sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2); % Distances among each point
    timeStamps = d0/sum(d0)*Tc;
    traj_time = [0, cumsum(timeStamps)];
    waypoints0 = waypoints;
    
    % % Minimum acceleration trajectories
    terms = 4; % Number of coefficients in one polynomial
    n = size(waypoints,2)-1; % Number of polynomials
    
    % Construction of A and b matrices
    A = zeros(terms*n,terms*n);
    b = zeros(terms*n,3); % 3 colums corresponding to [x y z] coordinates
    agg_cons = 0; % Counter to keep track of constraint number being worked on
    
    % % Filling A and b matrices
    % constraints corresponding to known locations in space at specified times  (2n in number)
    for ct=1:n
        A(2*(ct-1)+1,terms*(ct-1)+1:terms*ct) = ((traj_time(ct)-traj_time(ct))/timeStamps(ct)).^[0:terms-1];
        A(2*(ct-1)+2,terms*(ct-1)+1:terms*ct) = ((traj_time(ct+1)-traj_time(ct))/timeStamps(ct)).^[0:terms-1];
        b(2*(ct-1)+1,:) = waypoints0(:,ct);
        b(2*(ct-1)+2,:) = waypoints0(:,ct+1);
    end
    agg_cons = n*2;
    
    % At start - vel = 0
    A(agg_cons+1,2:terms) = ([1:terms-1]/timeStamps(1)).*((traj_time(1)-traj_time(1))/timeStamps(1)).^[0:terms-2];
%     A(agg_cons+2,3:terms) = ([2:terms-1].*[1:terms-2]/timeStamps(1)*(traj_time(1)-traj_time(1))/timeStamps(1)).^[0:terms-3];
%     A(agg_cons+3,4:terms) = ([3:terms-1].*[2:terms-2].*[1:terms-3]/timeStamps(1)*(traj_time(1)-traj_time(1))/timeStamps(1)).^[0:terms-4];
    
    % At end - vel,acc,jerk = 0
    A(agg_cons+2,terms*n-terms+2:end) = ([1:terms-1]/timeStamps(end)).*((traj_time(end)-traj_time(end-1))/timeStamps(end)).^[0:terms-2];
%     A(agg_cons+5,terms*n-terms+3:end) = ([2:terms-1].*[1:terms-2]/timeStamps(end)*(traj_time(end)-traj_time(end-1))/timeStamps(end)).^[0:terms-3];
%     A(agg_cons+6,terms*n-terms+4:end) = ([3:terms-1].*[2:terms-2].*[1:terms-3]/timeStamps(end)*(traj_time(end)-traj_time(end-1))/timeStamps(end)).^[0:terms-4];
    
    agg_cons = 2+agg_cons;
    % Continuous velocity curves
    for ct=1:n-1
        A(agg_cons+ct,terms*(ct-1)+2:terms*ct) = ([1:terms-1]/timeStamps(ct)).*((traj_time(ct+1)-traj_time(ct))/timeStamps(ct)).^[0:terms-2]; 
        A(agg_cons+ct,terms*(ct)+2:terms*ct+terms) = (-[1:terms-1]/timeStamps(ct+1)).*((traj_time(ct+1)-traj_time(ct+1))/timeStamps(ct+1)).^[0:terms-2]; 
    end
    
    agg_cons = agg_cons + n-1;
    % Continuous acceleration curves
    for ct=1:n-1
        A(agg_cons+ct,terms*(ct-1)+3:terms*ct) = ([2:terms-1].*[1:terms-2]/timeStamps(ct)/timeStamps(ct)).*((traj_time(ct+1)-traj_time(ct))/timeStamps(ct)).^[0:terms-3]; 
        A(agg_cons+ct,terms*(ct)+3:terms*ct+terms) = (-[2:terms-1].*[1:terms-2]/timeStamps(ct+1)/timeStamps(ct+1)).*((traj_time(ct+1)-traj_time(ct+1))/timeStamps(ct+1)).^[0:terms-3]; 
    end
%     agg_cons = agg_cons + n-1;
    
    alpha = A\b;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    elseif(t <=10)
        t_index = find(traj_time >= t,1); % Polynomial to be used =t_index-1
        q = (t-traj_time(t_index-1))/timeStamps(t_index-1);
        alphas = alpha((t_index-2)*4+1:(t_index-2)*4+4,:);
        x = (q).^[0:3]*alphas(:,1);
        y = (q).^[0:3]*alphas(:,2);
        z = (q).^[0:3]*alphas(:,3);
        desired_state.pos = [x;y;z];
        
        xdot = ([1:3]/timeStamps(t_index-1)).*((q).^[0:2])*alphas(2:end,1);
        ydot = ([1:3]/timeStamps(t_index-1)).*((q).^[0:2])*alphas(2:end,2);
        zdot = ([1:3]/timeStamps(t_index-1)).*((q).^[0:2])*alphas(2:end,3);
        desired_state.vel = [xdot;ydot;zdot];
        
        xddot = ([2:3].*[1:2]/timeStamps(t_index-1)/timeStamps(t_index-1)).*((q).^[0:1])*alphas(3:end,1);
        yddot = ([2:3].*[1:2]/timeStamps(t_index-1)/timeStamps(t_index-1)).*((q).^[0:1])*alphas(3:end,2);
        zddot = ([2:3].*[1:2]/timeStamps(t_index-1)/timeStamps(t_index-1)).*((q).^[0:1])*alphas(3:end,3);
        desired_state.acc = [xddot;yddot;zddot];
    elseif(t<=Tc+1)
        desired_state.pos = waypoints0(:,end);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    end
    
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
end

