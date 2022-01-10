
function [time_vect,x_opt,u_opt,cost_opt] = run_indirect_single_shooting()

addpath('indirect_single_shooting')
% Indirect Single Shooting 


% weights
global q_1 q_2 r p_1 p_2
q_1 = 0.001;
q_2 = 0.001;
r = 0.001;
p_1 = 10;
p_2 = p_1;
P = [p_1 0 ; 0 p_2];
% State Dynamics
f_dyn =@(x,lambda)  [(1-x(2).^2).*x(1) - x(2) - lambda(1)/r; x(1)]; % Van Der Pol Oscillator

% Adj Dynamics
g_dyn=@(x,lambda) [(x(2).^2-1).*lambda(1) - lambda(2) - q_1 * x(1) ; (2*x(1)*x(2)+1)*lambda(1) - q_2 * x(2) ];

% Hamiltonian system

H_dyn =@(t,y) [f_dyn(y(1:2),y(3:4)) ; g_dyn(y(1:2),y(3:4))] ;

t_span = [0,1];
y_0 = [ 1 1 0 0 ]';

[TOUT,YOUT] = ode45(H_dyn,t_span,y_0);

% Sensitivity equations

get_A(0.2,TOUT,YOUT);

A =@(t) get_A(t,TOUT,YOUT);

S_0 = [ zeros(2,2) ; eye(2) ];
s_0 = S_0(:);

odefun = @(t,y) deriv(t,y,A(t));

[TOUT_s,s_OUT] = ode45(odefun,t_span,s_0);  
S = reshape(s_OUT.',4,2,[]);  
G = S(1:2,1:2,:);
L = S(3:4,1:2,:);


% Newton iterations

max_iter = 10;
lambda_old = [0.01 0.0001]';  % Very sensible to the initial guess
x_0 = [1 1]';
R_iter = [];

for ii=1:max_iter
    
    % Solve Hamiltonian System
    y_0 = [ x_0 ; lambda_old ]';
    [TOUT,YOUT] = ode45(H_dyn,t_span,y_0);
    
    % Solve Sensitivity Equations
    A =@(t) get_A(t,TOUT,YOUT);
    odefun = @(t,y) deriv(t,y,A(t));
    [TOUT_s,s_OUT] = ode45(odefun,t_span,s_0);  
    S = reshape(s_OUT.',4,2,[]);  
    G_f = S(1:2,1:2,end);  % sensitivity of state at final time
    L_f = S(3:4,1:2,end);  % sensitivity of adjoint at final time
    
    % Compute error
    R = YOUT(end,3:4)' - P*YOUT(end,1:2)' ;
    R_iter = [R_iter R];
    dR = L_f -P*G_f ;
    new_dir = -dR\R;
    % Newton full step
    lambda_new = lambda_old +new_dir;
    lambda_old = lambda_new;
    
end

% Solve Hamiltonian system with lambda_0^star

y_0 = [ x_0 ; lambda_old ]';
[TOUT,YOUT] = ode45(H_dyn,t_span,y_0);


u_opt = -1/r*YOUT(:,3);
x_opt = YOUT(:,1:2);
time_vect = TOUT;

L = 0.5*r*u_opt.^2 + 0.5*q_1*x_opt(:,1).^2+0.5*q_2*x_opt(:,2).^2;
cost_opt = 0.5*x_opt(end,:)*P*x_opt(end,:)' + trapz(TOUT,L);





function ds = deriv(t,y,A)

S = reshape(y,[4,2]); 
S_dot = A*S;  
ds = S_dot(:);

end

end









