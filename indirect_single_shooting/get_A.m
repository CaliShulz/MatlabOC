function [A] = get_A(t,TOUT,YOUT)

global q_1 q_2 r p_1 p_2

%compute sensitivity state matrix

Y_int = interp1(TOUT,YOUT,t); % interpolate value of state and adjoint

x_1 = Y_int(1);
x_2 = Y_int(2);
lambda_1 = Y_int(3);

A = zeros(4,4);

A(1,1) = 1-x_2^2;
A(1,2) = -2*x_1*x_2-1;
A(1,3) = -1/r;
A(2,1) = 1;
A(3,1) = -q_1;
A(3,2) = 2*x_2*lambda_1;
A(3,3) = x_2^2-1;
A(3,4) = -1;
A(4,1) = 2*x_2*lambda_1;
A(4,2) = 2*x_1*lambda_1-q_2;
A(4,3) = 2*x_1*x_2+1;



end

