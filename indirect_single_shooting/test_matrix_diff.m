clear all
clc
close all

A = rand(2,2);  % Some arbitrary matrix we will use
F0 = eye(2);  % Some arbitrary matrix initial value
odefun = @(t,y) deriv(t,y,A);  % Anonymous derivative function with A
tspan = [0 5];
[T,F] = ode45(odefun,tspan,F0(:));  % Pass in column vector initial value
F = reshape(F.',2,2,[]);  % Reshape the output as a sequence of 2x2 matrices
n = size(F,3);
e = zeros(2,n);
for k=1:n
    e(:,k) = eig(F(:,:,k));  % Calculate the eigenvalues of the 2x2 matrices
end
plot(T,e(1,:),T,e(2,:)); grid on  % Plot them
xlabel('Time')
ylabel('Eigenvalues')

function dy = deriv(t,y,A)
F = reshape(y,size(A));  % Reshape input y into matrix
FA = F*A;  % Do the matrix multiply
dy = FA(:);  % Reshape output as a column vector
end