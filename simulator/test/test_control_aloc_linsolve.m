function test_control_aloc_linsolve
% syms f0 f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 f11 Fx Fy Fz real
% P = equationsToMatrix(f0 + f1 -f6 -f7 == Fx, f2 + f3 -f8 -f9 == Fy, f4 + f5 - f10 - f11 == Fz,[f0 f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11]);
% B = [0,0,0,0,1,-1,0,0,0,0,-1,1;1,-1,0,0,0,0,-1,1,0,0,0,0;0,0,1,-1,0,0,0,0,-1,1,0,0];
% A = double([P;B]);
% F = [f0 f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11]';
U = [0.2;0.2;0.1;-0;0.1;0];
Ax = [1 1 -1 -1;1 -1 -1 1];
Ux = [U(1);U(4)];

%linsolve
ans1 = linsolve(Ax,Ux);
verify_ans1 = Ax*ans1;
%Pseudo-Inverse
ans2 = Ax' /(Ax*Ax') * Ux; % A' * inv(A*A')
verify_ans2 = Ax*ans2;

%direct solve
ans3 = Ax\Ux;
verify_ans3 = Ax*ans3;

%% linprog
max__oneT = 0.13;
max_T = 0.26;
max_M = 0.26;

f_A = [1 1 1 1];
lb = 0*f_A;
ub = max__oneT*f_A;
ans4 = linprog(f_A,[],[],Ax,Ux,lb,ub);
verify_ans4 = Ax*ans4;
keyboard
%% linprog2
options = optimoptions(@linprog,'Algorithm','dual-simplex','Display','iter');
[x,fval,exitflag,output] = ...
    linprog(f_A,[],[],Ax,Ux,lb,ub,options);


%% fmincon
objectivef = @(x)(x*x');
[ans5,fval,exitflag,output,lambda,grad,hessian] = fmincon_sol([0 0 0 0],Ax,Ux,lb,ub);
ans5 = ans5';
verify_ans5 = Ax*ans5;

%results
disp([ans1,ans2,ans3,ans4])





function [x,fval,exitflag,output,lambda,grad,hessian] = fmincon_sol(x0,Aeq,beq,lb,ub)
%% This is an auto generated MATLAB file from Optimization Tool.

%% Start with the default options
options = optimoptions('fmincon');
%% Modify options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'PlotFcn', {  @optimplotx @optimplotfunccount @optimplotfval });
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(@(x)(x*x'),x0,[],[],Aeq,beq,lb,ub,[],options);

function [f0,f1,f6,f7] = custom_thruster_aloc(F,M)
if(F > 0)
    f6 = 0;
    f7 = 0;
    A01 = [1 1;1 -1];
    temp = A01\[F;M];
    if(any(temp > 0.13) )
        
    end
    
else
    
end
        

