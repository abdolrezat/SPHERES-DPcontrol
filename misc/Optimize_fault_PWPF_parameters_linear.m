function [x,f]  = Optimize_fault_PWPF_parameters_linear( )
% Optimize PWPF
%   optimizes parameters to get minimum of (1/2)*x'*Q*x + (1/2)*v'*Q*v 
% P0 = [1.5,0.2,0.6,0.3,0.8];
% P0 = [5.0000    0.6648    0.2094    0.0187    0.9641]; 
% P0 = [5.2505    0.6453    0.2016    0.0187    0.9328]; optimized for thruster#0 failed
% 5.3755    0.3953    0.3891    0.0187    0.9933 gives f=5.2742e+05
%35.6000    0.1453    0.3891    0.2687    0.8722 gives f=5.2715e+05 works for r > 5m 
%7.6000    0.3953    0.2387    0.2609    0.9972 optimized for r = 2 
%P_optim =
%    17.4992    0.1966    0.4009    0.6160    0.4733
N = 10;
% P0 = [35.6000    0.1453    0.3891    0.2687    0.8722];
lb =[0 0 0.2 0 0];
ub = [1 1 1 1 1];
% P0(1) = P0(1)/50; %scaled
Fval_optim = Inf;
tic
for i = 1:N 
P0 = lb + rand(size(lb)).*(ub - lb);

%% patternsearch algorithm
options = optimoptions('patternsearch','Display','iter','PlotFcn',@psplotbestf,...
    'InitialMeshSize',0.4,'MeshTolerance', 1e-2, 'StepTolerance', 1e-2, 'MaxMeshSize', 1, ...
    'UseParallel', true, 'UseCompletePoll', true, 'AccelerateMesh', true); %
[x,Fval,~,Output] = patternsearch(@objective_function,P0,[],[],[],[],lb,ub,[],options);
fprintf('The number of iterations was : %d\n', Output.iterations);
fprintf('The number of function evaluations was : %d\n', Output.funccount);
fprintf('The best function value found was : %g\n', Fval);

if(Fval < Fval_optim)
    Fval_optim = Fval
    P_optim = P0
end

end
toc

P_optim(1) = P_optim(1)*50
Fval_optim
%% fminsearch
% options_fmins = optimset('TolX',1e-2,'TolFun',1,'FiniteDifferenceStepSize',1e-1);
% x = fminsearch(@objective_function,P0,options_fmins);


function total_cost = objective_function(P)
P(1) = P(1)*50;
R0 = [-.5 0 0];

cR2 = 10;
cR3 = 30;
cR4 = 60; 
cost1 = run_simulator_faulty(R0, P) *cR4;
cost2 = run_simulator_faulty(R0*cR2, P)*cR4/cR2;
cost3 = run_simulator_faulty(R0*cR3, P)*cR4/cR3;
cost4 = run_simulator_faulty(R0*cR4, P);
% 
total_cost = cost1 + cost2 + cost3 + cost4;
fprintf('cost_path1: %.2g | cost_path2: %.2g | cost_path3: %.2g | cost_path4: %.2g, Total = %.2g\n',...
    cost1,cost2,cost3,cost4,total_cost)
disp(P)
fprintf('----------------------------------------------------------------------\n')

function trip_cost = run_simulator_faulty(dr0, P)
%runs the simulator for a faulty condition to optimize active set weights
Thruster_max_F = 0.12; % (N)
Thruster_dist = (9.65E-2); % (meters)
controllername = 'controller_attposition_9_longdistQ3_continuous';
controller.type = 'DP';

%% Simulation
    Thruster_max_F = 0.12; % (N)
    Thruster_dist = (9.65E-2); % (meters)
%simulator variables
    simulator_opts.mode = 'fault';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative
    simulator_opts.thruster_allocation_mode = 'PWPF'; % {'active set discrete', 'PWPF', 'Schmitt', 'none'}
    simulator_opts.faulty_thruster_index = [6]; %index of faulty thruster(s) #0-#11
    simulator_opts.current_controller = controllername;
    simulator_opts.controller_InterpmodeF = 'linear'; %interpolation mehod of F controller output
    simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
    simulator_opts.T_final = 100; %simulation Tfinal
    simulator_opts.h = 0.005; %simulation fixed time steps
    simulator_opts.Thruster_max_F = Thruster_max_F;
    simulator_opts.Thruster_dist = Thruster_dist;
    
    
    %initial state
    dv0 = [0 0 0]; %initial relative velocity offset
    q0 = flip(angle2quat(deg2rad(0),deg2rad(0),deg2rad(0))); %initial angles offset (yaw,pitch,roll)
    w0 = [0 0 0]; %initial rotational speed offset
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
    
    %active set parameter
    simulator_opts.active_set.Weighting_Matrix = [1,0;...
                                                  0,0];
    %PWPF and schmitt trigger configuration parameters
    simulator_opts.PWPF.Km = P(1);
    simulator_opts.PWPF.Tm = P(2);
    simulator_opts.PWPF.h = simulator_opts.h; 
    simulator_opts.PWPF.H_feed = P(3);
    
    simulator_opts.schmitt.Uout = Thruster_max_F;
    simulator_opts.schmitt.Uon = P(4)*simulator_opts.schmitt.Uout;
    simulator_opts.schmitt.Uoff = P(5)*simulator_opts.schmitt.Uon;
% create and run the simulator
SC = Simulator_CW(simulator_opts,controller);
history = SC.get_optimal_path_history();
trip_cost = derive_cost(history, 'XVU') + derive_cost(history, 'TWU');
x1_info = stepinfo(history.X_ode45(:,1),history.T_ode45,0);

% if isnan(x1_info.SettlingTime) 
%     trip_cost= NaN;
% end

SC.plot_optimal_path()

function C = derive_cost(history, type)
Qx = 10;
Qv = 45*Qx;
Qt = 0.01;
Qw = 0.8*Qt;
R = 1; 
DIST = (9.65E-2);

X = history.X_ode45;
U = history.F_Th_Opt;

if(strcmp(type,'XVU'))
C = sum( sum( X(:,1:3).^2 ))*Qx + ...
    sum( sum( X(:,4:6).^2 ))*Qv + ...
    sum( U(:))*R;
elseif(strcmp(type,'TWU'))
[theta1,theta2,theta3] = quat2angle(X(:,10:-1:7));

C = sum( theta1.^2 + theta2.^2 + theta3.^2 )*Qt + ...
    sum( sum( X(:,11:13).^2 ))*Qw + ...
    sum( U(:))*R/DIST;
end