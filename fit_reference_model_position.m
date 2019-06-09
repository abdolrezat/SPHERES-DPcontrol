function [res]  = fit_reference_model_position(SC) %SC is a simulation set
% final points: [0.0676 0.0019] for 150s simulation (not well fitted)
% -with pulse modulation
% embedded in this code for the Simulink Model
% RM.K1 = diag([0.03440625	0.031953125	0.033859375]);
% RM.K2 = 3.7e-4* diag([1, 1, 1]);
% RM.K3 = diag([3.31050000000000	4.60050000000000	3.23050000000000]);
% RM.K4 = diag([0.471100000000000	0.693600000000000	0.481100000000000]);

% -without pulse modulation
%               x - [0.3345    0.0180]
%               y - [0.3470    0.0182]
%               z - [0.3356    0.0180]
sim('reference_model_for_fit')
set_param('reference_model_for_fit', 'StopTime', num2str(SC.history.T_ode45(end) +0.01))
% init_point = [0.0432,5e-4];
init_point1 = [0.03475, 0.00037];
init_point2 = [ 0.03228125,0.00037];
init_point3 = [0.03409375 ,0.00037];
[x1,~,~]  = fit_reference_model_position_channel(SC,1,init_point1);
[x2,~,~]  = fit_reference_model_position_channel(SC,2,init_point2);
[x3,~,~]  = fit_reference_model_position_channel(SC,3,init_point3);
res = [x1;x2;x3]';

end

function [x,Fval,Output]  = fit_reference_model_position_channel(SC,i,init_point) %SC is a simulation set, i is channel id
% fit curve to data
%% choose x,y,z, to fit
X = SC.history.X_ode45(:,i); 
%set initial condition of the model
% sim('reference_model_for_fit')
set_param('reference_model_for_fit/Integrator3',...
        'InitialCondition',num2str(X(1)))

t = SC.history.T_ode45(:,1);
FgI = griddedInterpolant(t,X,'linear','linear');
% plot(t,x), hold on, plot(t,FgI(t),'r')

K0 = init_point*1000; %  initial point
lb = [0 0];
ub = [200 200];
x1_info = stepinfo(X,SC.history.T_ode45,0);
settling_time = x1_info.SettlingTime;
%% patternsearch
tic
options = optimoptions('patternsearch','Display','iter','PlotFcn',@psplotbestf,...
    'FunctionTolerance',1e-2,'StepTolerance',1e-3); %
[x,Fval,~,Output] = patternsearch(@obj_fun,K0,[],[],[],[],lb,ub,[],options);
toc

    function ee = obj_fun(K)
        ee =  simulate_refmodel(K,FgI,settling_time);
    end

x = x/1000;
%% show result
x_desired_info = x1_info
p_results(FgI)


end

function ee = simulate_refmodel(K, FgI, settling_time)
    K1 = K(1)/1000;
    K2 = K(2)/1000; 
    set_param('reference_model_for_fit/GainK1','Gain',num2str(K1))
    set_param('reference_model_for_fit/GainK2','Gain',num2str(K2))
    sim('reference_model_for_fit')
    
    if( max(abs(acc_out.Data)) > 0.12/4.16/5/1.1)
        ee = 10000;
       return 
    end
    id = Xm_out.Time > 0;%settling_time;
    id_ts = Xm_out.Time > settling_time;
    X_ts = Xm_out.Data(id_ts);
    T_ts = Xm_out.Time(id_ts);
%     e = Xm_out.Data(id) - FgI(Xm_out.Time(id)) ;
    e = ( X_ts - FgI(T_ts) )  ;
% %         ee = e'*e + ets'*ets;
ee = e'*e + e(1)*e(1)*1000;

end

function x_sim_info = p_results(FgI)
sim('reference_model_for_fit')
 figure
 plot(Xm_out.Time, Xm_out.Data,'b-')
 hold on
plot(Xm_out.Time, FgI(Xm_out.Time) , 'r--')
legend('simulated','desired')
x_sim_info = stepinfo(Xm_out.Data,Xm_out.Time,0)

end