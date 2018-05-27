function [res]  = fit_reference_model_position(SC) %SC is a simulation set
% final points: [0.0676 0.0019] for 150s simulation (not well fitted)
% -with pulse modulation
%               x - [0.099819, 0.0029471] for 75-150s (well fit)
%               y - [0.1073    0.0032]
%               z - [0.1113    0.0035]
% -without pulse modulation
%               x - [0.3345    0.0180]
%               y - [0.3470    0.0182]
%               z - [0.3356    0.0180]
% sim('reference_model_for_fit')
init_point = [0.0432,5e-4];

[x1,~,~]  = fit_reference_model_position_channel(SC,1,init_point);
[x2,~,~]  = fit_reference_model_position_channel(SC,2,init_point);
[x3,~,~]  = fit_reference_model_position_channel(SC,3,init_point);
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
    'FunctionTolerance',1e-3,'StepTolerance',1e-3); %
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
    
    if( max(abs(acc_out.Data)) > 0.12/4.16/5/1.2)
        ee = 10000;
       return 
    end
    id = Xm_out.Time > 0;%settling_time;
    id_ts = Xm_out.Time > settling_time - 50;
    
%     e = Xm_out.Data(id) - FgI(Xm_out.Time(id)) ;
    ets =  Xm_out.Data(id_ts) - FgI(Xm_out.Time(id_ts));
% %         ee = e'*e + ets'*ets;
ee = ets'*ets;

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