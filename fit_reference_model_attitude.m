function [res]  = fit_reference_model_attitude(SC) %SC is a simulation set
% final points: [0.0676 0.0019] for 150s simulation (not well fitted)
%               x - [0.099819, 0.0029471] for 75-150s (well fit)
%               y - [0.1073    0.0032]
%               z - [0.1113    0.0035]
%               t1 - [4.8930    0.9973]
%               t2 - [4.8639    1.0804]
%               t3 - [4.5605    1.3111]

% fit curve to data
init_point = [4.5605    1.3111];
set_param('reference_model_for_fit_attitude/s_dot','InitialCondition','0')

q = SC.history.X_ode45(:,7:10); %choose x,y,z,t1,t2,t3 to fit
MRPs = q(:,1:3)./(1+q(:,4));
t = SC.history.T_ode45(:,1);
FgI1 = griddedInterpolant(t,MRPs(:,1),'linear','linear');
FgI2 = griddedInterpolant(t,MRPs(:,2),'linear','linear');
FgI3 = griddedInterpolant(t,MRPs(:,3),'linear','linear');
% plot(t,x), hold on, plot(t,FgI(t),'r')

[m1,~,~]  = fit_reference_model_attitude_channel(FgI1, init_point); %SC is a simulation set
[m2,~,~]  = fit_reference_model_attitude_channel(FgI2, init_point); %SC is a simulation set
[m3,~,~]  = fit_reference_model_attitude_channel(FgI3, init_point); %SC is a simulation set

res = [m1;m2;m3]';


end

function [x,Fval,Output]  = fit_reference_model_attitude_channel(FgI, init_point) %SC is a simulation set

set_param('reference_model_for_fit_attitude/sigma','InitialCondition',num2str(FgI(0)))

K0 = init_point*100; %  initial point
lb = [0 0];

%% patternsearch
tic
options = optimoptions('patternsearch','Display','iter',...
    'FunctionTolerance',1e-2,'StepTolerance',1e-2); %
[x,Fval,~,Output] = patternsearch(@obj_fun,K0,[],[],[],[],lb,[],[],options);
toc

    function ee = obj_fun(K)
        ee =  simulate_refmodel(K,FgI);
    end

x = x/100;
%% show result
p_results(FgI)


end

function ee = simulate_refmodel(K, FgI)
    K1 = K(1)/100;
    K2 = K(2)/100; 
    set_param('reference_model_for_fit_attitude/GainK3','Gain',num2str(K1))
    set_param('reference_model_for_fit_attitude/GainK4','Gain',num2str(K2))
    sim('reference_model_for_fit_attitude')
    id = Xs_out.Time > 9;
    
    if( max(abs(acc_out.Data)) > 0.12*0.0965/0.0214/5/1.2)
        ee = 10000;
       return 
    end
    
    e = Xs_out.Data(id) - FgI(Xs_out.Time(id)) ;
        ee = e'*e;
end

function x1_info = p_results(FgI)
sim('reference_model_for_fit_attitude')
 figure
 plot(Xs_out.Time, Xs_out.Data,'b-')
 hold on
plot(Xs_out.Time, FgI(Xs_out.Time) , 'r--')
legend('simulated','desired')
x1_info = stepinfo(Xs_out.Data,Xs_out.Time,0)

end