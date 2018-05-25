Q = load('sigTSinterpd.mat');
t = Q.tsinterpd.Time;
acc_data = reshape(Q.tsinterpd.Data,[length(Q.tsinterpd.Data) 1]);

% params
    h = t(2) - t(1);
    % schmitt trigger
    Uout = 0.0625;
    Uon = 0.6*Uout;
    Uoff = 0.3*Uon;
% initialization
y = zeros(1,length(t)+1);
u = zeros(1,length(t)+1);
% simulation
for N=2:(length(t)+1)
    %data loading & error calculation
    rd = acc_data(N-1);
    e = rd - ufeedback;
%     %Transfer Function 1/ first order delay
%     udot = (Km*e - u_accum)/Tm;
%     u_accum = u_accum + udot*h;
    u(N) = e;
    u_accum = e;
    % schmitt trigger
    y(N) = Schmitt_fun_embedded(u_accum,Uout,Uon,Uoff);

    % feedback loop
    ufeedback = 0;
end
t2 = [t;t(end)+h];
%
figure
plot(t2,u,'b')
hold on
plot(t,acc_data,'r')
title('Delay block out')
plot(t,t*0+Uon,'g--')
plot(t,t*0+Uoff,'r--')
plot(t,t*0-Uon,'g--')
plot(t,t*0-Uoff,'r--')
%
figure
plot(t2,y,'b')
hold on
plot(t,acc_data,'r')
title('PWPF')