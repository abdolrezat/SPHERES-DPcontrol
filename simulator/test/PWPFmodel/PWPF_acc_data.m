Q = load('sigTSinterpd.mat');
t = Q.tsinterpd.Time;
acc_data = reshape(Q.tsinterpd.Data,[length(Q.tsinterpd.Data) 1]);
acc_data = acc_data * 4.16; %Thruster Force (N)
% params
    h = t(2) - t(1);
    % delay
    Km = 1.5;
    Tm = .2;
    H = .6; %Feedback TF
    % schmitt trigger
    Uout = 0.13;
    Uon = 0.8*Uout;
    Uoff = 0.8*Uon;
% initialization
ufeedback = 0;
u_accum = 0;
y = zeros(1,length(t)+1);
y2 = zeros(1,length(t)+1);
u = zeros(1,length(t)+1);

%test class PWPF
PWPF = PWPF_c(Km,Tm,h,Uout,Uon,Uoff,H);
% simulation
for N=2:(length(t)+1)
    %data loading & error calculation
    rd = acc_data(N-1);
    e = rd - H*ufeedback;
    %Transfer Function 1/ first order delay
    udot = (Km*e - u_accum)/Tm;
    u_accum = u_accum + udot*h;
    u(N) = u_accum;
    % schmitt trigger
    y(N) = Schmitt_fun_embedded(u_accum,Uout,Uon,Uoff);
    y2(N) = PWPF.signal_update(rd);
    % feedback loop
    ufeedback = y(N);
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

% %verification
% figure
% plot(t2,y,'b')
% hold on
% plot(t2,y2,'r.')
% title('PWPF script vs. class')