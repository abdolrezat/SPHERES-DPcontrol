function monte_carlo_thr9_fault()
%MONTE_CARLO_THR9_FAULT Summary of this function goes here
%   Detailed explanation goes here

% H = zeros(1,10);

if ~exist('montcarloData.mat')
    generate_montecarlo_data()
end

analyze_montecarlo


function generate_montecarlo_data()

for d = 1:10
    r = rand(1,3)*10 + 5;
    idrand = rand(1,3) > 0.5;
    r(idrand) = -r(idrand);
    
    v = rand(1,3)*0.05;
    idrand = rand(1,3) > 0.5;
    v(idrand) = -v(idrand);
    
    
S = fault_thr9_Dynamic_Programming(r,v);
H(d) = S;
disp(d)
end

save montcarloData H


function analyze_montecarlo
Q = load('montcarloData.mat');

X2 = zeros(1,numel(Q.H));
V2 = X2;
T3 = X2;
W3 = X2;
T_ode = Q.H(1).history.T_ode45;

for i = 1:numel(Q.H)
    obj = Q.H(i);
    X2_step = stepinfo(obj.history.X_ode45(:,2),obj.history.T_ode45,0);
    Ts = X2_step.SettlingTime;
%     Tselect = find(abs(T_ode > (Ts + 30)));
Tselect = length(T_ode);
    if isempty(Tselect)
        Tselect = length(T_ode);
    else
        Tselect = Tselect(1);
    end
    X2(i) = obj.history.X_ode45(Tselect,2);
    V2(i) = obj.history.X_ode45(Tselect,5);
    eul = quat2eul(obj.history.X_ode45(Tselect,10:-1:7));
    T3(i) = eul(3);
    W3(i) = obj.history.X_ode45(Tselect,13);
end

run_no = 1:length(X2);

X2 = cumsum(abs(X2))./run_no;
V2 = cumsum(abs(V2))./run_no;
T3 = cumsum(abs(T3))./run_no;
W3 = cumsum(abs(W3))./run_no;


figure('color','white')
subplot(4,1,1)
plot(run_no, abs(X2),'ko-')
set(gca, 'TickDir','out', 'Box', 'off')

subplot(4,1,2)
plot(run_no, abs(V2),'ko-')
set(gca, 'TickDir','out', 'Box', 'off')

subplot(4,1,3)
plot(run_no, abs(T3),'ko-')
set(gca, 'TickDir','out', 'Box', 'off')

subplot(4,1,4)
plot(run_no, abs(W3),'ko-')
set(gca, 'TickDir','out', 'Box', 'off')
xlabel('Run number')