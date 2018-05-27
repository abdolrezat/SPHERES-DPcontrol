function DP_4var_one_channel_U_Opt(s_x,s_v,s_t,s_w, ...
    f0, f1, f6, f7,Qx,Qv,Qt,Qw, R, h, T_final,Mass, J, T_dist,file_name)

%calculate and correct number of stages if needed
N_stage = T_final/h;

if(~isinteger( N_stage))
    N_stage = ceil(N_stage);
    T_final = h*N_stage;
    warning('T_final is not a factor of h (dt), increasing T_final to %.2f\n', T_final)
end

n_x = length(s_x);
n_v = length(s_v);
n_t = length(s_t);
n_w = length(s_w);

%% initialization
%get all combinations of thruster firings
[f0_allcomb,f1_allcomb,f6_allcomb,f7_allcomb] = ...
    vectors_allcomb( f0, f1, f6, f7);
%calculating next stage states, J input to the next states must be in accordance with the Moment direction
[x_next,v_next,t_next,w_next] = next_stage_states_simplified( s_x,s_v,s_t,s_w,...
    f0_allcomb, f1_allcomb, f6_allcomb, f7_allcomb, h ,Mass, J, T_dist);

%calculating J fixed
J_current = J_current_reshaped( s_x,s_v,s_t,s_w,...
    f0_allcomb,  f1_allcomb,  f6_allcomb,  f7_allcomb,...
    Qx,Qv,Qt,Qw,R);
F_gI = griddedInterpolant({s_x,s_v,s_t,s_w},...
    zeros(n_x,n_v,n_t,n_w,'single'),'linear','nearest');

%memory
%print memory
mem_vars = whos('F_gI','J_current','x_next','v_next','t_next','w_next');
sum_mem = 0;
for mem_i = 1:length(mem_vars)
    fprintf('variable name: %s - size: %4.f Mb\n',mem_vars(mem_i).name, mem_vars(mem_i).bytes/1e6)
    sum_mem = sum_mem + mem_vars(mem_i).bytes/1e9;
end
fprintf('total memory used by vars: %2.2f GigaBytes\n',sum_mem)

%excess memory
memory_safety_factor = 3;
[ MeMuser, MeMsystem] = memory;

fprintf('total memory used by MATLAB: %2.2f GigaBytes\n', MeMuser.MemUsedMATLAB/1e9)
fprintf('total available memory: %2.2f GigaBytes\n', MeMsystem.PhysicalMemory.Available/1e9)
if(MeMsystem.PhysicalMemory.Available > numel(J_current)*4 * memory_safety_factor)
    %if there is shortage of memory, do not spend more of it to
    %record error between stages
    excess_memory = 1;
else
    warning('there is no more memory available for keeping record of errors between stages...\n')
    excess_memory = 0;
end

fsum50_prev = 0;
idsum50_prev = 0;
tol = 1;
tic
for k_s = N_stage-1:-1:1
    %% move U_optimal_id to the terminal checkpoint
    [F_gI.Values, U_Optimal_id] = min( J_current + F_gI(x_next,v_next,t_next,w_next), [], 5);
    if ~rem(k_s,50)
        if(excess_memory)
            fsum50 = F_gI.Values - fsum50_prev;
            idsum50 = U_Optimal_id - idsum50_prev;
            e = sum(fsum50(:));
            e2 = sum(idsum50(:));
            fprintf('stage %d - %f seconds - errorF %g - errorU %g\n', k_s, toc, e, e2)
            fsum50_prev = F_gI.Values;
            idsum50_prev = U_Optimal_id;
            
            if(abs(e2) < tol)
                fprintf('sum of errors in the last 50 stages is under tolerance, breaking loop...\n')
                break
            end
        else
            fprintf('stage %d - %f seconds\n', k_s, toc)
        end
        
        tic
    end
end

clear('x_next','v_next','t_next','w_next','J_current')
save(file_name,'F_gI','U_Optimal_id','f0_allcomb','f1_allcomb','f6_allcomb','f7_allcomb','fsum50','idsum50')
%finish up
fprintf('\nstage calculations complete.\n')

end

function J_current_M = J_current_reshaped(x,v,t,w,f1,f2,f3,f4,Qx,Qv,Qt,Qw,R)

L_x = length(x);
L_v = length(v);
L_t = length(t);
L_w = length(w);

x = single(reshape(x,[L_x 1]  ));
v = reshape(v,[1 L_v]  );
t = reshape(t,[1 1 L_t]  );
w = reshape(w,[1 1 1 L_w]  );
f1 = reshape(f1,[1 1 1 1 length(f1)]  );
f2 = reshape(f2,[1 1 1 1 length(f2)]  );
f3 = reshape(f3,[1 1 1 1 length(f3)]  );
f4 = reshape(f4,[1 1 1 1 length(f4)]  );
%
J_current_M = (Qx * x.^2 + Qv * v.^2 + Qw * w.^2 + Qt * t.^2 +...
    ( R* f1.^2 +  R* f2.^2 +  R* f3.^2 +  R* f4.^2)  );
end

function  [f1_a,f2_a,f3_a,f4_a] = vectors_allcomb(f1,f2,f3,f4)
[f1,f2,f3,f4] = ndgrid(f1,f2,f3,f4);
f1 = f1(:);
f2 = f2(:);
f3 = f3(:);
f4 = f4(:);

idrm1 = find( f1 >0 & f3< 0 );
idrm2 = find( f2 > 0 & f4 < 0);

idrm = unique([idrm1,idrm2])';
id = setdiff(1:length(f1),idrm);

f1_a = f1(id);
f2_a = f2(id);
f3_a = f3(id);
f4_a = f4(id);

end

function [x_next,v_next,t_next,w_next] = next_stage_states_simplified(X, V, T, W, f1,f2,f6,f7 ,h ,Mass, J, T_dist)
%store length
L_x = length(X);
L_v = length(V);
L_t = length(T);
L_w = length(W);

% reshape
X = reshape(X,[L_x 1]  );
V = reshape(V,[1 L_v]  );
T = reshape(T,[1 1 L_t]  );
W = reshape(W,[1 1 1 L_w]  );
f1 = reshape(f1,[1 1 1 1 length(f1)]  );
f2 = reshape(f2,[1 1 1 1 length(f2)]  );
f6 = reshape(f6,[1 1 1 1 length(f6)]  );
f7 = reshape(f7,[1 1 1 1 length(f7)]  );
% ODE solve
x_next = single( RK4_x( X, V, h)) ;
v_next = single(RK4_v( V, f1,f2,f6,f7, h, Mass));
t_next = single(RK4_t( T, W, h));
w_next = single(RK4_w( W, f1,f2,f6,f7, h, J, T_dist));


% repmat each matrix to full size, as required for F inputs
x_next = repmat(x_next,[1 1 L_t L_w length(f1)]);
v_next = repmat(v_next,[L_x 1 L_t L_w 1]);
t_next = repmat(t_next,[L_x L_v 1 1 length(f1)]);
w_next = repmat(w_next,[L_x L_v L_t 1 1]);
end

function X2 = RK4_x( X1, V, h)
% Runge-Kutta - 4th order
% h = dt;
k1 = xdynamics( V);
k2 = xdynamics((V + k1*h/2));
k3 = xdynamics((V + k2*h/2));
k4 = xdynamics((V + k3*h));

X2 = X1 + h*(k1 + 2*k2 + 2*k3 + k4)/6;

end

function x_dot = xdynamics(v)
x_dot = v;
end

function V2 = RK4_v( V1, f1,f2,f6,f7, h, Mass) % does not need RK4, ki's are equal
% Runge-Kutta - 4th order
% h = dt;
k1 = vdynamics( V1 , f1,f2,f6,f7, Mass);
k2 = vdynamics((V1 + k1*h/2), f1,f2,f6,f7, Mass);
k3 = vdynamics((V1 + k2*h/2), f1,f2,f6,f7, Mass);
k4 = vdynamics((V1 + k3*h), f1,f2,f6,f7, Mass);

V2 = V1 + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end

function v_dot = vdynamics( ~, f1,f2,f6,f7, Mass)
v_dot = (f1+f2+f6+f7)/Mass;
end

function T2 = RK4_t( T1, W1, h)
%calculates next stage (k+1) states
% X2 = X1 + dt*a_d(X,u) where a_d is the spacecraft dynamics
%first order taylor expansion
%X2 = X1 + dt*spacecraft_dynamics(spacecraft, X1, U);

% Runge-Kutta - 4th order
% h = dt;
k1 = tdynamics(W1);
k2 = tdynamics((W1 + k1*h/2));
k3 = tdynamics((W1 + k2*h/2));
k4 = tdynamics((W1 + k3*h));

T2 = T1 + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end


function t_dot = tdynamics(w)
t_dot = w;
end

function W2 = RK4_w( w, f1,f2,f6,f7, h, J, T_dist)
%calculates next stage (k+1) states
% X2 = X1 + dt*a_d(X,u) where a_d is the spacecraft dynamics
%first order taylor expansion
%X2 = X1 + dt*spacecraft_dynamics(spacecraft, X1, U);

% Runge-Kutta - 4th order
% h = dt;
k1 = wdynamics(w , f1,f2,f6,f7, J, T_dist);
k2 = wdynamics((w + k1*h/2), f1,f2,f6,f7, J, T_dist);
k3 = wdynamics((w + k2*h/2), f1,f2,f6,f7, J, T_dist);
k4 = wdynamics((w + k3*h), f1,f2,f6,f7, J, T_dist);

W2 = w + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end


function w_dot = wdynamics( ~, f1,f2,f6,f7, J, T_dist)
w_dot = (f1*T_dist +f2*(-T_dist) + ...
    f6*T_dist +f7*(-T_dist) )/J;
end
