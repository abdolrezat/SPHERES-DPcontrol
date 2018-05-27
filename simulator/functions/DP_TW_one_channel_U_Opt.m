function [M_gI,U_Optimal_id] = DP_TW_one_channel_U_Opt(s_t,s_w, ...
    v_Mthruster,Qt,Qw, R, h, T_final, J)

%calculate and correct number of stages if needed
N_stage = T_final/h;

if(rem(T_final,h) == 0)
    N_stage = T_final/h;
else
    N_stage = T_final/h;
    N_stage = ceil(N_stage);
    T_final = h*N_stage;
    warning('T_final is not a factor of h (dt), increasing T_final to %.3f\n',T_final)
end

n_t = length(s_t);
n_w = length(s_w);

% initialization
%calculating next stage states, J input to the next states must be in accordance with the Moment direction
[t_next,w_next] = next_stage_states_simplified( s_t,s_w,...
    v_Mthruster, h , J);

%calculating J fixed
sin_s_t = sin(s_t/2);
J_current = J_current_reshaped( sin_s_t,s_w,...
    v_Mthruster,...
    Qt,Qw,R);
M_gI = griddedInterpolant({s_t,s_w},...
    zeros(n_t,n_w,'single'),'linear','linear');

%memory
%print memory
mem_vars = whos('M_gI','J_current','t_next','w_next');
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

% fsum50_prev = 0;
idsum50_prev = 0;
tol = -1;
tic
for k_s = N_stage-1:-1:1
    %% move U_optimal_id to the terminal checkpoint
    [M_gI.Values, U_Optimal_id] = min( J_current + M_gI(t_next,w_next), [], 3);
    if ~rem(k_s,50)
        if(excess_memory)
%             fsum50 = M_gI.Values - fsum50_prev;
            idsum50 = U_Optimal_id - idsum50_prev;
%             e = sum(fsum50(:));
            e2 = sum(idsum50(:));
            fprintf('stage %d - %f seconds - errorU %g\n', k_s, toc, e2)
%             fprintf('stage %d - %f seconds - errorF %g - errorU %g\n', k_s, toc, e, e2)
%             fsum50_prev = M_gI.Values;
            idsum50_prev = U_Optimal_id;
            
            if(abs(e2) < tol )
                %define a counter
                if(exist('stop_counter') == 0)
                    stop_counter = 0;
                end
                stop_counter = stop_counter + 1;
                
                %end the loop if abs(e2) < tol happens more than 5 times
                if(stop_counter > 5)
                fprintf('sum of errors in the last 50 stages is under tolerance, breaking loop...\n')
                break
                end
            end
        else
            fprintf('stage %d - %f seconds\n', k_s, toc)
        end
        
        tic
    end
end
%finish up
fprintf('\nstage calculations complete.\n')
M_gI = M_gI.GridVectors;

end



function J_current_M = J_current_reshaped(t,w,M,Qt,Qw,R)

L_t = length(t);
L_w = length(w);

t = single(reshape(t,[L_t 1]));
w = reshape(w,[1 L_w]  );
M = reshape(M,[1 1 length(M)]); %Moment (F*T_dist)

%
J_current_M = (Qw * w.^2 + Qt * t.^2 +...
     R* M.^2 );
end



function [t_next,w_next] = next_stage_states_simplified(T, W, M ,h , J)
%store length
L_t = length(T);
L_w = length(W);

% reshape
T = reshape(T,[L_t 1]);
W = single(reshape(W,[1 L_w]));
M = reshape(M,[1 1 length(M)]); %Moment (F*T_dist)
% ODE solve
t_next = RK4_t( T, W, h);
w_next = RK4_w( W, M, h, J);


% repmat each matrix to full size, as required for F inputs
t_next = repmat(t_next,[1 1 length(M)]);
w_next = repmat(w_next,[L_t 1 1]);
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

function W2 = RK4_w( w, M, h, J)
%calculates next stage (k+1) states
% X2 = X1 + dt*a_d(X,u) where a_d is the spacecraft dynamics
%first order taylor expansion
%X2 = X1 + dt*spacecraft_dynamics(spacecraft, X1, U);

% Runge-Kutta - 4th order
% h = dt;
k1 = wdynamics(w , M, J);
k2 = wdynamics((w + k1*h/2), M, J);
k3 = wdynamics((w + k2*h/2), M, J);
k4 = wdynamics((w + k3*h), M, J);

W2 = w + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end


function w_dot = wdynamics( ~, M, J)
w_dot = M/J;
end
