function [F_gI, U_Optimal_id] = DP_XV_one_channel_U_Opt(s_x,s_v, ...
    v_Fthruster,Qx,Qv,R, h, T_final, Mass)

%calculate and correct number of stages if needed
N_stage = T_final/h;

if(~isinteger( N_stage))
    N_stage = ceil(N_stage);
    T_final = h*N_stage;
    warning('T_final is not a factor of h (dt), increasing T_final to %.2f\n', T_final)
end

n_x = length(s_x);
n_v = length(s_v);

% initialization
%calculating next stage states, J input to the next states must be in accordance with the Moment direction
[x_next,v_next] = next_stage_states_simplified( s_x,s_v,...
    v_Fthruster, h ,Mass);

%calculating J fixed
J_current = J_current_reshaped( s_x,s_v,...
    v_Fthruster,...
    Qx,Qv,R);

F_gI = griddedInterpolant({s_x,s_v},...
    zeros(n_x,n_v,'single'),'linear','nearest');

%memory
%print memory
mem_vars = whos('F_gI','J_current','x_next','v_next');
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
tol = 20;
tic
for k_s = N_stage-1:-1:1
    %% move U_optimal_id to the terminal checkpoint
    [F_gI.Values, U_Optimal_id] = min( J_current + F_gI(x_next,v_next), [], 3);
    if ~rem(k_s,50)
        if(excess_memory)
%             fsum50 = F_gI.Values - fsum50_prev;
            idsum50 = U_Optimal_id - idsum50_prev;
%             e = sum(fsum50(:));
            e2 = sum(idsum50(:));
            fprintf('stage %d - %f seconds - errorU %g\n', k_s, toc, e2)
%             fprintf('stage %d - %f seconds - errorF %g - errorU %g\n', k_s, toc, e, e2)
%             fsum50_prev = F_gI.Values;
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

F_gI = F_gI.GridVectors; %clear memory

end %end of function: DP_XV_one_channel_U_Opt


%%% auxiliary functions
function J_current_M = J_current_reshaped(x,v,f,Qx,Qv,R)

L_x = length(x);
L_v = length(v);

x = single(reshape(x,[L_x 1]  ));
v = reshape(v,[1 L_v]  );
f = reshape(f,[1 1 length(f)]);
%
J_current_M = (Qx * x.^2 + Qv * v.^2 +...
    ( R* f.^2)  );
end

function [x_next,v_next] = next_stage_states_simplified(X, V, f ,h ,Mass)
%store length
L_x = length(X);
L_v = length(V);

% reshape
X = reshape(X,[L_x 1]  );
V = single(reshape(V,[1 L_v]  ));
f = reshape(f, [1 1 length(f)]);
% ODE solve
x_next =  RK4_x( X, V, h) ;
v_next =  RK4_v( V, f, h, Mass);
% repmat each matrix to full size, as required for F inputs
x_next = repmat(x_next,[1 1 length(f)]);
v_next = repmat(v_next,[L_x 1 1]);
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

function V2 = RK4_v( V1, f, h, Mass) % does not need RK4, ki's are equal
% Runge-Kutta - 4th order
% h = dt;
k1 = vdynamics( V1 , f, Mass);
k2 = vdynamics((V1 + k1*h/2), f, Mass);
k3 = vdynamics((V1 + k2*h/2), f, Mass);
k4 = vdynamics((V1 + k3*h), f, Mass);

V2 = V1 + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end

function v_dot = vdynamics( ~, f, Mass)
v_dot = (f)/Mass;
end

