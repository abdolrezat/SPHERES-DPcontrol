function [s_x,s_v,s_t,s_w] = mesh_states_4var(x,v,t,w, nx,nv,nt,nw)
%custom meshing function for 4 states
%pos
this.v_min = min(v);
this.v_max = max(v);
this.n_mesh_v = nv;

this.x_min = min(x);
this.x_max =  max(x);
this.n_mesh_x = nx;
%att
this.w_min = min(w);
this.w_max = max(w);
this.n_mesh_w = nw;

this.theta_min = min(t); %for channel x, this is angles of rotation about y-axis (pitch)
this.theta_max = max(t);
this.n_mesh_t = nt;
%
s_x = linspace2(this.x_min, this.x_max, this.n_mesh_x);
s_v = linspace2(this.v_min, this.v_max, this.n_mesh_v);
s_t = linspace2(this.theta_min, this.theta_max, this.n_mesh_t);
s_w = linspace2(this.w_min, this.w_max, this.n_mesh_w);


function vector = linspace2(vector_min, vector_max, n_mesh_vector)
%a custom linear spacing function that includes zero(absolute) in the vector
temp_vector = linspace(vector_min, vector_max, n_mesh_vector+1);
n_positive = sum(temp_vector >= 0 );
n_negative = sum(temp_vector < 0 );
temp_vector_positive = linspace(0, vector_max, n_positive);
temp_vector_negative = linspace(vector_min, 0, n_negative);
vector = [temp_vector_negative(1:end-1), temp_vector_positive];
