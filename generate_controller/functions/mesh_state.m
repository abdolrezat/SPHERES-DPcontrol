function s_1 = mesh_state(x1,n1)
%custom meshing function
s_1 = linspace2(min(x1), max(x1), n1);

function vector = linspace2(vector_min, vector_max, n_mesh_vector)
% a custom linear spacing function that includes zero(absolute) in the vector
temp_vector = linspace(vector_min, vector_max, n_mesh_vector+1);
n_positive = sum(temp_vector >= 0 );
n_negative = sum(temp_vector < 0 );
temp_vector_positive = linspace(0, vector_max, n_positive);
temp_vector_negative = linspace(vector_min, 0, n_negative);
vector = [temp_vector_negative(1:end-1), temp_vector_positive];
