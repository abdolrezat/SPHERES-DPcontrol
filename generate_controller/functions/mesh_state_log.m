function s_exp = mesh_state_log(x1,n1,a)
%custom meshing function
s_exp = logspace2(min(x1), max(x1), n1, a);
% figure
% plot(s_exp)
% title('mesh of one state')
% xlabel('index of element')
% ylabel('value of element')

function vector = logspace2(vector_min, vector_max, n_mesh_vector, a)
% a custom linear spacing function that includes zero(absolute) in the vector
temp_vector = linspace(vector_min, vector_max, n_mesh_vector+1);
% for positive values:
n_positive = sum(temp_vector >= 0 );
temp_vector_positive = linspace(0, a, n_positive);
temp = exp(temp_vector_positive);
%re-sale to 0 - 1
temp_01 = temp/max(temp) - min(temp)/max(temp);
% re-scale back to original limits
temp_vector_positive = temp_01*(vector_max);

% for negative values
n_negative = sum(temp_vector < 0 );
temp_vector_negative = linspace(0, a, n_negative);
temp = exp(temp_vector_negative);
%re-sale to 0 - 1
temp_01 = temp/max(temp) - min(temp)/max(temp);
% re-scale back to original limits
temp_vector_negative = flip(temp_01*(vector_min));


%cat
vector = [temp_vector_negative(1:end-1), temp_vector_positive];
