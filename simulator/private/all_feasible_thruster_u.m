function [f1_a,f2_a,f3_a,f4_a] = all_feasible_thruster_u(id_faulty)
[f0,f1,f6,f7] = set_thruster_u(id_faulty);
[f1_a,f2_a,f3_a,f4_a] = vectors_allcomb(f0,f1,f6,f7);
end

function  [f1_a,f2_a,f3_a,f4_a] = vectors_allcomb(f1,f2,f3,f4)
[f1,f2,f3,f4] = ndgrid(f1,f2,f3,f4);
f1 = f1(:);
f2 = f2(:);
f3 = f3(:);
f4 = f4(:);

idrm1 = find( f1 > 0 & f3 > 0 );
idrm2 = find( f2 > 0 & f4 > 0 );

idrm = unique([idrm1,idrm2])';
id = setdiff(1:length(f1),idrm);

f1_a = f1(id);
f2_a = f2(id);
f3_a = f3(id);
f4_a = f4(id);
end

function [f0,f1,f6,f7] = set_thruster_u(id_faulty)

if ~ismember(0, id_faulty)
    f0 = [0 1];
else
    f0 = 0;
end

if ~ismember(1, id_faulty)
    f1 = [0 1];
else
    f1 = 0;
end

if ~ismember(6, id_faulty)
    f6 = [0 1];
else
    f6 = 0;
end

if ~ismember(7, id_faulty)
    f7 = [0 1];
else
    f7 = 0;
end

end