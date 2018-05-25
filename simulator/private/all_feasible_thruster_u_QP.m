function [f_comb] = all_feasible_thruster_u_QP(id_faulty,v)
[f0,f1,f6,f7] = set_thruster_u(id_faulty,v);
[f_comb] = vectors_allcomb(f0,f1,f6,f7);
end

function  [f_comb] = vectors_allcomb(f1,f2,f3,f4)
[f1,f2,f3,f4] = ndgrid(f1,f2,f3,f4);
f1 = f1(:);
f2 = f2(:);
f3 = f3(:);
f4 = f4(:);

idrm1 = find( f1 > 0 & f3 > 0 );
idrm2 = find( f2 > 0 & f4 > 0 );

idrm = unique([idrm1,idrm2])';
id = setdiff(1:length(f1),idrm);

f_comb = [f1(id),...
    f2(id),...
    f3(id),...
    f4(id)]';
end

function [f0,f1,f6,f7] = set_thruster_u(id_faulty,v)

if ~ismember(0, id_faulty)
    f0 = v;
else
    f0 = 0;
end

if ~ismember(1, id_faulty)
    f1 = v;
else
    f1 = 0;
end

if ~ismember(6, id_faulty)
    f6 = v;
else
    f6 = 0;
end

if ~ismember(7, id_faulty)
    f7 = v;
else
    f7 = 0;
end

end