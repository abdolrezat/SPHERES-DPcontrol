function [q] = mrp2quat(s)
%MRP2QUAT Summary of this function goes here
%   Detailed explanation goes here
q = zeros(4,size(s,2));
f = 1;
f2 = 1;
pm2     = s(1,:).*s(1,:) + s(2,:).*s(2,:) + s(3,:).*s(3,:);
    c0      = 1  ./ (f2 + pm2);
%     q = zeros(4, length(s));
   
    q(4,:)  = c0 .* (f2 - pm2);
    c0      = (2*f) * c0;
    q(1,:)  = c0 .* s(1,:);
    q(2,:)  = c0 .* s(2,:);
    q(3,:)  = c0 .* s(3,:);
end

