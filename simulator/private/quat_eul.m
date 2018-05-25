function [X,Y,Z] = quat_eul(q0)
% function [phi,theta,sai] = quat_eul(q0)
%QUAT_EUL Summary of this function goes here
%   Detailed explanation goes here
% theta = asin(   -2.*(q0(2).*q0(4) - q0(1).*q0(3))   ).*180/pi;
% phi = atan2( 2.*(q0(3).*q0(4) + q0(1).*q0(2) ), 1 - 2.*(q0(2)^2 + q0(3)^2) ).*180/pi;
% sai = atan2( 2.*(q0(2).*q0(3) + q0(1).*q0(4) ), 1 - 2.*(q0(3)^2 + q0(4)^2) ).*180/pi;
w = q0(:,1);
x = q0(:,2);
y = q0(:,3);
z = q0(:,4);

ysqr = y .* y;
t0 = +2.0 .* (w .* x + y .* z);
t1 = +1.0 - 2.0 .* (x .* x + ysqr);
X = rad2deg(atan2(t0, t1));
t2 = +2.0 .* (w .* y - z .* x);

    t2(t2 > +1.0) = +1.0 ;


    t2(t2 < -1.0)= -1.0;

Y = rad2deg(asin(t2));

t3 = +2.0 .* (w .* z + x .* y);
t4 = +1.0 - 2.0 .* (ysqr + z .* z);
Z = rad2deg(atan2(t3, t4));


end

