function c = stumpC(z)
% wwwwwwwwwwwwwwwwwwwwww
%{
This function evaluates the Stumpff function C(z) according
to Equation 3.53.
z - input argument
c - value of C(z)
User M-functions required: none
%}
% ----------------------------------------------
if z > 0
c = (1 - cos(sqrt(z)))/z;
elseif z < 0
c = (cosh(sqrt(-z)) - 1)/(-z);
else
c = 1/2;
end