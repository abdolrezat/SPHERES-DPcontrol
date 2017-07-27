function [R2,V2] = update_RV_target(R0,V0,t)
%Updates the state vectors of the target sat
% mu - gravitational parameter (km^3/s^2)
% R0 - initial position vector (km)
% V0 - initial velocity vector (km/s)
% t - elapsed time (s)
% R - final position vector (km)
% V - final velocity vector (km/s)
mu = 398600;
%...Magnitudes of R0 and V0:
r0 = norm(R0);
v0 = norm(V0);
%...Initial radial velocity:
vr0 = dot(R0, V0)/r0;
%...Reciprocal of the semimajor axis (from the energy equation):
alpha = 2/r0 - v0^2/mu;
%...Compute the universal anomaly:
x = kepler_U(t, r0, vr0, alpha);
%...Compute the f and g functions:
[f, g] = f_and_g(x, t, r0, alpha);
%...Compute the final position vector:
R2 = f*R0 + g*V0;
%...Compute the magnitude of R:
r2 = norm(R2);
%...Compute the derivatives of f and g:
[fdot, gdot] = fDot_and_gDot(x, r2, r0, alpha);
%...Compute the final velocity:
V2 = fdot*R0 + gdot*V0;
end

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  function x = kepler_U(dt, ro, vro, a)
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%{
  This function uses Newton's method to solve the universal
  Kepler equation for the universal anomaly.

  mu   - gravitational parameter (km^3/s^2)
  x    - the universal anomaly (km^0.5)
  dt   - time since x = 0 (s)
  ro   - radial position (km) when x = 0
  vro  - radial velocity (km/s) when x = 0
  a    - reciprocal of the semimajor axis (1/km)
  z    - auxiliary variable (z = a*x^2)
  C    - value of Stumpff function C(z)
  S    - value of Stumpff function S(z)
  n    - number of iterations for convergence
  nMax - maximum allowable number of iterations
 
  User M-functions required: stumpC, stumpS
%}
% ----------------------------------------------
mu = 398600;

%...Set an error tolerance and a limit on the number of iterations:
error = 1.e-8;
nMax  = 1000;

%...Starting value for x:
x = sqrt(mu)*abs(a)*dt;

%...Iterate on Equation 3.65 until until convergence occurs within
%...the error tolerance:
n     = 0;
ratio = 1;
while abs(ratio) > error && n <= nMax
    n     = n + 1;
    C     = stumpC(a*x^2);
    S     = stumpS(a*x^2);
    F     = ro*vro/sqrt(mu)*x^2*C + (1 - a*ro)*x^3*S + ro*x - sqrt(mu)*dt;
    dFdx  = ro*vro/sqrt(mu)*x*(1 - a*x^2*S) + (1 - a*ro)*x^2*C + ro;
	ratio = F/dFdx;
    x     = x - ratio;
end

%...Deliver a value for x, but report that nMax was reached:
if n > nMax
    fprintf('\n **No. iterations of Kepler''s equation = %g', n)
    fprintf('\n   F/dFdx                              = %g\n', F/dFdx)
end
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  end