%POSQSTEP   Robot control function
%   POSQSTEP(T, XCURRENT, XEND, DIR, B) is the control function which
%   is called after every timer tick by POSQINTEGR.
%
%   Where:
%   t        : Current simulation time (starts at 0)
%   xcurrent : 3x1-vector of the current pose [x; y; theta]
%   xend     : 3x1-vector of the goal [x; y; theta]
%   dir      : optional direction of motion (1 forward (default),
%              -1 backward, 0 automatic)
%   b        : b of robot
%   Kv       : Kv is the gain give 
%   vl, vr   : Left/right wheel velocity
%   eot      : Boolean value. 1 if end of trajectory
%
%   See also posqintegr
% Based on the position controller by Allesandro Astolfi, ETHZ
% (c) 1998 R�my Blank EPFL-ASL / Modified 2000.10.09 Gilles Caprari EPFL-ASL
% v.1.0, Kai Arras, EPFL-ASL
% v.1.1, Jan Weingarten, Kai Arras, EPFL-ASL: wheelbase argument B added
% v.1.1.1, Kai Arras, CAS-KTH: minor modifications.
% v 2.0  Luigi Palmieri, SRL - Uni-Freiburg : implementing new version of
% the position controller, POSQ that cancels the discontinuity
% asymptotically.
% Copyright (c) 2014, Luigi Palmieri, Social Robotics Laboratory,
% University of Freiburg
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
% 
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
% 
% 2. Redistributions in binary form must reproduce the above copyright
% notice, this list of conditions and the following disclaimer in the
% documentation and/or other materials provided with the distribution.
% 
% 3. Neither the name of the copyright holder nor the names of its
% contributors may be used to endorse or promote products derived from this
% software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
% IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
% THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
% PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
% CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



function [vl, vr, eot,vm,vd] = posqstep(t, xcurrent, xend, dir, b)
persistent oldBeta

Kv      = 2;
Krho    = 1;                  % Condition: Kalpha + 5/3*Kbeta - 2/pi*Krho > 0 !
Kalpha  = 3;    
Kbeta   = -1;
Kphi    = -1;


Vmax    = Krho;                % [m/s]

RhoEndCondition = 0.005;      % [m]
PhiEndCondition = 0.01744/10;

Verbose = 0;

if t == 0									    % First call in loop
    oldBeta = 0;
end;

% extract coordinates
xc = xcurrent(1); yc = xcurrent(2); tc = xcurrent(3);
xe = xend(1); ye = xend(2); te = xend(3);

% rho
dx = xe - xc;
dy = ye - yc;
rho = sqrt(dx^2 + dy^2);
fRho = rho;
if fRho > (Vmax/Krho),
    fRho = Vmax/Krho;
end;

% alpha
alpha = atan2(dy, dx) - tc;
alpha = normangle(alpha, -pi);


% direction
if dir == 0							  % controller choose the forward direction
    if alpha > pi/2
        fRho = -fRho;					% backwards
        alpha = alpha-pi;
    elseif alpha <= -pi/2
        fRho = -fRho;					% backwards
        alpha = alpha+pi;
    end;
elseif dir == -1					% arrive backwards
    fRho = -fRho;
    alpha = alpha+pi;
    if alpha > pi
        alpha = alpha - 2*pi;
    end;
else
    % arrive forward
end;

% phi
phi = te-tc;
phi = normangle(phi, -pi);

beta = normangle(phi-alpha, -pi);
if abs(oldBeta-beta) > pi			% avoid instability
    beta = oldBeta;
end;
oldBeta = beta;


% New version
vm= Krho*tanh(fRho*Kv)
vd=(Kalpha*alpha + Kbeta*beta);
tanh(fRho)

eot = (rho < RhoEndCondition) & (abs(phi) < PhiEndCondition);      

if eot,
    if Verbose,
        disp(sprintf('t: %G sec  x: %G  y: %G  theta: %G',t, xc, yc, tc*180/pi));
    end;
end;

% Convert speed to wheel speeds
vl = vm - vd*b/2;
if abs(vl) > Vmax
    vl = Vmax*sign(vl);
end;

vr = vm + vd*b/2;
if abs(vr) > Vmax
    vr = Vmax*sign(vr);
end;



end


