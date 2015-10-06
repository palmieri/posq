%POSQINTEGR   POSQ controller for differential drive robots
%   [XVEC, VVEC] = POSQINTEGR(XSTART, XEND, DIR, DT, B) generates
%   and returns the trajectory which brings a differential drive robot
%   with wheelbase B from the start pose XSTART to the goal pose XEND.
%   Calls the control function POSQSTEP every DT seconds.
%
%   Where:
%   xstart : 3x1 initial robot pose vector [x; y; theta]
%   xend   : 3x1 goal pose vector [x; y; theta]
%   dir    : direction of motion (1 forward, -1 backward, 0 automatic)
%   dt     : time in [s]. Control frequency is 1/dt
%   b      : wheelbase of robot
%   xvec   : robot trajectory. 3xn matrix of n 3x1-poses
%   vvec   : 2xn matrix of wheel speeds: left (1st row), right (2nd row)
%
%   See also posqstep
% Based on the position controller by Allesandro Astolfi, ETHZ
% Matlab version: 1998 Remy Blank EPFL-ASL
% Modified 2000.10.09 Gilles Caprari EPFL-ASL
% v.1.0, Kai Arras, EPFL-ASL
% v.1.1, Jan Weingarten, Kai Arras, EPFL-ASL : b argument added 
% v.1.2, Kai Arras, CAS-KTH: suboptimal odometry expressions improved (dSd/2)
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


function [xvec, speedvec, vel, inct] = posqintegr(xstart, xend, dir, deltaT, b,initT)

if (size(xstart,1)*size(xstart,2) == 3) && (size(xend,1)*size(xend,2) == 3),
    if size(xstart) == [1,3], xstart = xstart'; end;
    if size(xend)   == [1,3], xend   = xend';   end;
    vel=[];
    % Initialize variables
    sl = 0;
    sr = 0;
    oldSl = 0;
    oldSr = 0;
    xvec = [];				         % pose vectors for trajectory
    speedvec = [0; 0];         % velocities during trajectory
    encoders = [0; 0];
    t = initT;                     % initialize global timer
    ti=0;                          % initialize local timer
    eot = 0;                   % initialize end-of-trajectory flag
    xcurrent = xstart;         % current pose equals start pose
    
    visual=0; % If you want visualize the robot step by step
    inct=[]
    
    while ~eot,
        
        % Calculate distances for both wheels
        dSl = sl-oldSl;
        dSr = sr-oldSr;
        dSm = (dSl+dSr)/2;
        dSd = (dSr-dSl)/b;
        
        % Integrate robot position
        xcurrent(1) = xcurrent(1) + dSm*cos(xcurrent(3)+dSd/2);
        xcurrent(2) = xcurrent(2) + dSm*sin(xcurrent(3)+dSd/2);
        xcurrent(3) = normangle(xcurrent(3) + dSd, -pi);

        
        % implementation of the controller
        [vl, vr, eot,vm,vd] = posqstep(ti, xcurrent, xend, dir, b);
        
        
        
        vel=[vel [vm;vd]];
        speeds = [vl; vr];
        % Add current values to trajectory
        speedvec = [speedvec, speeds];
        xvec     = [xvec, xcurrent];
        
        % Increase timers
        ti=ti+deltaT;
        t = t + deltaT;
        
        % Increase accumulated encoder values
        encoders = encoders + speeds*deltaT;			% simulated encoders of robot
        
        % Keep track of previous wheel positions
        oldSl = sl;
        oldSr = sr;
        % noise on the encoders
        nS=0;
        sl = encoders(1,1)+nS*rand;
        sr = encoders(2,1)+nS*rand;
        
        
       inct=[t;inct];
    end;
    
%     inct=t; % at the end of the trajectory the time elapsed is added
    
else
    disp('Wrong input. Check your arguments');
end;