% ----------------------------------------------------------------------------
% Copyright 2022, Jeferson Lima
% All Rights Reserved
% See LICENSE for the license information
% -------------------------------------------------------------------------- */

% @file   examples/1/dc_motor_ex.m
% @author Jeferson Lima
% @brief  DC Motor Simulation Example 
% @date   Mar 25, 2022

function [T,X] = dc_motor
  Tmax = 6;                 
  Tsim = 1e-3;              
  time = 0:Tsim:Tmax;
  x0 = [0; 0];
  options = odeset('MaxStep',1e-4);
  [T,X]=ode45(@dc_motor_model,time,x0,options);
  save('out.mat','X','T');
end

function dx = dc_motor_model(t,x)  
  % parameters
  J = 0.01; 	% (J)     moment of inertia of the rotor     0.01 kg.m^2
  b = 0.1;	% (b)     motor viscous friction constant    0.1 N.m.s 
  K = 0.01; 	% (Kt)    motor torque constant              0.01 N.m/Amp
  		% (Ke)    electromotive force constant       0.01 V/rad/sec
  R = 1;	% (R)     electric resistance                1 Ohm
  L = 0.5;	% (L)     electric inductance                0.5 H
  V = 12;   	% (V)     motor votage                       12 V
  % state space model
  A = [-b/J   K/J
    -K/L   -R/L];
  B = [0
    1/L];
  C = [1   0];
  % input voltage
  u = V;
  % system equation
  dx = zeros(2,1);
  dx = A*x+B*u;
end
