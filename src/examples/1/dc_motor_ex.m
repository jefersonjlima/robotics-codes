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
  J = 0.01;
  b = 0.1;
  K = 0.01;
  R = 1;
  L = 0.5;
  % state space model
  A = [-b/J   K/J
    -K/L   -R/L];
  B = [0
    1/L];
  C = [1   0];
  % input voltage
  u = 12;
  % system equation
  dx = zeros(2,1);
  dx = A*x+B*u;
end
