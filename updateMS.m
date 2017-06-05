%% Update MS location, velocity, direction
%
% Frees channels by incrementing BTS free_channel property for each channel
% to be freed. Channel value is the BTS index.
%
% Inputs
%       MS - **instance** of MS to update, just 1 not the 1-N list
%       time - current simulation time
%       time_step - simulation time step
%       lasttime - last time the direction, velocity were updated
%       velocity_mean - mean of velocity model, Gaussian distribution
%       velocity_stddev - standard deviation of velocity model
%       velocity_limit - upper limit for velocity model output, m/s
%
% Outputs
%       MS - updated MS **instance** not the full structure, just 1 MS
%       lasttime - need to keep this for next call to function
%
% License
%    Copyright (C) 2017 Tim Cardenuto
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU Lesser General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public License
%    along with this program. If not, see <http://www.gnu.org/licenses/>.
%#########################################################################

function [MS, lasttime] = updateMS(MS, time, time_step, lasttime, ...
                            velocity_mean, velocity_stddev, velocity_limit)
          
    % update mobile position
    distance = time_step * MS.velocity; 
    MS.x = distance * cos(MS.direction) + MS.x;
    MS.y = distance * sin(MS.direction) + MS.y;

    if (time - lasttime >= 10)
        lasttime=time;
        % update direction every 10s
        MS.direction = (unidrnd(360)/180)*pi; % in radians

        % update velocity every 10s
        MS.velocity = normrnd(velocity_mean,velocity_stddev);
        while ( MS.velocity < 0 || MS.velocity > velocity_limit )
           MS.velocity = normrnd(velocity_mean,velocity_stddev);
        end
    end
    
end
