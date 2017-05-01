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