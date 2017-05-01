%% MS generation and initialization
%
% Inputs
%       num_calls - total number of calls in simulation
%       call_arrival_mean - mean of Poisson process for call arrival
%       call_duration_mean - mean of exponential process for call duration
%       BTS - structure array for all BTS
%       BTS_num - number of BTS to be used in simulation
%       BTS_radius - effective radius of each BTS
%       BTS_channel_num - number of free channels of BTS
%       velocity_mean - mean of velocity model, Gaussian distribution
%       velocity_stddev - standard deviation of velocity model
%       velocity_limit - upper limit for velocity model output, m/s
%
% Outputs
%       MS - structure array of each MS (i.e. call) with the following properties
%           x
%           y
%           direction
%           velocity
%           call_start_time
%           call_end_time
%
%#########################################################################

function MS = generateMS(num_calls, call_arrival_mean, call_duration_mean, ...
                        BTS, BTS_num, BTS_radius, BTS_channel_num, ...
                        velocity_mean, velocity_stddev, velocity_limit)

    % Create MS structure
    MS = struct('x',0,'y',0,'direction',0,'velocity',0,'call_start_time',0, ...
        'call_end_time',0,'RSS',0,'BTS',0,'active',zeros(1,3),'activetimer', ...
        zeros(1,3),'candidate',zeros(1,6),'candidatetimer',zeros(1,6),'neighbor', ...
        zeros(1,6),'remaining',zeros(1,BTS_num*BTS_channel_num));
    MS(num_calls).x = 0;    % this pre-allocates the structure length

    call_interval = exprnd(call_arrival_mean,[num_calls 1]);   % Poisson process has exponentially distributed inter-arrival times
    call_duration = exprnd(call_duration_mean,[num_calls 1]);  % average hold time = 120 seconds
    
    for i=1:num_calls
        % Ensure all values are initialized (yes the first one is redundant)
        MS(i) = struct('x',0,'y',0,'direction',0,'velocity',0,'call_start_time',0, ...
            'call_end_time',0,'RSS',0,'BTS',0,'active',zeros(1,3),'activetimer',zeros(1,3), ...
            'candidate',zeros(1,6),'candidatetimer',zeros(1,6),'neighbor',zeros(1,6), ...
            'remaining',zeros(1,BTS_num*BTS_channel_num));
        
        % Generate initial x,y coordinates
        rand_magnitude = randi(BTS_radius);
        rand_angle = randi(360);
        rand_BTS = randi(BTS_num);
        MS(i).x = rand_magnitude * cos(pi*rand_angle/180) + BTS(rand_BTS).x;
        MS(i).y = rand_magnitude * sin(pi*rand_angle/180) + BTS(rand_BTS).y;

        % plot call locations
        plot(MS(i).x,MS(i).y,'.k')  
        
        % Generate initial velocities and directions
        MS(i).direction = (unidrnd(360)/180)*pi;                     % radians
        MS(i).velocity = normrnd(velocity_mean,velocity_stddev);     % m/s ?
        while ( MS(i).velocity < 0 || MS(i).velocity > velocity_limit )
            MS(i).velocity = normrnd(velocity_mean,velocity_stddev); % try again
        end

        % Generate call start and end times
        if (i>1)
            MS(i).call_start_time = MS(i-1).call_start_time + call_interval(i-1);   % call start times
        end
        MS(i).call_end_time = MS(i).call_start_time + call_duration(i);             % call end times
    end

end
