clc
clear

center = 10000;     % plot center in meters from 0,0 point, first BTS
area = [5000 15000; 5000 15000];

BTS_num = 19;
BTS_radius = 1000;      % meters
BTS_channel_num = 3;   % traffic channels

% build matrix of BTS locations 
BTS_locations = zeros(2,BTS_num);
BTS_locations(:,1) = [ center; center ];    % first BTS in center
num_BTS_in_ring = 6;
i = 2;
for n=1:BTS_num     % add rest of BTS circling out from the first one    
    theta = 0:(2*pi/num_BTS_in_ring):2*pi-(2*pi/num_BTS_in_ring);
    BTS_x = round(2*BTS_radius * cos(theta) + BTS_locations(1,n));  
    BTS_y = round(2*BTS_radius * sin(theta) + BTS_locations(2,n));
    
    j = 1;
    while (i <= BTS_num && j <= length(BTS_x))
        % is this BTS_x, BTS_y already in the list? if not add it
        find_x = find(BTS_locations(1,:)==BTS_x(j)); % returns column index
        find_y = find(BTS_locations(2,:)==BTS_y(j)); % returns column index
        if (isempty(intersect(find_x,find_y)))  % only add BTS if not already in the list
            BTS_locations(:,i) = [ BTS_x(j); BTS_y(j) ];
        	i = i + 1;
        end
        j = j + 1;
    end

end

% TODO replace everything with these structs
BTS = struct('x',0,'y',0,'radius',0,'free_channels',0);
BTS(BTS_num).x = 0;     % pre-allocates structure memory
for n=1:BTS_num
    BTS(n).x = BTS_locations(1,n);
    BTS(n).y = BTS_locations(2,n);
    BTS(n).radius = BTS_radius;
    BTS(n).free_channels = BTS_channel_num;
end
            
            
%% draw BTS locations
map = figure();
hold on
% axis([ 7000 13000 7000 13000])
xlabel('x (meters)');
ylabel('y (meters)');
title('IS-95 Simulation');
% theta = 0:2*pi/6:2*pi;          % circle version by using pi/50 points
theta = pi/2:2*pi/6:2*pi+pi/2;  % hexagon version, 6 points rotated 90 degrees
inclusive_radius = BTS_radius/sin(pi/3); % b/c  if you use the BTS_radius between BTS, you'll end up with non-overlaping circles
for n=1:BTS_num
    text(BTS(n).x-70,BTS(n).y+30,num2str(n))
    xunit = inclusive_radius * cos(theta) + BTS(n).x;
    yunit = inclusive_radius * sin(theta) + BTS(n).y;
    plot(xunit, yunit, 'k');
end


%% Generate mobile call structure and initial data

num_calls = 100;
mobile = struct('x',0,'y',0,'direction',0,'velocity',0,'call_start_time',0,'call_end_time',0,'RSS',0,'BTS',0,'active',zeros(1,3),'activetimer',zeros(1,3),'candidate',zeros(1,6),'candidatetimer',zeros(1,6),'neighbor',zeros(1,6),'remaining',zeros(1,BTS_num*20));
mobile(num_calls).x = 0;    % this pre-allocates the structure length
for i=2:num_calls           % this ensures all values are initialized
    mobile(i) = struct('x',0,'y',0,'direction',0,'velocity',0,'call_start_time',0,'call_end_time',0,'RSS',0,'BTS',0,'active',zeros(1,3),'activetimer',zeros(1,3),'candidate',zeros(1,6),'candidatetimer',zeros(1,6),'neighbor',zeros(1,6),'remaining',zeros(1,BTS_num*20));
end

for i=1:num_calls
    % Generate initial x,y coordinates
    rand_magnitude = randi(BTS_radius);
    rand_angle = randi(360);
    rand_BTS = randi(BTS_num);
    mobile(i).x = rand_magnitude * cos(pi*rand_angle/180) + BTS(rand_BTS).x; % had sym('pi') previously...way faster without it
    mobile(i).y = rand_magnitude * sin(pi*rand_angle/180) + BTS(rand_BTS).y;
    
    % Generate initial velocities and directions
    mobile(i).direction = (unidrnd(360)/180)*pi; % in radians
    mobile(i).velocity = normrnd(6,10);          % m/s ?
    while ( mobile(i).velocity < 0 || mobile(i).velocity > 16 )
        mobile(i).velocity = normrnd(6,10);      % try again
    end

    plot(mobile(i).x,mobile(i).y,'.r')     % plot call locations
end


%% Generate call start, duration, end properties

timeline = figure()                                    % plot a way to visualize call overlap                                   
hold on
xlabel('time (s)')
ylabel('call #')
title('call overlap')
legend('call duration')

call_interval = exprnd(6,[num_calls 1]);    % Poisson process has exponentially distributed inter-arrival times
call_duration = exprnd(120,[num_calls 1]);  % average hold time = 120 seconds
for i=1:num_calls
    if (i>1)
        mobile(i).call_start_time = mobile(i-1).call_start_time + call_interval(i-1);   % call start times
    end
    mobile(i).call_end_time = mobile(i).call_start_time + call_duration(i);         % call end times

    plot([mobile(i).call_start_time mobile(i).call_end_time],[i i], 'k')        
end



%% determine initial BTS assignment for 1st mobile



%% simulation time, 10s update rate
% draw calls as they pop up and connect line to nearest BTS

% initialize first call
current_calls = 1; % track indexes for mobiles that are still in a call
K1 = 0;                     % some constant depending on environment
K2 = 30;                    % BTS radiated power
S = normrnd(0,6.5);         % shadowing effect, zero mean Gaussian w/ standard deviation of 6.5
RSS = zeros(1,BTS_num);
for n = 1:BTS_num           % calculate RSS from each BTS pilot channel
    distance = sqrt((BTS(n).x - mobile(1).x)^2 + (BTS(n).y - mobile(1).y)^2);
    RSS(n) = K1 - K2*log(distance) + S;
end
[RSS, btsindex] = sort(RSS, 'descend'); % sort in order of highest power to lowest power
mobile(current_calls(1)).BTS = btsindex(1);
mobile(current_calls(1)).active(1) = btsindex(1);
BTS(mobile(current_calls(1)).BTS).free_channels = BTS(mobile(current_calls(1)).BTS).free_channels - 1;


% TODO call mobility doesn't seem to be enough to cause
% handoffs, or power levels aren't set well....
T_ADD = -230;   % TODO set these elsewhere
T_DROP = -260;
T_TDROP = 5;


figure(map)
hold on
calls = plot(0,0,'ro');
calls.XDataSource = 'callx';
calls.YDataSource = 'cally';
currentbts = plot(rand(2,num_calls*length(mobile(1).active))); % length has to match the tempx/tempy length, pad them

% for k = 1:numel(currentbts)
%     set(currentbts(k), 'XDataSource', sprintf('currentbtsx(%d,:)', k), ...
%                        'YDataSource', sprintf('currentbtsy(%d,:)', k))
% end

% 
% 
% currentbts.XDataSource = 'currentbtsx';
% currentbts.YDataSource = 'currentbtsy';

% simulation length is total call time
callcount = [];
time_step = 5;
for time=time_step:time_step:mobile(num_calls).call_end_time
    disp(' ');
    disp(['Simulation Time ',num2str(time),' ----------------------']);

    figure(timeline)
    plot([time time],[0 num_calls], '--r')        
    
    figure(map)

    oldx = [];
    oldy = [];
    newx = [];
    newy = [];
    tempx = cell(1,num_calls*length(mobile(1).active))';
    tempy = cell(1,num_calls*length(mobile(1).active))';
    tempc = cell(1,num_calls*length(mobile(1).active))';
    tempx(:)={[center center]};
    tempy(:)={[center center]};
    tempc(:)={'blue'};
    i = 1;
    while ( i < length(current_calls)+1 && isempty(current_calls) == 0)  % loop thru all current calls

        % remove completed calls
        if (mobile(current_calls(i)).call_end_time < time)  % any calls ended?
            %disp('-- removed call')
            % free up channel from BTS being used
            BTS(mobile(current_calls(i)).BTS).free_channels = BTS(mobile(current_calls(i)).BTS).free_channels + 1; 
            current_calls(i) = [];                          % remove call
            continue;                                       % handle next call in while loop
        end
        
        % update mobile position
        %disp('update mobile position')
        distance = time_step * mobile(current_calls(i)).velocity;   % update every 10 seconds

        oldx = [ oldx mobile(current_calls(i)).x ];
        oldy = [ oldy mobile(current_calls(i)).y ];

        mobile(current_calls(i)).x = distance * cos(mobile(current_calls(i)).direction) + mobile(current_calls(i)).x;
        mobile(current_calls(i)).y = distance * sin(mobile(current_calls(i)).direction) + mobile(current_calls(i)).y;

        newx = [ newx mobile(current_calls(i)).x ];
        newy = [ newy mobile(current_calls(i)).y ];

        % update mobile direction
        %disp('update mobile direction')
        mobile(current_calls(i)).direction = (unidrnd(360)/180)*pi; % in radians
       
        % update velocity
        %disp('update mobile velocity')
        mobile(current_calls(i)).velocity = normrnd(6,10);          % m/s ?
        while ( mobile(current_calls(i)).velocity < 0 || mobile(current_calls(i)).velocity > 16 )
            mobile(current_calls(i)).velocity = normrnd(6,10);      % try again
        end
        
        
        %-----------------------------------------------------------------
        % NOTE
        % Because we're just simulating the MS/BTS interactions we don't
        % create and handle the actual messages/handshaking for IS-95A.
        % This section represents a state diagram for MS's to initialize,
        % register with a BTS, maintain the different BTS lists, perform
        % handoffs, deregister when call is done. There are several ways
        % to simulate this process.
        % 
        %
        % SIMPLE
        % Simply assign BTS with highest power Pilot Channel to MS, check 
        % for new highest power BTS every simulation time step. All BTS
        % in the simulation space are treated as a single Set in order of
        % RSS by the MS.
        %
        % REALISTIC
        % The MS maintains multiple sets - Active, Candidate, Neighbor,
        % Remaining. The sets are maintained based on power thresholds
        % T_ADD and T_DROP, with the timer T_TDROP
        %-----------------------------------------------------------------

        % calculate RSS from each BTS pilot channel for current mobile
        K1 = 0;                     % some constant depending on environment
        K2 = 30;                    % BTS radiated power
        S = normrnd(0,6.5);         % shadowing effect, zero mean Gaussian w/ standard deviation of 6.5
        RSS = zeros(1,BTS_num);
        for n = 1:BTS_num,          % Calculate RSS by current MS for each BTS
            distance = sqrt((BTS(n).x - mobile(current_calls(i)).x)^2 + (BTS(n).y - mobile(current_calls(i)).y)^2);
            RSS(n) = K1 - K2*log(distance) + S;
                        
%             disp(['MS ', num2str(current_calls(i))]);
%             disp(['   BTS ', num2str(n)]);
%             disp(['   RSS ', num2str(RSS(n))]);
  
            % This should replace below
            if (RSS(n) > T_ADD)
                if (BTS(n).free_channels == 0)
                    disp(['--BTS ',num2str(n),' has no free channels']);
                    if (find(mobile(current_calls(i)).neighbor==0)),        % ? still add to neighbor list?
                        openindex = find(mobile(current_calls(i)).neighbor==0);
                        mobile(current_calls(i)).neighbor(openindex(1)) = n;   % add BTS to neighbor set
                        disp(['-- added BTS ',num2str(n),' channel to MS ',num2str(current_calls(i)),' Neighbor set']);
                    end
                end
                
                for free=1:BTS(n).free_channels     % assign as many open channels as the MS and BTS support
%                 if (BTS(n).free_channels > 0)      % if this BTS has an open channel
                    if (find(mobile(current_calls(i)).active==0))           % and this MS has an open active set slot
                        openindex = find(mobile(current_calls(i)).active==0);
                        mobile(current_calls(i)).active(openindex(1)) = n;     % add BTS to active set
                        BTS(n).free_channels = BTS(n).free_channels - 1;    % decrement BTS free_channels 
                        disp(['-- added BTS ',num2str(n),' channel to MS ',num2str(current_calls(i)),' Active set']);
                        if (find(mobile(current_calls(i)).candidate==n))  % remove this BTS channel from candidate or neighbor sets if its there
                            index = find(mobile(current_calls(i)).candidate==n);
                            mobile(current_calls(i)).candidate(index(1)) = 0;
                        end
                        if (find(mobile(current_calls(i)).neighbor==n))
                            index = find(mobile(current_calls(i)).neighbor==n);
                            mobile(current_calls(i)).neighbor(index(1)) = 0;
                        end
                    elseif (find(mobile(current_calls(i)).candidate==0)),   % active set is full, check candidate set
                        openindex = find(mobile(current_calls(i)).candidate==0);
                        mobile(current_calls(i)).candidate(openindex(1)) = n;  % add BTS to candidate set
                        disp(['-- added BTS ',num2str(n),' channel to MS ',num2str(current_calls(i)),' Candidate set']);
                        if (find(mobile(current_calls(i)).neighbor==n))    % remove this BTS from neighbor set if it's there
                            index = find(mobile(current_calls(i)).neighbor==n);
                            mobile(current_calls(i)).neighbor(index(1)) = 0;
                        end
                    elseif (find(mobile(current_calls(i)).neighbor==0)),    % candidate set is full, check neighbor set
                        openindex = find(mobile(current_calls(i)).neighbor==0);
                        mobile(current_calls(i)).neighbor(openindex(1)) = n;   % add BTS to neighbor set
                        disp(['-- added BTS ',num2str(n),' channel to MS ',num2str(current_calls(i)),' Neighbor set']);
                    else    % neighbor set is full, add to remaining set
%                         openindex = find(mobile(current_calls(i)).remaining==0);
%                         mobile(current_calls(i)).remaining(openindex(1)) = n;  % add BTS to remaining set
                    end
                end
            end

            if (RSS(n) < T_DROP)
                if (find(mobile(current_calls(i)).active==n))           % if BTS in active 
                    bts = find(mobile(current_calls(i)).active==n);
                    if (mobile(current_calls(i)).activetimer(bts)==0)   % if timer not started
                        mobile(current_calls(i)).activetimer(bts) = 1;
                        disp(['-- STARTING MS ', num2str(current_calls(i)), ' Active channel timer ', num2str(mobile(current_calls(i)).activetimer(bts)), ' for BTS ',num2str(n)]);
                    else                                                % update timer
                        mobile(current_calls(i)).activetimer(bts) = mobile(current_calls(i)).activetimer(bts) + time_step;
                        disp(['-- UPDATING MS ', num2str(current_calls(i)), ' Active channel timer ', num2str(mobile(current_calls(i)).activetimer(bts)), ' for BTS ',num2str(n)]);
                    end

                    if (mobile(current_calls(i)).activetimer(bts) > T_TDROP)    % if the timer has been on longer than T_TDROP
                        disp(['-- EXPIRED MS ', num2str(current_calls(i)), ' Active channel timer ', num2str(mobile(current_calls(i)).activetimer(bts)), ' for BTS ',num2str(n)]);
                        mobile(current_calls(i)).active(bts) = 0;               % drop it from active
                        mobile(current_calls(i)).activetimer(bts) = 0;
                        BTS(n).free_channels = BTS(n).free_channels + 1;        % return freed channel to BTS
                        if (find(mobile(current_calls(i)).neighbor==0)),           % if neighbor set has open slot
                            openindex = find(mobile(current_calls(i)).neighbor==0);
                            mobile(current_calls(i)).neighbor(openindex(1)) = n;   % add BTS to neighbor set
                            disp(['-- moved BTS ',num2str(n),' channel to MS ',num2str(current_calls(i)),' Neighbor set']);
                        else                                                       % neighbor set is full, add to remaining set
%                             openindex = find(mobile(current_calls(i)).remaining==0);
%                             mobile(current_calls(i)).remaining(openindex(1)) = n;  % add BTS to remaining set
%                             disp(['-- moved BTS ',num2str(n),' channel to MS ',num2str(current_calls(i)),' Remaining set']);
                        end
                    end

                elseif (find(mobile(current_calls(i)).candidate==n))    % if BTS in candidate sets
                    index = find(mobile(current_calls(i)).candidate==n);
                    if (mobile(current_calls(i)).candidatetimer(index)==0) % if timer not started
                        mobile(current_calls(i)).candidatetimer(index) = 1;
                    else                                               % update timer
                        mobile(current_calls(i)).candidatetimer(index) = mobile(current_calls(i)).candidatetimer(bts) + time_step;
                    end

                    if (mobile(current_calls(i)).candidatetimer(index) > T_TDROP)    % if the timer has been on longer than T_TDROP
                        mobile(current_calls(i)).candidate(index) = 0;               % drop it from candidate
                        mobile(current_calls(i)).candidatetimer(index) = 0;
                        if (find(mobile(current_calls(i)).neighbor==0)),           % if neighbor set has open slot
                            openindex = find(mobile(current_calls(i)).neighbor==0);
                            mobile(current_calls(i)).neighbor(openindex(1)) = n;   % add BTS to neighbor set
                        else                                                      % neighbor set is full, add to remaining set
%                             openindex = find(mobile(current_calls(i)).remaining==0);
%                             mobile(current_calls(i)).remaining(openindex(1)) = n;  % add BTS to remaining set
                        end
                    end 
                    
                elseif (find(mobile(current_calls(i)).neighbor==n)) % just remove it, there's no neighbor list timers
                    index = find(mobile(current_calls(i)).neighbor==n);
                    mobile(current_calls(i)).candidate(index) = 0; 
                end

            else   % if RSS(n) > T_DROP
                if (find(mobile(current_calls(i)).active==n))            % if BTS in active 
                    index = find(mobile(current_calls(i)).active==n);
                    if (mobile(current_calls(i)).activetimer(index)~=0)  % if timer started
                        mobile(current_calls(i)).activetimer(index) = 0; % stop timer
                        disp('-- STOPING active timer');
                    end

                elseif (find(mobile(current_calls(i)).candidate==n))        % if BTS in candidate sets
                    index = find(mobile(current_calls(i)).candidate==n);
                    if (mobile(current_calls(i)).candidatetimer(index)~=0)  % if timer started
                        mobile(current_calls(i)).candidatetimer(index) = 0; % reset timer
                    end
                end
            end
            

%             disp(['   active ', num2str(mobile(current_calls(i)).active)]);
%             disp(['   candidate ', num2str(mobile(current_calls(i)).candidate)]);
%             disp(' ');
%             pause;
        end  % loops through each BTS connections with 1 MS
        
        % TODO - graph pilot RSS over time for given MS
        
        % plot all active connections (variable)
        for k=1:length(mobile(current_calls(i)).active)
            if (mobile(current_calls(i)).active(k) ~= 0)
%                 plot([mobile(current_calls(i)).x BTS(mobile(current_calls(i)).active(k)).x],[mobile(current_calls(i)).y, BTS(mobile(current_calls(i)).active(k)).y],'--k')
                tempx(((i-1)*length(mobile(current_calls(i)).active))+k) = {[mobile(current_calls(i)).x BTS(mobile(current_calls(i)).active(k)).x]};
                tempy(((i-1)*length(mobile(current_calls(i)).active))+k) = {[mobile(current_calls(i)).y BTS(mobile(current_calls(i)).active(k)).y]};
                if (k==1)
                    tempc(((i-1)*length(mobile(current_calls(i)).active))+k) = {'blue'};
                elseif (k==2 && mobile(current_calls(i)).active(k) == mobile(current_calls(i)).active(k-1))
                    tempc(((i-1)*length(mobile(current_calls(i)).active))+k) = {'red'};
                elseif (k==3 && (mobile(current_calls(i)).active(k) == mobile(current_calls(i)).active(k-1) || mobile(current_calls(i)).active(k) == mobile(current_calls(i)).active(k-2)))
                    tempc(((i-1)*length(mobile(current_calls(i)).active))+k) = {'red'};
                else
                    tempc(((i-1)*length(mobile(current_calls(i)).active))+k) = {'blue'};
                end
            end
        end

        i = i + 1;
    end % loop through all current calls
    
    %---------------------------------------------------------------------
    % update plot of mobiles and previous path
    callx = newx;   % updates current mobile position
    cally = newy;
    refreshdata
    set(currentbts, {'XData'}, tempx)   % plots updated BTS connections 
    set(currentbts, {'YData'}, tempy)   % padded with centerpoint values, since length must be globally constant
    set(currentbts, {'Color'}, tempc)
    for j=1:length(current_calls)   % plots movement for this time_step
        plot([oldx(j) newx(j)],[oldy(j) newy(j)],'r')
    end
    pause;
    %---------------------------------------------------------------------
    % NOTE
    % This section handles adding new MS as the simulation progresses, it
    % repeats some of the previous code to register and assign an initial
    % BTS to the MS.
    %---------------------------------------------------------------------
    
    % add new calls
    %disp('add new mobiles')
    for j=1:num_calls
        if (mobile(j).call_start_time > time-time_step && mobile(j).call_start_time <= time)
           
            % calculate RSS from each BTS pilot channel
            K1 = 0;                     % some constant depending on environment
            K2 = 30;                    % BTS radiated power
            S = normrnd(0,6.5);         % shadowing effect, zero mean Gaussian w/ standard deviation of 6.5
            RSS = zeros(1,BTS_num);
            for n = 1:BTS_num
                distance = sqrt((BTS(n).x - mobile(j).x)^2 + (BTS(n).y - mobile(j).y)^2);
                RSS(n) = K1 - K2*log(distance) + S;
            end
            
            % TODO should evaluate all the same channel/set stuff as above
            % for new calls... make it a function
            
            % TODO the above version needs some fallback or else MS will be
            % left with no BTS channels when the power level of all of them
            % are below T_ADD....is this just something I want to track?
            
            [RSS, btsindex] = sort(RSS, 'descend'); % sort in order of highest power to lowest power
            maxrss = 0;
            for p=1:length(RSS)         % try assigning the BTS with best RSS, if no free channels try next in list
                if (BTS(btsindex(p)).free_channels > 0)   % check that there are free channels on this BTS
                    disp(['-- new channel assigned from BTS ',num2str(btsindex(p)),' to MS ',num2str(j)])
                    BTS(btsindex(p)).free_channels = BTS(btsindex(p)).free_channels - 1;  % occupy channel from new BTS..
                    mobile(j).BTS = btsindex(p);                                          % reassign mobile
                    
                    mobile(j).active(1) = btsindex(p);  % assign first channel
                    
                    maxrss = RSS(p);
                    break;      % done searching for free channel, exit loop
                end          
            end        
            mobile(j).RSS = maxrss;  % update RSS measurement, once a BTS is assigned

            % Check that at least 1 BTS channel assigned
            if (mobile(j).BTS == 0 || mobile(j).RSS == 0)   % could mean all channels are occupied
                disp(['##### MS ',num2str(j),' blocked, no BTS channels available #####'])
                %TODO track blocked call 
                return
            else
                disp('-- added call')
                current_calls = [ current_calls j ];    % add to current call list
                if (mobile(j).RSS < T_DROP)
                    disp(['##### MS ',num2str(j),' assigned BTS channel below T_DROP #####'])
                    %TODO track crappy call metric
                end
            
                % plot all active connections (variable)
                for k=1:length(mobile(j).active)
                    if (mobile(j).active(k) ~= 0)
%                         plot([mobile(j).x BTS(mobile(j).active(k)).x],[mobile(j).y, BTS(mobile(j).active(k)).y],'--k')
                    end
                end
            end
        end
    end % loop num_calls
    
    drawnow;
%     pause;
    pause(0.1);
end
