clc
clear

FIS_on = 0;     % switch to use FIS for T_ADD, T_DROP updates. without this, the simulation basically performs hard handoffs
BTS_num = 19;
BTS_center = 10000;     % plot center in meters from 0,0 point, first BTS
BTS_radius = 1000;      % meters
BTS_channel_num = 10;   % traffic channels
num_calls = 100;
call_arrival_mean = .6;    % seconds, minutes??
call_duration_mean = 120; % seconds
velocity_mean = 6;
velocity_stddev = 10;
velocity_limit = 16;

% TODO call mobility doesn't seem to be enough to cause handoffs, or power levels aren't set well....
T_ADD = -110;
T_DROP = -120;
T_TDROP = 5;


%% Generate Base Station (BTS) data structures and initialize 
BTS = generateBTS(BTS_num, BTS_center, BTS_radius, BTS_channel_num);
            
            
%% draw BTS locations
map = figure();
hold on
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


%% Generate Mobile Station (MS) structures and initialize
% TODO: this function will plot calls on whatever the last figure() was
mobile = generateMS(num_calls, call_arrival_mean, call_duration_mean, ...
                        BTS, BTS_num, BTS_radius, BTS_channel_num, ...
                        velocity_mean, velocity_stddev, velocity_limit, ...
                        T_ADD, T_DROP);


%% Plot a way to visualize call overlap, start/end/duration properties 

timeline = figure();                                 
hold on
xlabel('time (s)')
ylabel('call #')
title('call overlap')
legend('call duration')
for i=1:num_calls
    plot([mobile(i).call_start_time mobile(i).call_end_time],[i i], 'k')        
end


%% determine initial BTS assignment for 1st mobile

% initialize first call
current_calls = 1; % track indexes for mobiles that are still in a call
K1 = -113;                  % some constant depending on environment
K2 = 40;                    % BTS radiated power
RSS = zeros(1,BTS_num);
for n = 1:BTS_num           % calculate RSS from each BTS pilot channel
    S = normrnd(0,6.5);     % shadowing effect, zero mean Gaussian w/ standard deviation of 6.5
    distance = sqrt((BTS(n).x - mobile(1).x)^2 + (BTS(n).y - mobile(1).y)^2);
    RSS(n) = K1 - K2*log(distance/BTS_radius) + S;
end
[RSS, btsindex] = sort(RSS, 'descend'); % sort in order of highest power to lowest power
mobile(current_calls(1)).BTS = btsindex(1);
mobile(current_calls(1)).active(1) = btsindex(1);
BTS(mobile(current_calls(1)).BTS).free_channels = BTS(mobile(current_calls(1)).BTS).free_channels - 1;




%% simulation time, 10s update rate
% draw calls as they pop up and connect line to nearest BTS

rssplot = figure();
hold on
plot(0,RSS,'.k')
xlabel('time (s)');
ylabel('RSS (dB)');
title('RSS calculations for all MS locations');

erlangs = figure();
hold on
grid on
er1 = subplot(2,1,1);
xlabel('time (s)');
ylabel('# of channels');
title('Channel Usage Metric');
legend('Active Set channels','remaining free channels')
grid on
hold on
er2 = subplot(2,1,2);
xlabel('time (s)');
ylabel('# of channels (percent)');
title('Channel Usage Metric (Percent)');


figure(map)
hold on
calls = plot(0,0,'ob');
calls.XDataSource = 'callx';
calls.YDataSource = 'cally';
currentbts = plot(rand(2,num_calls*length(mobile(1).active))); % length has to match the tempx/tempy length, pad them


% simulation length is total call time
callcount = [];
time_step = 5;
lasttime = 0;
num_blockedcalls = 0;
num_droppedcalls = 0;
num_handoffs = 0;
num_handoffs2 = 0;
num_blockedhandoffs = 0;
num_crappycalls = 0;
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
    tempx(:)={[BTS_center BTS_center]};
    tempy(:)={[BTS_center BTS_center]};
    tempc(:)={'blue'};
    
    rss = RSS;
    num_active = 0;
    
    i = 1;
    while ( i < length(current_calls)+1 && isempty(current_calls) == 0)  % loop thru all current calls

        % remove completed calls
        if (mobile(current_calls(i)).call_end_time < time)
            BTS = freeChannels(mobile(current_calls(i)).active, BTS); % free channels from BTS being used
            current_calls(i) = [];                          % remove call
            disp('-- removed call')
            continue;                                       % handle next call in while loop
        end
        
        % update mobile position, velocity, direction
%         oldx = [ oldx mobile(current_calls(i)).x ];
%         oldy = [ oldy mobile(current_calls(i)).y ];
        [mobile(current_calls(i)), lasttime] = updateMS(mobile(current_calls(i)), ...
                            time, time_step, lasttime, ...
                            velocity_mean, velocity_stddev, velocity_limit);
        newx = [ newx mobile(current_calls(i)).x ];
        newy = [ newy mobile(current_calls(i)).y ];                    
        
        
        %-----------------------------------------------------------------
        % NOTE
        % Because we're just simulating the MS/BTS interactions we don't
        % create and handle the actual messages/handshaking for IS-95A.
        % This section represents a state diagram for MS's to initialize,
        % register with a BTS, maintain the different BTS lists, perform
        % handoffs, deregister when call is done. There are several ways
        % to simulate this process.
        % 
        % The MS maintains multiple sets - Active, Candidate, Neighbor,
        % Remaining. The sets are maintained based on power thresholds
        % T_ADD and T_DROP, with the timer T_TDROP
        %-----------------------------------------------------------------

        % calculate RSS from each BTS pilot channel for current mobile
        K1 = -113;                  % some constant depending on environment
        K2 = 40;                    % BTS radiated power
        RSS = zeros(1,BTS_num);
        disp(['MS ', num2str(current_calls(i))]);
        for n = 1:BTS_num,          % Calculate RSS by current MS for each BTS
            S = normrnd(0,6.5);         % shadowing effect, zero mean Gaussian w/ standard deviation of 6.5
            distance = sqrt((BTS(n).x - mobile(current_calls(i)).x)^2 + (BTS(n).y - mobile(current_calls(i)).y)^2);
            RSS(n) = K1 - K2*log(distance/BTS_radius) + S;
                        
%             disp(['   BTS ', num2str(n)]);
%             disp(['   RSS ', num2str(RSS(n))]);
  
            % This should replace below
            if (RSS(n) > mobile(current_calls(i)).T_ADD)
                if (BTS(n).free_channels == 0)                                % if there are no free channels...
                    if (length(find(mobile(current_calls(i)).active~=0)) < length(mobile(current_calls(i)).active)) % and I have free Active Set slots...
                        num_blockedhandoffs = num_blockedhandoffs + 1;        % treat as a blocked handoff, since I would have asked to use this BTS
                        disp(['##### MS ',num2str(current_calls(i)),' handoff to BTS ',num2str(n),' blocked, no channels available #####'])
                        if (find(mobile(current_calls(i)).neighbor==0)),        % ? still add to neighbor list?
                            openindex = find(mobile(current_calls(i)).neighbor==0);
                            mobile(current_calls(i)).neighbor(openindex(1)) = n;   % add BTS to neighbor set
                            disp(['-- added BTS ',num2str(n),' channel to MS ',num2str(current_calls(i)),' Neighbor set']);
                        end
                    end
                end
                
%                 for free=1:BTS(n).free_channels     % assign as many open channels as the MS and BTS support
                if (BTS(n).free_channels > 0)      % if this BTS has an open channel
                    
                    % this MS has an open active set slot
                    if (find(mobile(current_calls(i)).active==0))           
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
                        
                        % if this was a new BTS (not just channel) being added then count as handoff
                        if(length(find(mobile(current_calls(i)).active==n)) == 1)
                            num_handoffs = num_handoffs + 1;
                        end
                       
                        % add T_SEL checks
                        
                    % active set is full, but this new channel might have better RSS than what I have already
                    else 
                        rss = zeros(1,length(mobile(current_calls(i)).active));
                        for q=1:length(mobile(current_calls(i)).active)
                            S = normrnd(0,6.5);         
                            distance = sqrt((BTS(mobile(current_calls(i)).active(q)).x - mobile(current_calls(i)).x)^2 + (BTS(mobile(current_calls(i)).active(q)).y - mobile(current_calls(i)).y)^2);
                            rss(q) = K1 - K2*log(distance/BTS_radius) + S;
                        end
                        [val idx] = min(rss);
                        if (RSS(n) > val)  % replace weakest RSS channel, if new one is stronger
                            oldbts = mobile(current_calls(i)).active(idx);
                            BTS(oldbts).free_channels = BTS(oldbts).free_channels + 1; % return freed channel to old BTS
                            BTS(n).free_channels = BTS(n).free_channels - 1;        % grab freed channel from new BTS
                            mobile(current_calls(i)).active(idx) = n;               % replace old with new
                            mobile(current_calls(i)).activetimer(idx) = 0;
                            num_handoffs2 = num_handoffs2 + 1; % this counts as a handoff
                            
                            % also shift old channel down to candidate set
                            if (find(mobile(current_calls(i)).candidate==0)),   % check candidate set
                                openindex = find(mobile(current_calls(i)).candidate==0);
                                mobile(current_calls(i)).candidate(openindex(1)) = oldbts;  % add BTS to candidate set
                                disp(['-- added BTS ',num2str(n),' channel to MS ',num2str(current_calls(i)),' Candidate set']);
                            end
                            
                        elseif (find(mobile(current_calls(i)).candidate==0)),   % check candidate set
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
            end

            if (RSS(n) < mobile(current_calls(i)).T_DROP)
                % update process to remove low power BTS from Active Set
                if (find(mobile(current_calls(i)).active==n))           % if BTS in active 
                    bts = find(mobile(current_calls(i)).active==n);
                    for q=1:length(bts)                                 % b/c could have more than 1 channel from same BTS
                        if (mobile(current_calls(i)).activetimer(bts(q))==0)   % if timer not started
                            mobile(current_calls(i)).activetimer(bts(q)) = 1;
                            disp(['-- STARTING MS ', num2str(current_calls(i)), ' Active channel timer ', num2str(mobile(current_calls(i)).activetimer(bts(q))), ' for BTS ',num2str(n)]);
                        else                                                % update timer
                            mobile(current_calls(i)).activetimer(bts(q)) = mobile(current_calls(i)).activetimer(bts(q)) + time_step;
                            disp(['-- UPDATING MS ', num2str(current_calls(i)), ' Active channel timer ', num2str(mobile(current_calls(i)).activetimer(bts(q))), ' for BTS ',num2str(n)]);
                        end

                        if (mobile(current_calls(i)).activetimer(bts(q)) > T_TDROP)    % if the timer has been on longer than T_TDROP
                            disp(['-- EXPIRED MS ', num2str(current_calls(i)), ' Active channel timer ', num2str(mobile(current_calls(i)).activetimer(bts(q))), ' for BTS ',num2str(n)]);
                            mobile(current_calls(i)).active(bts(q)) = 0;               % drop it from active
                            mobile(current_calls(i)).activetimer(bts(q)) = 0;
                            BTS(n).free_channels = BTS(n).free_channels + 1;           % return freed channel to BTS
                                       
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
                    end
                
                % update process to remove low power BTS from Candidate Set
                elseif (find(mobile(current_calls(i)).candidate==n))    % if BTS in candidate sets
                    index = find(mobile(current_calls(i)).candidate==n);
                    if (mobile(current_calls(i)).candidatetimer(index)==0) % if timer not started
                        mobile(current_calls(i)).candidatetimer(index) = 1;
                    else                                               % update timer
                        mobile(current_calls(i)).candidatetimer(index) = mobile(current_calls(i)).candidatetimer(index) + time_step;
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
                    
                % update process to remove low power BTS from Neighbor Set    
                elseif (find(mobile(current_calls(i)).neighbor==n)) % just remove it, there's no neighbor list timers
                    index = find(mobile(current_calls(i)).neighbor==n);
                    mobile(current_calls(i)).candidate(index) = 0; 
                end

            else   % if RSS(n) > mobile(current_calls(i)).T_DROP
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
        end  % loops through each BTS connections with 1 MS
        
        disp(['   active ', num2str(mobile(current_calls(i)).active)]);
%         disp(['   candidate ', num2str(mobile(current_calls(i)).candidate)]);
        disp(' ');
        
        % Check that the MS still has at least 1 BTS in active set, if not drop the call
        if (sum(mobile(current_calls(i)).active) == 0)
            disp(['##### MS ',num2str(current_calls(i)),' call dropped, no active channels #####'])
            num_droppedcalls = num_droppedcalls + 1;
            plot(mobile(current_calls(i)).x, mobile(current_calls(i)).y,'xm')
            current_calls(i) = [];                          % drop call
            continue;                                       % handle next call in while loop
        end
        
        % plot all active connections (variable)
        % dual use function - track number of active channels for metrics
        % triple use function - get NOBS value for FIS
        NOBS = 0;                                                   %**************** FIS relevant code ****************%
        for k=1:length(mobile(current_calls(i)).active)
            if (mobile(current_calls(i)).active(k) ~= 0)
%                 plot([mobile(current_calls(i)).x BTS(mobile(current_calls(i)).active(k)).x],[mobile(current_calls(i)).y, BTS(mobile(current_calls(i)).active(k)).y],'--k')
                tempx(((i-1)*length(mobile(current_calls(i)).active))+k) = {[mobile(current_calls(i)).x BTS(mobile(current_calls(i)).active(k)).x]};
                tempy(((i-1)*length(mobile(current_calls(i)).active))+k) = {[mobile(current_calls(i)).y BTS(mobile(current_calls(i)).active(k)).y]};
                if (k==1)
                    tempc(((i-1)*length(mobile(current_calls(i)).active))+k) = {'blue'};
                elseif (k>1)
                    tempc(((i-1)*length(mobile(current_calls(i)).active))+k) = {'green'};
                else
                    tempc(((i-1)*length(mobile(current_calls(i)).active))+k) = {'blue'};
                end
                
                NOBS = NOBS + 1;                                    %**************** FIS relevant code ****************%
                num_active = num_active + 1;
            end
        end

        if (FIS_on)
            % Calculate # of free channels for all BTS, input to FIS    %**************** FIS relevant code ****************%
            CHRM = 0;
            for q=1:BTS_num
               CHRM = CHRM + sum(BTS(q).free_channels);
            end
%             tdrop_low_center = -94;         % Center Average (CA) of output MF's
%             tdrop_medium_center = -92;      % these can be shown to be roughly equal to Centroid of Average (COA)
%             tdrop_high_center = -90;        % because the MF are triangle functions
            
            tdrop_low_center = -125;         % My choices
            tdrop_medium_center = -115;
            tdrop_high_center = -105; 

            shw_low_center = 1;
            shw_medium_center = 5;
            shw_high_center = 10;
            % Actual FIS
            [mobile(current_calls(i)).T_DROP, SHW] = IS95A_fuzzycontroller(NOBS, CHRM, ...
                    tdrop_low_center, tdrop_medium_center, tdrop_high_center, ...
                    shw_low_center, shw_medium_center, shw_high_center);
            mobile(current_calls(i)).T_ADD = mobile(current_calls(i)).T_DROP + SHW;                                       %**************** FIS relevant code ****************%
        end
        
        rss = [rss RSS];
        
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
%     for j=1:length(current_calls)   % plots movement for this time_step
%         plot([oldx(j) newx(j)],[oldy(j) newy(j)],'r')
%     end
    
    
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
            K1 = -113;                  % some constant depending on environment
            K2 = 40;                    % BTS radiated power
            RSS = zeros(1,BTS_num);
            for n = 1:BTS_num
                S = normrnd(0,6.5);         % shadowing effect, zero mean Gaussian w/ standard deviation of 6.5
                distance = sqrt((BTS(n).x - mobile(j).x)^2 + (BTS(n).y - mobile(j).y)^2);
                RSS(n) = K1 - K2*log(distance/BTS_radius) + S;
            end
            rss = [rss RSS];
            % TODO should evaluate all the same channel/set stuff as above
            % for new calls... make it a function
            
            % TODO the above version needs some fallback or else MS will be
            % left with no BTS channels when the power level of all of them
            % are below T_ADD....is this just something I want to track?
            
            [RSS, btsindex] = sort(RSS, 'descend'); % sort in order of highest power to lowest power
            for p=1:length(RSS)         % try assigning the BTS with best RSS, if no free channels try next in list
                if (BTS(btsindex(p)).free_channels > 0)   % check that there are free channels on this BTS
                    if (RSS(p) > mobile(j).T_DROP)
                        disp(['-- new channel assigned from BTS ',num2str(btsindex(p)),' to MS ',num2str(j)])
                        BTS(btsindex(p)).free_channels = BTS(btsindex(p)).free_channels - 1;  % occupy channel from new BTS..
                        mobile(j).BTS = btsindex(p);                                          % reassign mobile
                        mobile(j).RSS = RSS(p); % record *first* RSS measurement, once a BTS is assigned

                        mobile(j).active(1) = btsindex(p);  % assign first channel
                        current_calls = [ current_calls j ];    % add to current call list
                        num_active = num_active + 1;
                        %TODO plot my connection
                        break;      % done searching for free channel, exit loop                        
                    end
                end          
            end        

            % Check that at least 1 BTS channel assigned
            if (mobile(j).BTS == 0 || mobile(j).RSS < mobile(j).T_DROP)   % could mean all channels are occupied
                disp(['##### MS ',num2str(j),' call blocked, no BTS channels available #####'])
                num_blockedcalls = num_blockedcalls + 1;
                plot(mobile(j).x, mobile(j).y,'xr')       
            end
        end
    end % loop num_calls
    
    
    
    % Plot metrics, perform error checks
    figure(erlangs)
    hold on
    grid on
    plot(er1,time,num_active,'*k')
    grid on
    hold on
    total_free = 0;
    for q=1:BTS_num
       total_free = total_free + sum(BTS(q).free_channels);
    end
    plot(er1,time,total_free,'*r')
    plot(er2,time,num_active/(BTS_num*BTS_channel_num),'*k')

    % error check, this should never happen
    if (num_active+total_free ~= BTS_num*BTS_channel_num)
        disp('##### ERROR: Num active + free channels != total number of channels #####')
        return
    end
    
    % Plot RSS over time, use to determine average, dynamic range
    figure(rssplot)
    hold on
    plot(time,rss,'.k')
    
    drawnow;
    
%     if(time == time_step)   % one time pause to let me move windows around
%         pause
%     end
    pause(0.1);
end


num_blockedcalls
num_droppedcalls
num_handoffs
num_handoffs2
num_blockedhandoffs
num_crappycalls

