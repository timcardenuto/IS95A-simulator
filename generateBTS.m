%% BTS location generator
% 
% Inputs
%       num_BTS - number of BTS to be used in simulation
%       center - center point of first BTS, the rest are circled outward
%       radius - effective radius of each BTS
%       num_channels - total number of channels of each BTS
%
% Outputs
%       BTS - structure array of each BTS with the following properties
%           x
%           y
%           free_channels
%
%#########################################################################

function BTS = generateBTS(num_BTS, center, radius, num_channels)

    BTS_locations = zeros(2,num_BTS);
    BTS_locations(:,1) = [ center; center ];    % first BTS in center
    num_BTS_in_ring = 6;
    i = 2;
    for n=1:num_BTS     % add rest of BTS circling out from the first one    
        theta = 0:(2*pi/num_BTS_in_ring):2*pi-(2*pi/num_BTS_in_ring);
        BTS_x = round(2*radius * cos(theta) + BTS_locations(1,n));  
        BTS_y = round(2*radius * sin(theta) + BTS_locations(2,n));

        j = 1;
        while (i <= num_BTS && j <= length(BTS_x))
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

    BTS = struct('x',0,'y',0,'free_channels',0);
    BTS(num_BTS).x = 0;     % pre-allocates structure memory
    for n=1:num_BTS
        BTS(n).x = BTS_locations(1,n);
        BTS(n).y = BTS_locations(2,n);
        BTS(n).free_channels = num_channels;
    end
    
end
