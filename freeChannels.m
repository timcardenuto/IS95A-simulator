%% Free channels for BTS's
%
% Frees channels by incrementing BTS free_channel property for each channel
% to be freed. Channel value is the BTS index.
%
% Inputs
%       channels - vector of channels to be freed, values are the BTS #
%       BTS - reference to BTS structure
%
% Outputs
%       BTS - updated BTS structure
%
%#########################################################################

function BTS = freeChannels(channels, BTS)
    for i=1:length(channels)
        if (channels(i) ~= 0)
            BTS(channels(i)).free_channels = BTS(channels(i)).free_channels + 1; 
            disp(['--BTS ',num2str(channels(i)),' free channels ', num2str(BTS(channels(i)).free_channels)]);
        end
    end
end