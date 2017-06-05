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
% License
%    Copyright (C) 2017 Tim Cardenuto
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program. If not, see <http://www.gnu.org/licenses/>.
%#########################################################################

function BTS = freeChannels(channels, BTS)
    for i=1:length(channels)
        if (channels(i) ~= 0)
            BTS(channels(i)).free_channels = BTS(channels(i)).free_channels + 1; 
            disp(['--BTS ',num2str(channels(i)),' free channels ', num2str(BTS(channels(i)).free_channels)]);
        end
    end
end
