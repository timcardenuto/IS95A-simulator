%% IS-95A Fuzzy Logic Controller
% This is a 2 Input, 2 Output, 9 Rule Minimum Inference Engine (MIE)
% with Centroid of Area (COA) Defuzzification
%
% Inputs
%       NOBS - current number of BTS in Active set (of single MS?)
%       CHRM - current number of BTS free channels (of single BTS?)
%
% Outputs
%       T_DROP - threshold power value (dB) where MS starts drop timers
%       SHW    - Soft Handoff Window, distance between add/drop thresholds
%
% Additional input variables to set input/output universe space
%       size_active - size of Active set
%       num_channels - total number of free channels per BTS
%
% We DON'T calculate T_DROP and SHW output COA centers because this
% function gets run for each individual input value... that would be
% crazy. Instead, figure this out once and pass in these parameters.
%
% To use Center Average (CA) for defuzzification, use these values. This is
% also roughly the same as COA....
%       tdrop_low_center = -94;
%       tdrop_medium_center = -92;
%       tdrop_high_center = -90;
% 
%       shw_low_center = 1.5;
%       shw_medium_center = 2.5;
%       shw_high_center = 3.5;
%
% Issues
%       * NOBS MF allows for fractional number of BTS in Active set, which
%       would never happen.... unless the input is suppose to be an average
%       of all MS Active sets or something... not explained by paper.
%       * 
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

function [T_DROP, SHW] = IS95A_fuzzycontroller(NOBS, CHRM, ...
                tdrop_low_center, tdrop_medium_center, tdrop_high_center, ...
                shw_low_center, shw_medium_center, shw_high_center)

    % "Fuzzify" the inputs at the given measurement values
    % this output known as "firing strength" or "rule weight"
    nobs_low_MFgrade = max([0 min([1 (2-NOBS)/(2-1)])]);
    nobs_medium_MFgrade = max([0 min([(NOBS-1)/(2-1) (3-NOBS)/(3-2)])]);
    nobs_high_MFgrade = max([0 min([(NOBS-2)/(3-2) 1])]);

    chrm_low_MFgrade = max([0 min([1 (6-CHRM)/(6-2)])]);
    chrm_medium_MFgrade = max([0 min([(CHRM-2)/(6-2) (10-CHRM)/(10-6)])]);
    chrm_high_MFgrade = max([0 min([(CHRM-6)/(10-6) 1])]);
    
    %  Perform MIE for all rules
    wR = zeros(1,9);
    tdrop_center = 0;
    shw_center = 0;

    % Rule 1, if NOBS is low and CHRM is low then TDROP is high and SHW is high
    wR(1) = min(nobs_low_MFgrade,chrm_low_MFgrade);                % 2 inputs, AND operator (min or *)
    tdrop_center = tdrop_center + wR(1) * tdrop_high_center;    % using CA or COA of output MF
    shw_center = shw_center + wR(1) * shw_high_center;

    % Rule 2, if NOBS is low and CHRM is medium then TDROP is medium and SHW is medium
    wR(2) = min(nobs_low_MFgrade,chrm_medium_MFgrade);
    tdrop_center = tdrop_center + wR(2) * tdrop_medium_center;
    shw_center = shw_center + wR(2) * shw_medium_center;

    % Rule 3, if NOBS is low and CHRM is high then TDROP is low and SHW is low
    wR(3) = min(nobs_low_MFgrade,chrm_high_MFgrade);
    tdrop_center = tdrop_center + wR(3) * tdrop_low_center;
    shw_center = shw_center + wR(3) * shw_low_center;

    % Rule 4, if NOBS is medium and CHRM is low then TDROP is high and SHW is high
    wR(4) = min(nobs_medium_MFgrade,chrm_low_MFgrade); 
    tdrop_center = tdrop_center + wR(4) * tdrop_high_center; 
    shw_center = shw_center + wR(4) * shw_high_center;

    % Rule 5, if NOBS is medium and CHRM is medium then TDROP is medium and SHW is medium
    wR(5) = min(nobs_medium_MFgrade,chrm_medium_MFgrade);
    tdrop_center = tdrop_center + wR(5) * tdrop_medium_center;
    shw_center = shw_center + wR(5) * shw_medium_center;

    % Rule 6, if NOBS is medium and CHRM is high then TDROP is low and SHW is low
    wR(6) = min(nobs_medium_MFgrade,chrm_high_MFgrade);
    tdrop_center = tdrop_center + wR(6) * tdrop_low_center;
    shw_center = shw_center + wR(6) * shw_low_center;

    % Rule 7, if NOBS is high and CHRM is low then TDROP is high and SHW is high
    wR(7) = min(nobs_high_MFgrade,chrm_low_MFgrade); 
    tdrop_center = tdrop_center + wR(7) * tdrop_high_center; 
    shw_center = shw_center + wR(7) * shw_high_center;

    % Rule 8, if NOBS is high and CHRM is medium then TDROP is high and SHW is high
    wR(8) = min(nobs_high_MFgrade,chrm_medium_MFgrade);
    tdrop_center = tdrop_center + wR(8) * tdrop_high_center;
    shw_center = shw_center + wR(8) * shw_high_center;

    % Rule 9, if NOBS is high and CHRM is high then TDROP is medium and SHW is medium
    wR(9) = min(nobs_high_MFgrade,chrm_high_MFgrade);
    tdrop_center = tdrop_center + wR(9) * tdrop_medium_center;
    shw_center = shw_center + wR(9) * shw_medium_center;

    % "Defuzzify" ouput using center average 
    T_DROP = tdrop_center ./ sum(wR);
    SHW = shw_center ./ sum(wR);
end
