%% IS-95A fuzzy controller MF's

clc
clear

size_active = 4;
num_channels = 20;
rss_low = -95;
rss_high = -89;
shw_low = 0;
shw_high = 4;

nobs_x = 0:.02:size_active-.02;
nobs_low = zeros(1,length(nobs_x));
nobs_medium = zeros(1,length(nobs_x));
nobs_high = zeros(1,length(nobs_x));

chrm_x = 0:.1:num_channels-.1;
chrm_low = zeros(1,length(chrm_x));
chrm_medium = zeros(1,length(chrm_x));
chrm_high = zeros(1,length(chrm_x));

tdrop_x = rss_low:.1:rss_high;
tdrop_low = zeros(1,length(tdrop_x));
tdrop_medium = zeros(1,length(tdrop_x));
tdrop_high = zeros(1,length(tdrop_x));

shw_x = shw_low:.01:shw_high;
shw_low = zeros(1,length(shw_x));
shw_medium = zeros(1,length(shw_x));
shw_high = zeros(1,length(shw_x));


for i=1:length(nobs_x)	% NOBS input MF
    nobs_low(i) = max([0 min([1 (2-nobs_x(i))/(2-1)])]);
    nobs_medium(i) = max([0 min([(nobs_x(i)-1)/(2-1) (3-nobs_x(i))/(3-2)])]);
    nobs_high(i) = max([0 min([(nobs_x(i)-2)/(3-2) 1])]);
end

for i=1:length(chrm_x)	% CHRM input MF
    chrm_low(i) = max([0 min([1 (6-chrm_x(i))/(6-2)])]);
    chrm_medium(i) = max([0 min([(chrm_x(i)-2)/(6-2) (10-chrm_x(i))/(10-6)])]);
    chrm_high(i) = max([0 min([(chrm_x(i)-6)/(10-6) 1])]);
end

for i=1:length(tdrop_x) % TDROP output MF
    tdrop_low(i) = max([0 min([1 ((-92)-tdrop_x(i))/((-92)-(-94))])]);
    tdrop_medium(i) = max([0 min([(tdrop_x(i)-(-94))/((-92)-(-94)) ((-90)-tdrop_x(i))/((-90)-(-92))])]);
    tdrop_high(i) = max([0 min([(tdrop_x(i)-(-92))/((-90)-(-92)) 1])]);
end
tdrop_low_coa = trapz(tdrop_low.*tdrop_x)/trapz(tdrop_low);             % Center of Area (COA)
tdrop_medium_coa = trapz(tdrop_medium.*tdrop_x)/trapz(tdrop_medium);    % trapz() 'estimates' integral of a trapazoid
tdrop_high_coa = trapz(tdrop_high.*tdrop_x)/trapz(tdrop_high);
tdrop_low_coa2 = defuzz(tdrop_x,tdrop_low,'centroid');                  % MATLAB fuzzy toolbox method
tdrop_medium_coa2 = defuzz(tdrop_x,tdrop_medium,'centroid');
tdrop_high_coa2 = defuzz(tdrop_x,tdrop_high,'centroid');

for i=1:length(shw_x)	% SHW output MF
    shw_low(i) = trapmf(shw_x(i),[1 1 1.5 2.5]); % doesn't work? -> max([0 min([(shw_x(i)-.9/(1-.9)) 1 (2.5-shw_x(i))/(2.5-1.5)])]);
    shw_medium(i) = max([0 min([(shw_x(i)-1.5)/(2.5-1.5) (3.5-shw_x(i))/(3.5-2.5)])]);
    shw_high(i) = max([0 min([(shw_x(i)-2.5)/(3.5-2.5) 1])]);
end
shw_low_coa = trapz(shw_low.*shw_x)/trapz(shw_low);
shw_medium_coa = trapz(shw_medium.*shw_x)/trapz(shw_medium);
shw_high_coa = trapz(shw_high.*shw_x)/trapz(shw_high);
shw_low_coa2 = defuzz(shw_x,shw_low,'centroid');                  % MATLAB fuzzy toolbox method
shw_medium_coa2 = defuzz(shw_x,shw_medium,'centroid');
shw_high_coa2 = defuzz(shw_x,shw_high,'centroid');

subplot(4,1,1);
hold on
plot(nobs_x,nobs_low);
plot(nobs_x,nobs_medium);
plot(nobs_x,nobs_high);
xlabel('Number of BTS in Active Set');
ylabel('MF Grade');
title('Input NOBS Membership Functions');
legend('Low', 'Medium', 'High')

subplot(4,1,2);
hold on
plot(chrm_x,chrm_low);
plot(chrm_x,chrm_medium);
plot(chrm_x,chrm_high);
xlabel('Number of BTS Free Channels');
ylabel('MF Grade');
title('Input CHRM Membership Functions');
legend('Low', 'Medium', 'High')

subplot(4,1,3);
hold on
plot(tdrop_x,tdrop_low);
plot(tdrop_x,tdrop_medium);
plot(tdrop_x,tdrop_high);
stem(tdrop_low_coa,1,'b')
stem(tdrop_medium_coa,1,'b')
stem(tdrop_high_coa,1,'b')
stem(tdrop_low_coa2,1,'r')
stem(tdrop_medium_coa2,1,'r')
stem(tdrop_high_coa2,1,'r')
xlabel('RSS Power (dB)');
ylabel('MF Grade');
title('Output T\_DROP Membership Functions');
legend('Low', 'Medium', 'High')

subplot(4,1,4);
hold on
plot(shw_x,shw_low);
plot(shw_x,shw_medium);
plot(shw_x,shw_high);
stem(shw_low_coa,1,'b')
stem(shw_medium_coa,1,'b')
stem(shw_high_coa,1,'b')
stem(shw_low_coa2,1,'r')
stem(shw_medium_coa2,1,'r')
stem(shw_high_coa2,1,'r')
xlabel('T\_ADD - T\_DROP difference (dB)');
ylabel('MF Grade');
title('Output SHW Membership Functions');
legend('Low', 'Medium', 'High')

% break;

%% test IS-95A fuzzy controller

tdrop = zeros(length(nobs_x),length(chrm_x));
shw = zeros(length(nobs_x),length(chrm_x));

for i=1:length(nobs_x)
    for j=1:length(chrm_x)
        [tdrop(i,j), shw(i,j)] = IS95A_fuzzycontroller(nobs_x(i),chrm_x(j), ...
            tdrop_low_coa, tdrop_medium_coa, tdrop_high_coa, ...
            shw_low_coa, shw_medium_coa, shw_high_coa);
    end
end

figure()
mesh(chrm_x,nobs_x,tdrop)  % rule surface
ylabel('NOBS');
xlabel('CHRM');
zlabel('T\_DROP');
title('T\_DROP rule surface');

figure()
mesh(chrm_x,nobs_x,shw)  % rule surface
ylabel('NOBS');
xlabel('CHRM');
zlabel('SHW');
title('SHW rule surface');