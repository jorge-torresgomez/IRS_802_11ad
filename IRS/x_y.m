clear all
close all
clc

fc = 28e9;
c = physconst('lightspeed');
lambda = c/fc;

% Setup surface
Nr = 16;
Nc = 16;
dr = 0.5*lambda;
dc = 0.5*lambda;

% construct surface
ris = helperRISSurface('Size',[Nr Nc],'ElementSpacing',[dr dc],...
    'ReflectorElement',phased.IsotropicAntennaElement,'OperatingFrequency',fc);


% scene
dbr = 50;
pos_ap = [20;-20;3];
pos_ris = [0;0;3]; 
v = zeros(3,1);

du = 50;
dv = -5; % Generating the array of dv values from -35 to -5

du = 50;
center = [20;0;0];
num_points = 3000;

% Generate points around a circle
radius = 3;
% Generate points within the circle
radius = radius * sqrt(rand(1, num_points)); % Randomly generate radii within the circle
angles = 2 * pi * rand(1, num_points); % Randomly generate angles

% Calculate points within the circle with z values set to 0
x_values = center(1) + radius .* cos(angles);
y_values = center(2) + radius .* sin(angles);
z_values = zeros(1,num_points); % Setting z values to 0

% Form the pos_ue_results matrix
pos_ue_results = [x_values; y_values; z_values];
pos_ue_results = [pos_ue_results, center];
% 
% x_pos=0:1:40;
% y_pos=zeros(1,length(x_pos));
% z_pos=zeros(1,length(y_pos));
% 
%     pos_ue_results=[x_pos;y_pos;z_pos];
% pos_ue_results=[pos_ue_results, center];
% compute the range and angle of the RIS from the base station and the UE
[r_ap_ris,ang_ap_ris] = rangeangle(pos_ap,pos_ris);
[r_ue_ris_t,ang_ue_ris_t] = rangeangle(pos_ue_results,pos_ris);


% signal
fs = 10e6;
x = 2*randi(2,[100 1])-3;
tx = phased.Transmitter('PeakPower',50e-3,'Gain',0);
xt = tx(x);
N0dB = -60-30;

% channel
chanAPToRIS = phased.FreeSpace('SampleRate',fs,'PropagationSpeed',c,'MaximumDistanceSource','Property','MaximumDistance',500);
chanRISToUE = phased.FreeSpace('SampleRate',fs,'PropagationSpeed',c,'MaximumDistanceSource','Property','MaximumDistance',500);
chanAPToUE = phased.FreeSpace('SampleRate',fs,'PropagationSpeed',c,'MaximumDistanceSource','Property','MaximumDistance',500);


% channel estimation
stv = getSteeringVector(ris);
a=length(pos_ue_results(1,:));
g = db2mag(-fspl(r_ap_ris,lambda))*exp(1i*2*pi*r_ap_ris/c)*stv(fc,ang_ap_ris);
hr = db2mag(-fspl(r_ue_ris_t(a),lambda))*exp(1i*2*pi*r_ue_ris_t(a)/c)*stv(fc,ang_ue_ris_t(:,a));
% compute optimal phase control
rcoeff_ris = exp(1i*(-angle(hr)-angle(g)));

% rerun simulation

for i=1:length(pos_ue_results(1,:))
    pos_ue=pos_ue_results(:,i);
    ang_ue_ris=ang_ue_ris_t(:,i);
    
x_ris_in = chanAPToRIS(xt,pos_ap,pos_ris,v,v);
x_ris_out = ris(x_ris_in,ang_ap_ris,ang_ue_ris,rcoeff_ris);
yriso = chanRISToUE(x_ris_out,pos_ris,pos_ue,v,v);
SNRriso(i) = pow2db(bandpower(yriso))-N0dB;

end

figure;
Table=[pos_ue_results(1,:)',pos_ue_results(2,:)',SNRriso'];
T=array2table(Table);
s = scatter(T,'Table1','Table2','filled','ColorVariable','Table3');
s.SizeData = 50;
colorbar
title('SNR_{dB}')

hold on
plot(center(1), center(2),'r*');
text(center(1),center(2),' User', 'Color','red','FontSize',10)