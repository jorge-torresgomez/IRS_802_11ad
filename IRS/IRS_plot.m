
clear all
close all
clc

fc = 60e9;
c = physconst('lightspeed');
lambda = c/fc;

% Setup surface
Nr = 25;
Nc = 25;
dr = 0.5*lambda;
dc = 0.5*lambda;

% construct surface
ris = helperRISSurface('Size',[Nr Nc],'ElementSpacing',[dr dc],...
    'ReflectorElement',phased.IsotropicAntennaElement,'OperatingFrequency',fc);


% scene
dbr = 50;
pos_ap = [-7;7;1.5];
pos_ris = [0;0;0]; 
v = zeros(3,1);

du = 50;
dv = -5; % Generating the array of dv values from -35 to -5

du = 50;
center = [10;0;0];
num_points = 150000;

% Generate points around a circle
radius = 2;
% Generate points within the circle
radius = radius * sqrt(rand(1, num_points)); % Randomly generate radii within the circle
angles = 2 * pi * rand(1, num_points); % Randomly generate angles

% Calculate points within the circle with z values set to 0
x_values = center(1)*ones(1,num_points); 
y_values = center(2) + radius .* cos(angles);
z_values = center(3) + radius .* sin(angles); % Setting z values to 0

% 
    pos_ue_results=[x_values;y_values;z_values];
pos_ue_results=[pos_ue_results, center];

% 
    pos_ue_results=[x_values;y_values;z_values];
pos_ue_results=[pos_ue_results, center];
% compute the range and angle of the RIS from the base station and the UE
[r_ap_ris,ang_ap_ris] = rangeangle(pos_ap,pos_ris);
[r_ue_ris_t,ang_ue_ris_t] = rangeangle(pos_ue_results,pos_ris);


% signal
fs = 10e6;
x = 2*randi(2,[100 1])-3;
tx = phased.Transmitter('PeakPower',100e-3,'Gain',0);
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


    pos_ue ;
    ang_ue_ris=ang_ue_ris_t(:,i);
    
x_ris_in = chanAPToRIS(xt,pos_ap,pos_ris,v,v);
x_ris_out = ris(x_ris_in,ang_ap_ris,ang_ue_ris,rcoeff_ris);
yriso = chanRISToUE(x_ris_out,pos_ris,pos_ue,v,v);
SNRriso(i) = pow2db(bandpower(yriso))-N0dB;

SNRriso;

end
%% 
figure;
fontsize=15;
Table=[pos_ue_results(2,:)',pos_ue_results(3,:)',SNRriso'];
T=array2table(Table);
s = scatter(T,'Table1','Table2','filled','SizeData', 5,'ColorVariable','Table3');
s.SizeData = 13;
s.MarkerEdgeAlpha = 0; % Set marker edge alpha to 0 for blending
s.MarkerFaceAlpha = 0.6; % Adjust face alpha as needed
colorbar
clim([-10 45])
%title('SNR_{dB}')
set(gca,'FontSize',fontsize);
xlabel('$x$ [m]','Interpreter','latex');
ylabel('$y$ [m]','Interpreter','latex');

hold on
plot(center(2), center(3),'r*');
text(center(2),center(3),' User', 'Color','red','FontSize',fontsize);
axis square
