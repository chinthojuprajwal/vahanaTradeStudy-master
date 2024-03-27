% Function that loads the configuration sizing trade study results and
% generates some interesting plots 

function mBreakdownT = plotTradeResults_atc()

% Constants
km2m = 1000;

%% Load trade study data
load('tradeStudyResults.mat');
load('mass_atc.mat');

%% Save data in usable format
ranges = ranges';
mH = nan(length(ranges),1); % Helicopter mass
mT = nan(length(ranges),1); % Tilt-wing mass
mBatH = nan(length(ranges),1); % Helicopter battery mass
mBatT = nan(length(ranges),1); % Tilt-wing battery mass
vH = nan(length(ranges),1); % Helicopter cruise speed
vT = nan(length(ranges),1); % Tilt-wing cruise speed
bH = nan(length(ranges),1); % Helicopter span
bT = nan(length(ranges),1); % Tilt-wing span
lH = nan(length(ranges),1); % Helicopter length
lT = nan(length(ranges),1); % Tilt-wing length
cH = nan(length(ranges),1); % Helicopter DOC per flight
cT = nan(length(ranges),1); % Tilt-wing DOC per flight
rH = nan(length(ranges),1); % Helicopter rotor radius
rT = nan(length(ranges),1); % Tilt-wing fan radius
pHH = nan(length(ranges),1); % Helicopter hover power
pHT = nan(length(ranges),1); % Tilt-wing hover power
pCH = nan(length(ranges),1); % Helicopter cruise power
pCT = nan(length(ranges),1); % Tilt-wing cruise power
mBreakdownH = nan(length(ranges),6); % Helicopter mass breakdown
mBreakdownT = nan(length(ranges),7); % Tilt-wing mass breakdown
CBreakdownH = nan(length(ranges),6); % Helicopter cost breakdown
CBreakdownT = nan(length(ranges),6); % Tilt-wing cost breakdown

% Loop through results and pack vectors
for i = 1:length(ranges)
    km2m = 1000;
%     if length(CHelicopter) >= i
%         if ~isempty(massHelicopter(i).m) 
%             mH(i) = massHelicopter(i).m; 
%         end
%         if ~isempty(massHelicopter(i).battery)
%             mBatH(i) = massHelicopter(i).battery; 
%         end
%         vH(i) = VHelicopter(i);
%         bH(i) = 2 * rPropHelicopter(i);
%         lH(i) = 2.25 * rPropHelicopter(i);
%         rH(i) = rPropHelicopter(i);
%         pHH(i) = hoverOutputHelicopter(i).PBattery * 1e-3;
%         pCH(i) = cruiseOutputHelicopter(i).PBattery * 1e-3;
%         mBreakdownH(i,:) = 1.1*[massHelicopter(i).payload, massHelicopter(i).avionics + ...
%             massHelicopter(i).servos + massHelicopter(i).wire, ...
%             massHelicopter(i).seat + massHelicopter(i).brs, ...
%             massHelicopter(i).battery, massHelicopter(i).motors + massHelicopter(i).transmission, ...
%              massHelicopter(i).structural];
%         CBreakdownH(i,:) = [CHelicopter(i).acquisitionCostPerFlight,...
%             CHelicopter(i).insuranceCostPerFlight,...
%             CHelicopter(i).facilityCostPerFlight,...
%             CHelicopter(i).energyCostPerFlight,...
%             CHelicopter(i).batteryReplCostPerFlight + CHelicopter(i).motorReplCostPerFlight + CHelicopter(i).servoReplCostPerFlight, ...
%             CHelicopter(i).laborCostPerFlight];
%         if ~isempty(CHelicopter(i).costPerFlight); cH(i) = CHelicopter(i).costPerFlight; end;
%     end
    if length(CTiltWing) >= i
        if ~isempty(massTiltWing_atc(i).m)
            mT(i) = massTiltWing_atc(i).m; 
        end
        if ~isempty(massTiltWing_atc(i).battery)
            mBatT(i) = massTiltWing_atc(i).battery; 
        end
        vT(i) = VTiltWing(i);
        bT(i) = 8 * rPropTiltWing(i) + 1;
        lT(i) = 4 * rPropTiltWing(i) + 3;
        rT(i) = rPropTiltWing(i);
        pHT(i) = hoverOutputTiltWing(i).PBattery * 1e-3;
        pCT(i) = cruiseOutputTiltWing(i).PBattery * 1e-3;
        mBreakdownT(i,:) = 1.1*[massTiltWing_atc(i).payload, massTiltWing_atc(i).avionics + ...
            massTiltWing_atc(i).servos + massTiltWing_atc(i).wire + massTiltWing_atc(i).tilt, ...
            massTiltWing_atc(i).seat + massTiltWing_atc(i).brs, ...
            massTiltWing_atc(i).battery, massTiltWing_atc(i).motors,massTiltWing(i).gearbox, ...
             massTiltWing_atc(i).structural];
         CBreakdownT(i,:) = [CTiltWing(i).acquisitionCostPerFlight,...
            CTiltWing(i).insuranceCostPerFlight,...
            CTiltWing(i).facilityCostPerFlight,...
            CTiltWing(i).energyCostPerFlight,...
            CTiltWing(i).batteryReplCostPerFlight + CTiltWing(i).motorReplCostPerFlight + CTiltWing(i).servoReplCostPerFlight, ...
            CTiltWing(i).laborCostPerFlight];
        if ~isempty(CTiltWing(i).costPerFlight); cT(i) = CTiltWing(i).costPerFlight; end;
    end
end





%% Mass Breakdown
% figuren('Mass Breakdown'); clf;
% subplot(2,1,1); hold on;
% bar(ranges/km2m, mBreakdownH,'stacked')
% xlim([0,210])
% ylim([0,2000])
% grid on
% xlabel('Range [km]')
% ylabel('Mass [kg]')
% title('Electric Helicopter')
% legend('Payload','Avionics','Misc','Battery','Motors+Transmission','Structure','Location','Best')


%bar(ranges/km2m, mBreakdownT, 'stacked')
% bar(70, mBreakdownT, 'stacked')
% hold on;
% grid on
% xlabel('Range [km]')
% ylabel('Mass [kg]')
% title('Electric Tilt-Wing Multirotor')
% saveas(gcf,'./massBreakdown','png');
% legend('Payload','Avionics','Misc','Battery','Motors','Structure','Location','Best')

end
