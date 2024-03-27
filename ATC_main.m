clc;clear;close all;
range=[50000,100000,150000];

load_mass=300;
for i=1:3
    
    

mBreakdownT(i,:) = 1.1*[atc.mass_struct.payload, atc.mass_struct.avionics + ...
            atc.mass_struct.servos + atc.mass_struct.wire + atc.mass_struct.tilt, ...
            atc.mass_struct.seat + atc.mass_struct.brs, ...
            atc.mass_struct.battery, atc.mass_struct.motors ...
             atc.mass_struct.structural];
         
[massTiltWing_sand(i),xT] = sizingTradeStudy(range(i),load_mass);
mBreakdownT_sand(i,:) = 1.1*[massTiltWing_sand(i).payload, massTiltWing_sand(i).avionics + ...
            massTiltWing_sand(i).servos + massTiltWing_sand(i).wire + massTiltWing_sand(i).tilt, ...
            massTiltWing_sand(i).seat + massTiltWing_sand(i).brs, ...
            massTiltWing_sand(i).battery, massTiltWing_sand(i).motors ...
             massTiltWing_sand(i).structural];
mBreakdown(i,1,:)= mBreakdownT(i,:);
mBreakdown(i,2,:)= mBreakdownT_sand(i,:);
groupLabels={'50km','100km','150km'};
xT=xT';
 

% figure(27)      
% bar(range(i)/1000, [mBreakdownT(i,:);mBreakdownT_sand(i,:)], 'stacked','grouped')
% hold on;
% bar(range(i)/1000, [mBreakdownT_sand(i,:)], 'stacked','grouped')
% %bar(70, mBreakdownT, 'stacked')
% hold on;
% grid on
% xlabel('Range [km]')
% ylabel('Mass [kg]')
% title('Electric Tilt-Wing Multirotor')
% saveas(gcf,'./massBreakdown','png');
% legend('Payload','Avionics','Misc','Battery','Motors','Structure','Location','Best')
clear atc;


end
plotBarStackGroups(mBreakdown, groupLabels);
legend('Payload','Avionics','Misc','Battery','Motors','Structure','Location','Best');
title('eVTOL mass breakdown: Monolithic (left) vs ATC (right)');
xlabel('Range [km]')
ylabel('Mass [kg]')
% associate 