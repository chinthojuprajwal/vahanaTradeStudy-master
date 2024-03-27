clc;clear;close all;
range=[50000,100000,150000];
load_mass=300;
for i=1:3
% ----------------------Problem setup----------------------------


% ATC Parameters

atc.beta=1.25;
atc.gamma=0.6;

% No. of optimisation blocks in the tree, all elements after the 1st
% element of the bracket are children of first element eg:
% {1,2,{3,4}} is equvivalent to 1 is parent of 2 and 3 and 3 is parent of 4

atc.struct={1,2,3,4};
c_x=[1 1 1 1 1];
% ho=ones([1,16]);
% co=ones([1,15]);
% mo=ones([1,3]);

% rProp = x(1);
% V = x(2);
% mBattery = x(3);
% mMotors = x(4);
% mtow = x(5);
% flight time=x(6);
% enominal=x(7);

% Dependance of each block on above variables set
% All blocks must have same size and zero where var is independent
% atc.sel_vec=[[1 0 0 0 0 1 1 zeros([1,16]) co mo];[1 1 0 0 1 0 0 ho co zeros([1,3])];...
%     [1 0 1 1 1 0 0 ho co mo]];
atc.sel_vec=[[1,1,1,1,1,1,1,1,1,1];[1,1,1,1,1,1,1,1,1,1];[1,1,1,1,1,1,1,1,1,1];[1,1,1,1,1,1,1,1,1,1]];

% specify constraints
atc.constr=[0,1,1,1];

% Specify initial values for ATC
atc.x0=[8, 80, 290, 300, 650,1,1,5000,0.9,16];
% h_lb=[0.005, 0.05, 1, 1, 0.5, 0.1, 0.1, 100, 100, 100, 0, 0, 1e+05,0,3, 1e+05, 1.320334949429926e+03, 2.219013312942524e+05, 2.610603897579440e+05, 8.631939445696098e+02
% h_ub=[0.02,0.3,20,3,3,0.9,0.9,300,300,1000,1,1,2e5,1,10,2e5,
atc.lb=[0,10,50,20,100,1,1,2600,0.01,0]';
atc.ub=[10, 100, 999, 999, 9999,100,30,8800,1,200]';
%atc.x0=atc.lb+rand([10,1]).*(atc.ub-atc.lb);
atc.tol=0.1;
params.ranges=range(i);
params.load=load_mass;
atc=do_ATC_script(atc,params);
result(i)=atc;

        mBreakdownT(i,:) = 1.1*[atc.mass_struct.payload, atc.mass_struct.avionics + ...
            atc.mass_struct.servos + atc.mass_struct.wire + atc.mass_struct.tilt, ...
            atc.mass_struct.wing+atc.mass_struct.canard,...
            atc.mass_struct.props+atc.mass_struct.hub,...
            atc.mass_struct.seat + atc.mass_struct.brs, ...
            atc.mass_struct.battery, atc.mass_struct.motors, ...
            atc.mass_struct.structural+atc.mass_struct.fuselage...
            , atc.mass_struct.gearbox];
         
massTiltWing_sand(i) = sizingTradeStudy(range(i),load_mass);
mBreakdownT_sand(i,:) = 1.1*[massTiltWing_sand(i).payload, massTiltWing_sand(i).avionics + ...
            massTiltWing_sand(i).servos + massTiltWing_sand(i).wire + massTiltWing_sand(i).tilt, ...
            massTiltWing_sand(i).wing+massTiltWing_sand(i).canard,...
            massTiltWing_sand(i).props+massTiltWing_sand(i).hub,...
            massTiltWing_sand(i).seat + massTiltWing_sand(i).brs, ...
            massTiltWing_sand(i).battery, massTiltWing_sand(i).motors, ...
             massTiltWing_sand(i).structural+massTiltWing_sand(i).fuselage...
             ,massTiltWing_sand(i).gearbox];
mBreakdown(i,1,:)= mBreakdownT(i,:);
mBreakdown(i,2,:)= mBreakdownT_sand(i,:);
groupLabels={'50km','100km','150km'};
 

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
atc_res{i}=atc;
clear atc;



end
plotBarStackGroups(mBreakdown, groupLabels);
legend('Payload','Avionics','Wing+Canard','Propeller','Misc','Battery','Motors','Structure','Gearbox','Location','east');
title('EVTOL mass breakdown: SAND (left bar) vs ATC (right bar)');
xlabel('Range [km]')
ylabel('Mass [kg]')
% associate 