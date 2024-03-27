close all;
lb=[0,10,50,20,100,1,1,2600,0.01,0]';
ub=[10, 100, 999, 999, 9999,100,30,8800,1,200]';
x=[1.59808217208494
    31.7019110328425
    214.074569514902
    45.5816806279735
    994.624899765219
    46.7752934390062
    13.7074444953533
    5048.44639263640
    0.987905292847000
    0];
vehicle = 'tiltwing';
payload=300;
range=150000;
labels=["rProp";
"V";
"mBattery";
"mMotors";
"mtow";
"EReserve";
"S";
"rpm";
"eta_motor";
"m_gb"];
for i=1:length(x)
    
    xT=x;
    figure(i);
    for xl=linspace(lb(i),ub(i),50)
        xT(i)= xl;
        [myf,myc,myceq] = computePerformance(xT,vehicle,range,payload);
        yyaxis left;
        pl1=plot(xl,myf,'r*'); hold on;
        yyaxis right;
        pl2=plot(xl,max(myc),'b*'); hold on;

        xT=x;
    end
    [opf,opc,opceq] = computePerformance(x,vehicle,range,payload);
    yyaxis left;
    pl3 = plot(x(i),opf,'ro'); hold on;
    yyaxis right;
    pl4 = plot(x(i),max(opc),'bo'); hold on;
    title(labels(i));
    legend([pl1,pl2],{"Objective","Sum of constraints"});    
end
