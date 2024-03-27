% EVTOL MDO via ATC
%Ghanendra K Das

clc
clear
close all
%Initia:lize
beta=15;  %can give a vector
gamma=0.6;    %only one value supported for now
%range=[50:10:100];
range=50000;

for i=1:numel(range)
    for j=1:numel(beta)
       doATC(beta(j),gamma(j),range(i)); %Perform ATC
    end
end

load EVTOL_ATC_data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EVTOL ATC done here
%Ghanendra K Das
%AE , UIUC, 2021

%System level: Performance (Think Upper in hierarchy)
%Subsystem Level 1: Wing design in cruise(Think Lower in hierarchy)
%Subsystem Level 2: Propeller design in Hover(Think Lower in hierarchy)


%U refers to shared variable from Upper passed to lower as target
%L refers to shared variable from Lower passed to upper as target
%All shared variables at the same heirarchical level communicate via system
%level coordination, hence ATC


function doATC(beta,gamma,rangei)

disp(['Beta: ' num2str(beta)])

%INITIALIZE
misc.range=rangei;
misc.vehicle='tiltwing';
misc.payload=300;

FLAG=1;
fObj=inf;
fObjATCs= inf; %s is for performance
fObjATCw= inf;  %w is for wing
fObjATCm= inf;  %m is for motor
fObjATCg= inf;  %g is for gearbox
ITER=1;

%Initial shared variables
   
    mtoww=650;
    Sw=1;
    rpmm=5000;
    eta_motorm=0.9;
    rPropg=8;
    Vg=80;
    mtowg=650;
    Sg=1;
    rpmg=5000;
    eta_motorg=0.9;
    m_gbg=16;


%Initial design variables
x0s=[8, 80, 290, 300, 650,1,1,5000,0.9,16]; % system =[rProp,V,mBattery,MMotor,
%mtow,Ereserve,S, rpm, eta_motor,m_gb]
x0w=[650,1];%Wing:  [mtow, S]
x0m =[5000,0.9]; %  motor: [RPM, efficiency]
x0g=[8,80,650,1,5000,0.9,16];%[rProp,V,mtow,S,rpm,eta_motor,m_gb]

%Initial weights
%system performance
vsU = []; %read as :for parameters passed to s from Upper to S; no upper to s
vsL=0*ones(1,11);  %read as : for parameters passed to p from lower to p
wsU = [];
wsL=.1*ones(1,11);

%Wing
vwU = 0*ones(1,2);%read as :for parameters passed to w from Upper to w
vwL=[]; %read as : for parameters passed to w from lower to w
wwU =.1*ones(1,2);
wwL=[];

%Motor
vmU = 0*ones(1,2);%read as :for paramenters passed to r from Upper tor
vmL=[]; %read as : for parameters passed to r from lower to r
wmU =.1*ones(1,2);
wmL=[];

%Gearbox
vgU = 0*ones(1,7);%read as :for paramenters passed to r from Upper tor
vgL=[]; %read as : for parameters passed to r from lower to r
wgU =.1*ones(1,7);
wgL=[];



%consistency difference. (Target-Response)/Target
s_fold.c=0*vsL ;
w_fold.c=0*vwU;
m_fold.c=0*vmU;
g_fold.c=0*vgU;


%Tolerance criteria
tolc = 0.005;

%BEGIN ATC LOOP
disp(['Range:', num2str(rangei)])
while FLAG==1
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    disp(['ATC iteration : ' num2str(ITER)])
    
    %STEP 1: Run Performance s
    xsU=[];
    xsL=[mtoww,Sw,rpmm,eta_motorm,rPropg,Vg,mtowg,Sg,rpmg,eta_motorg,m_gbg]; %targets for system level
      %Use this to pass anything directly
    [s_f]=Psystem(x0s, xsU,xsL, vsU,vsL,wsU,wsL,misc);
    x0s=s_f.xopt; %update initial point for future use
    
    
    %STEP 2: Update data from upper and Run Wing
    mtows =s_f.mtow ;
    Ss =s_f.S ;
   

    xwU = [mtows,Ss];
    xwL=[];
    [w_f]=Pwing(x0w, xwU,xwL, vwU,vwL,wwU,wwL,misc);
    x0w=w_f.xopt;
    
    %STEP 3: Update from s and Run motor
    rpms =s_f.rpm ;
    eta_motors = s_f.eta_motor;

    xmU = [rpms eta_motors];
    xmL=[];
    [m_f]=Pmotor(x0m, xmU,xmL, vmU,vmL,wmU,wmL,misc);
    x0m=m_f.xopt;    
    
    
    %STEP 4: Update from s and Run gearbox


    
    
    rProps=s_f.rProp;
    Vs=s_f.V;
    mtows=s_f.mtow;
    Ss=s_f.S;
    rpms=s_f.rpm;
    eta_motors=s_f.eta_motor;
    m_gbs=s_f.m_gb;

    
    xgU = [rProps,Vs,mtows,Ss,rpms,eta_motors,m_gbs];
    xgL=[];
    [g_f]=Pgearbox(x0g, xgU,xgL, vgU,vgL,wgU,wgL,misc);
    x0g=g_f.xopt;   
    
    %STEP 6: Update for system level

    
    
    mtoww =w_f.mtow ;
    Sw =w_f.S ;
    rpmm =m_f.rpm ;
    eta_motorm = m_f.eta_motor;
    rPropg=g_f.rProp;
    Vg=g_f.V;
    mtowg=g_f.mtow;
    Sg=g_f.S;
    rpmg=g_f.rpm;
    eta_motorg=g_f.eta_motor;
    m_gbg=g_f.m_gb;

    
    %STEP 7: update v and w
    vsL= updatev(vsL,wsL,s_f.c);
    vwU= updatev(vwU,wwU,w_f.c);
    vmU= updatev(vmU,wmU,m_f.c);
    vgU= updatev(vgU,wgU,g_f.c);

    
    
    %update w based on c in each subsystem
    %Performance, system
    wsL = updatew(wsL, s_fold.c,s_f.c,beta,gamma);
    
    %Wing
    wwU = updatew(wwU, w_fold.c,w_f.c,beta,gamma);
    
    %motor
    wmU = updatew(wmU, m_fold.c,m_f.c,beta,gamma);
    
    
    %gearbox
    wgU = updatew(wgU, g_fold.c,g_f.c,beta,gamma);
    
    %STEP 8: CHECK CONVERGENCE
    %Record iterationwise data
    fObjNew(ITER)=s_f.foriginal;
    fObjATCsNew(ITER) = s_f.fATC;
    fObjATCwNew(ITER) = w_f.fATC;
    fObjATCmNew(ITER) = m_f.fATC;
    fObjATCgNew(ITER) = g_f.fATC; 
 
    
  
    %To plot iteration-wise during execution
    figure(100)
    subplot(2,2,1);
    %plot(1:ITER,fObjNew,'-sb','LineWidth',2,'MarkerFace','b')
    plot(1:ITER,fObjNew,'-s','LineWidth',2);
    xlabel('Iteration')
    ylabel('Objective')
    title(['Objective:', num2str(fObjNew(ITER))]);
    grid on
    set(gca,'XMinorTick','on','YMinorTick','on')
    drawnow

    subplot(2,2,2);
    hold on
    %plot(1:ITER,fObjNew,'-sb','LineWidth',2,'MarkerFace','b')
    plot(ITER,s_f.S,'-sb','LineWidth',2);
    plot(ITER,w_f.S,'-*r','LineWidth',2);
    xlabel('Iteration')
    ylabel('Wing Span')
    legend('System','Wing')
    grid on
    set(gca,'XMinorTick','on','YMinorTick','on')
    drawnow    
    
    
    subplot(2,2,3);
    hold on
    %plot(1:ITER,fObjNew,'-sb','LineWidth',2,'MarkerFace','b')
    plot(ITER,s_f.rpm,'-sb','LineWidth',2);
    plot(ITER,m_f.rpm,'-*r','LineWidth',2);
    xlabel('Iteration')
    ylabel('Motor RPM')
    legend('System','Motor')
    grid on
    set(gca,'XMinorTick','on','YMinorTick','on')
    drawnow    
    
    
    
    subplot(2,2,4);
    hold on
    %plot(1:ITER,fObjNew,'-sb','LineWidth',2,'MarkerFace','b')
    plot(ITER,s_f.m_gb,'-sb','LineWidth',2);
    plot(ITER,g_f.m_gb,'-*r','LineWidth',2);
    xlabel('Iteration')
    ylabel('Mass of gearbox')
    legend('System','Gearbox')
    grid on
    set(gca,'XMinorTick','on','YMinorTick','on')
    drawnow   
    
    %[rProp,V,mBattery,MMotor,mtow,Ereserve,S, rpm, eta_motor,m_gb]
    %GET ALL IterationWise DATA
    %From Performance:
    irProps(ITER) = s_f.rProp;
    iVs(ITER) = s_f.V;
    imBatterys(ITER) = s_f.mBattery;
    imMotors(ITER) =s_f.mMotors;
    imtows(ITER) = s_f.mtow;
    iEreserves(ITER) =s_f.Ereserve;
    iSs(ITER) = s_f.S;
    irpms(ITER) = s_f.rpm;
    ieta_motors(ITER) = s_f.eta_motor;
    im_gbs(ITER) = s_f.m_gb;

    
    
    %From Wing
    imtoww(ITER) = w_f.mtow;
    iSw(ITER) = w_f.S;

    
    
    %From motor
    irpmm(ITER) = m_f.rpm;
    ieta_motorm(ITER) = m_f.eta_motor;
    
    %From gearbox
    irPropg(ITER) = g_f.rProp;
    iVg(ITER) = g_f.V;
    imtowg(ITER) = g_f.mtow;
    iSg(ITER) = g_f.S;
    irpmg(ITER) = g_f.rpm;
    ieta_motorg(ITER) = g_f.eta_motor;
    im_gbg(ITER) = g_f.m_gb;
    

    
    convergence = checkConvergence(fObjNew(ITER), fObj, s_f.c, s_fold.c,...
        w_f.c, w_fold.c,m_f.c, m_fold.c, g_f.c, g_fold.c, tolc);
    if convergence ==1
        break;
    else
        fObj=fObjNew(ITER);
        
        %update c vectors
        s_fold.c=s_f.c;
        w_fold.c=w_f.c;
        m_fold.c=m_f.c;
        g_fold.c=g_f.c;
        disp(['Objective value:' num2str(fObj)]);
        ITER=ITER+1;
    end %convergence check
    
    if ITER>2000  %Aim is to get under 10 iterations. If >30, make changes
        ITER=ITER-1;
        break;
    end
    
    disp('%%%%%%%%%%%%%%      %%%%%%%%%%%%%%%%%%%%%%%')
    
end %ATC LOOP
consum=sum(abs(g_f.c))+sum(abs(w_f.c))+sum(abs(m_f.c))+sum(abs(s_f.c));
disp('%%%%%%%%%%%%%%   ATC COMPLETE    %%%%%%%%%%%%%%%%%%%%%%%')
save EVTOL_ATC_data


% %MAKE PLOTS
% figure(105)
% hold on
% massevtol = imassWs;
% mBreakdownT = 1.1*[massevtol.payload, massevtol.avionics + massevtol.servos...
%     + massevtol.wire + massevtol.tilt, massevtol.seat + massevtol.brs,massevtol.battery, massevtol.motors, massevtol.structural];
% barplot = bar(iranges/1000, mBreakdownT,'stacked','LineWidth',2,'Facecolor','flat');
% grid on
% legend('Payload','Avionics','Misc','Battery','Motors+Transmission','Structure','Location','Best','interpreter','latex','Fontsize',14)
% xlabel('Range, km','interpreter','latex','Fontsize',14);
% ylabel('Weight, kg','interpreter','latex','Fontsize',14)
% for k = 1:numel(mBreakdownT)
%     barplot(k).CData = k;
% end
% 
% %Plot Objective Iteration-wise
%     figure(1)
%     hold on
%     plot(1:ITER,fObjNew,'-s','LineWidth',2,'MarkerFace','b')
%     xlabel('Iteration','Interpreter','latex','fontsize',14)
%     ylabel('MGTOW, kg','Interpreter','latex','fontsize',14)
%     title(['Converged  MGTOW(kg)=' num2str(fObjNew(end))],'interpreter','latex','fontsize',14);
%     grid on
%     set(gca,'XMinorTick','on','YMinorTick','on')
% 
%     %shared variable wing weight
%     figure(2)
%     hold on
%     plot(1:ITER,imwings,'-s','LineWidth',2)
%     plot(1:ITER,imwingw,'-s','LineWidth',2)
%     xlabel('Iteration','Interpreter','latex','fontsize',14)
%     ylabel('Wing weight, kg','Interpreter','latex','fontsize',14)
%     title(['Converged  wing weight(kg)=' num2str(imwings(end))],'interpreter','latex','fontsize',14);
%     legend('System: Performance','Subsystem: Wing','interpreter','latex','fontsize',12)
%     grid on
%     set(gca,'XMinorTick','on','YMinorTick','on')
% 
%     %Motor weights
%     figure(3)
%     hold on
%     plot(1:ITER,immotorrs,'-s','LineWidth',2)
%     plot(1:ITER,immotorws,'-s','LineWidth',2)
%     xlabel('Iteration','Interpreter','latex','fontsize',14)
%     ylabel('Motor weight, kg','Interpreter','latex','fontsize',14)
%     title('Converged  Lift+Cruise motor weights(kg)','interpreter','latex','fontsize',14);
%     legend('Lift motor','Cruise motor','interpreter','latex','fontsize',12)
%     grid on
%     set(gca,'XMinorTick','on','YMinorTick','on')
%     
%     %shared variable Propeller weight
%     figure(4)
%     hold on
%     plot(1:ITER,impropellers,'-s','LineWidth',2)
%     plot(1:ITER,impropellerr,'-s','LineWidth',2)
%     xlabel('Iteration','Interpreter','latex','fontsize',14)
%     ylabel('Propeller weight, kg','Interpreter','latex','fontsize',14)
%     title(['Converged Propeller weights(kg):', num2str(impropellers(end))],'interpreter','latex','fontsize',14);
%     legend('System: Performance','Subsystem: Hover','interpreter','latex','fontsize',12)
%     grid on
%     set(gca,'XMinorTick','on','YMinorTick','on')
%     
%     %shared variable bspan
%     figure(5)
%     hold on
%     plot(1:ITER,ibspans,'-s','LineWidth',2)
%     plot(1:ITER,ibspanw,'-s','LineWidth',2)
%     xlabel('Iteration','Interpreter','latex','fontsize',14)
%     ylabel('Wing span, m','Interpreter','latex','fontsize',14)
%     title(['Converged wing span, m: ', num2str(ibspans(end))],'interpreter','latex','fontsize',14);
%     legend('System: Performance','Subsystem: Wing','interpreter','latex','fontsize',12)
%     grid on
%     set(gca,'XMinorTick','on','YMinorTick','on')    
%     
%     %shared variable rProp
%     figure(6)
%     hold on
%     plot(1:ITER,irProps,'-s','LineWidth',2)
%     plot(1:ITER,irPropw,'-s','LineWidth',2)
%     plot(1:ITER,irPropr,'-s','LineWidth',2)
%     xlabel('Iteration','Interpreter','latex','fontsize',14)
%     ylabel('Propeller radius, m','Interpreter','latex','fontsize',14)
%     title(['Converged Propeller Radius, m: ', num2str(irProps(end))],'interpreter','latex','fontsize',14);
%     legend('System: Performance','Subsystem: Wing','Subsystem: Hover','interpreter','latex','fontsize',12)
%     grid on
%     set(gca,'XMinorTick','on','YMinorTick','on') 
%     
%     %shared variable rProp
%     figure(7)
%     hold on
%     plot(1:ITER,ivelocitys,'-s','LineWidth',2)
%     plot(1:ITER,ivelocityw,'-s','LineWidth',2)
%     xlabel('Iteration','Interpreter','latex','fontsize',14)
%     ylabel('Cruise velocity, m/s: ','Interpreter','latex','fontsize',14)
%     title(['Converged Cruise velocity, m/s: : ', num2str(ivelocitys(end))],'interpreter','latex','fontsize',14);
%     legend('System: Performance','Subsystem: Wing','interpreter','latex','fontsize',12)
%     grid on
%     set(gca,'XMinorTick','on','YMinorTick','on')     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%utility functions

    function w = updatew(wold, cold,cnew,beta,gamma)
        %Updates w based on difference in c from last iteration
        for i =1:numel(wold)
            if norm((cnew(i)))<=gamma*norm((cold(i)))
                w(i) = wold(i);
            else
                w(i)=beta*wold(i);
            end
        end
    end


    function v = updatev(vold, w,c)
        %Updates v
        v= vold+ 2*w.*w.*c;
        
    end


    function v = updatev2(vold,w, cold,cnew,beta,gamma)
        %Updates w based on difference in c from last iteration
        for i =1:numel(vold)
            if norm((cnew(i)))<=0.01*norm((cold(i)))
                v(i) = vold(i);
            else
                v(i)= vold(i)+ w(i).*w(i).*cold(i);
            end
        end
    end

    function convergence = checkConvergence(fObjNew, fObjOld, cs,...
            csold, cw,cwold, cm,cmold, cg,cgold, tol)
        t1 = abs(fObjNew/fObjOld -1);
        t2 = norm(cs./csold-1);
        t3 = norm(cw./cwold-1);
        t4 = norm(cm./cmold-1);
        t5 = norm(cg./cgold-1);
      
       
        if t1<tol && t2<tol && t3<tol && t4<tol && t5<tol
            convergence=1; %true
        else
            convergence=0;  %false
        end
        
        if ~any(cs>tol) && ~any(cw>tol) && ...
                ~any(cm>tol) && ~any(cg>tol) && t1 <tol
            convergence =1;
        end
    end

end