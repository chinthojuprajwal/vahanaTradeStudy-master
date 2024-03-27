% Wing ATC on a conventional Jet, v10, 11 June
%Ghanendra K Das
%A

clc
clear
close all

%Initia:lize
beta=[20];  %can give a vector
gamma=[0.4];    %only one value supported for now

for i=1:numel(gamma)
    for j=1:numel(beta)
        [ITER(j)] =doATC(beta(j),gamma(i)); %Perform ATC
    end
end


% figure(10)
% hold on
% plot(beta, ITER,'-sr','LineWidth',2,'MarkerFace','r')
% xlabel('beta')
% ylabel('Total number of ATC iterations')
% title(['ATC convergence']);
% grid on
% set(gca,'XMinorTick','on','YMinorTick','on')

%Reload final data to make changes if required
load dg3to1data


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Wing ATC done here , v10, 11 June
%Ghanendra K Das
%AE , UIUC, 2021

%System level: Performance (Think Upper in hierarchy)
%Subsystem Level: Wing(Think Lower in hierarchy)

%LoDU refers to shared variable from Upper passed to lower as target
%LoDL refers to shared variable from Lower passed to upper as target


function [ITER, fObjNew, fObjATCpNew, fObjATCwNew,...
    iLoDU, iWwingU, iSU, iW1U, iWfuelU, iW0U, iLoDL,...
    iWwingL, iSL, iWfuelL, iW0L, irootAOA, iCL, ispan, iAR]=doATC(beta,gamma)

disp(['Beta: ' num2str(beta)])

%INITIALIZE
LoDL=10;
WwingL=1200;
SL=245;
LoDU=10;
WwingU = 100;
SU = 278;

FLAG=1;
fObj=inf;
fObjATCp= inf; %p is for performance
fObjATCw= inf;  %w is for wing
ITER=1;

%Initial design variables
x0U=[11 1200 245]; % design variables at upper: L/D, Wwing,S
x0L=[6 24.5 2]; % design variables at lower

%Initial weights
%Performance
vpU = []; %read as :for paramenters passed to p from Upper to p; no upper to p
vpL=0*[1,1,1];  %read as : for parameters passed to p from lower to p
wpU = [];
wpL=0.01*[1,1,1];

%Wing
vwU = 0*[1,1,1];%read as :for paramenters passed to w from Upper to w
vwL=[]; %read as : for parameters passed to w from lower to w
wwU =0.01*[1,1,1];
wwL=[];


%consistency difference. (Target-Response)/Target
p_fold.c=0*[1 1 1];
w_fold.c=0*[1 1 1];

%To store total function evaluations
ifunccountL=0;
ifunccountU=0;

%Tolerance criteria
tolc = 0.001;

%BEGIN ATC LOOP
while FLAG==1
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    disp(['ATC iteration : ' num2str(ITER)])
    
    %STEP 1: Run Performance
    xU=[];
    xL= [LoDL WwingL SL]; %targets from wing
    miscp=[];  %Use this to pass anything directly
    [p_f]=sperf(x0U, xU,xL, vpU,vpL,wpU,wpL,miscp);
    x0U=p_f.xopt; %update initial point for future use
    
    
    %STEP 2: Update data from upper and Run Wing
    LoDU=p_f.LoD;
    WwingU=p_f.Wwing;
    SU=p_f.S;
    miscw(1) = p_f.W0; %For now, supply W0, Wfuel directly
    miscw(2) = p_f.Wfuel;
    xwU = [LoDU WwingU SU];
    xwL=[];
    [w_f]=swing(x0L, xwU,xwL, vwU,vwL,wwU,wwL,miscw);
    x0L=w_f.xopt;
    
    %NOTE: Repeat 2 for each subsystem in future. Each subsystem can be executed in parallel.
    %But this require no update between subsystems. What would be if allowed?
    
    %STEP 3:Update data from Wing.
    LoDL=w_f.LoD;
    WwingL=w_f.Wwing;
    SL =w_f.S;
    
    %STEP 4: update v and w
    vpL= updatev(vpL,wpL,p_f.c);
    vwU= updatev(vwU,wwU,w_f.c);
    
    %     vpL=updatev2(vpL,wpL, p_fold.c,p_f.c,beta,gamma)
    %     vwU=updatev2(vwU,wpL, w_fold.c,w_f.c,beta,gamma)
    
    
    %update w based on c in each subsystem
    %Performance
    wpL = updatew(wpL, p_fold.c,p_f.c,beta,gamma)
    %Wing
    wwU = updatew(wwU, w_fold.c,w_f.c,beta,gamma)
    
    
    %STEP 5: CHECK CONVERGENCE
    %Record iterationwise data
    fObjNew(ITER)=p_f.foriginal;
    fObjATCpNew(ITER) = p_f.fATC;
    fObjATCwNew(ITER) = w_f.fATC;
    
    %from Performance
    iLoDU(ITER)=p_f.LoD;
    iWwingU(ITER) = p_f.Wwing;
    iSU(ITER) = p_f.S;
    iW1U(ITER) = p_f.W1;
    iWfuelU(ITER) = p_f.Wfuel;
    iW0U(ITER) = p_f.W0;
    ifunccountU = ifunccountU+p_f.funcCount;
    
    %from Wing
    iLoDL(ITER)=w_f.LoD;
    iWwingL(ITER) = w_f.Wwing;
    iSL(ITER) = w_f.S;
    
    iWfuelL(ITER) = w_f.Wfuel;
    iW0L(ITER) = w_f.W0;
    irootAOA(ITER) = w_f.rootaoa;
    iCL(ITER) = w_f.CL;
    ispan(ITER) = w_f.bspan;
    iAR(ITER) = w_f.AR;
    itotallift(ITER) = w_f.totalLift;
    iwconstr1(ITER) = w_f.constr1;
    iwconstr2(ITER) = w_f.constr2;
    iwconstreq(ITER) = w_f.constreq;
    ifunccountL = ifunccountL+w_f.funcCount;
    
    %To plot iteration-wise during execution
    figure(100)
    %plot(1:ITER,fObjNew,'-sb','LineWidth',2,'MarkerFace','b')
        plot(1:ITER,fObjNew,'-s','LineWidth',2);
    xlabel('Iteration')
    ylabel('Objective')
    title(['Objective:', num2str(fObjNew(ITER))]);
    grid on
    set(gca,'XMinorTick','on','YMinorTick','on')
    drawnow
    
    convergence = checkConvergence(fObjNew(ITER), fObj, p_f.c,p_fold.c,w_f.c,w_fold.c, tolc);
    if convergence ==1
        break;
    else
        fObj=fObjNew(ITER);
        fObjATCp = fObjATCpNew(ITER);
        fObjATCw = fObjATCwNew(ITER);
        %update c vectors
        p_fold.c=p_f.c;
        w_fold.c=w_f.c;
        disp(['Objective value:' num2str(fObj)]);
        ITER=ITER+1;
    end %convergence check
    
    if ITER>30  %Aim is to get under 10 iterations. If >30, make changes
        ITER=ITER-1;
        break;
    end
    
    disp('%%%%%%%%%%%%%%      %%%%%%%%%%%%%%%%%%%%%%%')
    
end %ATC LOOP

disp('%%%%%%%%%%%%%%   ATC COMPLETE    %%%%%%%%%%%%%%%%%%%%%%%')

%Write to file for each beta value
writetodat(ITER,beta, fObjNew(:), fObjATCpNew(:), fObjATCwNew(:),...
    iLoDU(:), iLoDL(:), iWwingU(:),iWfuelL(:), iSU(:),iSL(:), iW1U(:), iWfuelU(:), iW0U(:),...
    iWwingL(:),   iW0L(:), irootAOA(:), iCL(:), ispan(:), iAR(:),ifunccountU(:),...
    ifunccountL(:),iwconstr1(:),iwconstr2(:),iwconstreq(:) );

%MAKE PLOTS
if ITER<30 %whether to plot non-convergent values
    %Objective: W0
    figure(1)
    hold on
  %  plot([1:ITER],fObjNew,'-sb','LineWidth',1,'MarkerFace','b')
  plot([1:ITER],fObjNew,'-s','LineWidth',2);
    xlabel('ATC Iteration')
    ylabel('MGTOW , lb')
    title(['Converged  MGTOW(lb)=' num2str(fObjNew(end))]);
   % text(2, fObjNew(2)+100,num2str(beta),'Color','red','FontSize',14);
    grid on
    set(gca,'XMinorTick','on','YMinorTick','on')
    
    
    %Aspect ratio
    figure(2)
    hold on
    %plot([1:ITER],iAR,'-sb','LineWidth',1,'MarkerFace','b')
    plot([1:ITER],iAR,'-s','LineWidth',2);
    xlabel('ATC Iteration')
    ylabel('Aspect Ratio')
    title(['Converged  AR=' num2str(iAR(end))]);
   % text(ITER, iAR(ITER)+0.5,num2str(beta),'Color','red','FontSize',14);
    grid on
    set(gca,'XMinorTick','on','YMinorTick','on')
    
    %LoD evolution
    figure(3)
    hold on
    %plot([1:ITER],iLoDU,'-sr','LineWidth',1,'MarkerFace','r')
    plot([1:ITER],iLoDU,'-s','LineWidth',2);
    hold on
    %plot([1:ITER],iLoDL,'-sb','LineWidth',1,'MarkerFace','b');
    plot([1:ITER],iLoDL,'-s','LineWidth',2);
    xlabel('ATC Iteration')
    ylabel('CL/CD')
    title('Converged  CL/CD');
   % text(2, iLoDU(2)+1,num2str(beta),'Color','red','FontSize',14);
   % text(2, iLoDL(2)+1,num2str(beta),'Color','blue','FontSize',14);
    grid on
    legend('System: Performance','Subsystem: Wing Aerostructural (calculation)')
    set(gca,'XMinorTick','on','YMinorTick','on')
    
    %Span area evolution
    figure(4)
    hold on
   % plot([1:ITER],iSU,'-sr','LineWidth',1,'MarkerFace','r')
    plot([1:ITER],iSU,'-s','LineWidth',2)
    hold on
    plot([1:ITER],iSL,'-s','LineWidth',2)
    xlabel('ATC Iteration')
    ylabel('Span area,sq ft')
    title('Converged  Span area, sq ft');
   % text(2, iSU(2)+100,num2str(beta),'Color','red','FontSize',14);
   % text(2, iSL(2)+100,num2str(beta),'Color','blue','FontSize',14);
    grid on
    legend('System: Performance','Subsystem: Wing Aerostructural ')
    set(gca,'XMinorTick','on','YMinorTick','on')
    
    figure(5)
    hold on
    %plot([1:ITER],iWwingU,'-sr','LineWidth',1,'MarkerFace','r')
    plot([1:ITER],iWwingU,'-s','LineWidth',2)
    hold on
    %plot([1:ITER],iWwingL,'-sb','LineWidth',1,'MarkerFace','b')
    plot([1:ITER],iWwingL,'-s','LineWidth',2)
    xlabel('ATC Iteration')
    ylabel('Wing weight, lb')
    title('Converged  Wing weight, lb');
  %  text(2, iWwingU(2)+100,num2str(beta),'Color','red','FontSize',14);
  %  text(2, iWwingL(2)+100,num2str(beta),'Color','red','FontSize',14);
    grid on
    legend('System: Performance','Subsystem: Wing Aerostructural (calculation)')
    set(gca,'XMinorTick','on','YMinorTick','on')
    
    %total function evalations
    figure(6)
    hold on
   % plot(beta,ifunccountL,'-sr','LineWidth',1,'MarkerFace','r')
     plot(beta,ifunccountL,'-s','LineWidth',2)
    hold on
   %  plot(beta,ifunccountU,'-sb','LineWidth',1,'MarkerFace','b')
    plot(beta,ifunccountU,'-s','LineWidth',2)
    xlabel('beta')
    ylabel('Total function evaluation')
    title(['Function evaluations']);
    grid on
    legend('Wing aerostructural','Performance')
    set(gca,'XMinorTick','on','YMinorTick','on')
    
    
    figure(7)
    hold on
   % plot(beta, ITER,'-sb','LineWidth',2,'MarkerFace','b')
    plot(beta, ITER,'-s','LineWidth',2)
    xlabel('beta')
    ylabel('Total number of ATC iterations')
    title(['ATC convergence']);
    grid on
    set(gca,'XMinorTick','on','YMinorTick','on')
    
        figure(8)
    hold on
  %  plot([1:ITER],fObjNew,'-sb','LineWidth',1,'MarkerFace','b')
  plot([1:ITER],irootAOA,'-s','LineWidth',2);
    xlabel('ATC Iteration')
    ylabel('Root Angle of Attack , degree')
    title(['Root Angle of Attack, degree=' num2str(irootAOA(end))]);
   % text(2, fObjNew(2)+100,num2str(beta),'Color','red','FontSize',14);
    grid on
    set(gca,'XMinorTick','on','YMinorTick','on')
    
    
    
    
    %     figure(8)
    %     hold on
    %     plot([1:ITER],iwconstr1,'-sb','LineWidth',1,'MarkerFace','b')
    %     hold on
    %     plot([1:ITER],iwconstr2,'-sr','LineWidth',1,'MarkerFace','r')
    %     plot([1:ITER],itotallift/iW0L-1.01,'-sg','LineWidth',1,'MarkerFace','g')
    %     xlabel('ATC Iteration')
    %     ylabel('constraint in wing , ft')
    %     title(['constraint violation in wing']);
    %     grid on
    %     set(gca,'XMinorTick','on','YMinorTick','on')
    
    drawnow
end %Plot condition

save dg3to1data %save for future use

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

    function convergence = checkConvergence(fObjNew, fObjOld, cU,cUold, cL,cLold, tol)
        t1 = abs(fObjNew/fObjOld -1);
        t2 = abs(cU./cUold-1);
        t3 = abs(cL./cLold-1);
        t4 = norm(t2);
        t5 = norm(t3);
        
        if t1<tol && t4<tol && t5<tol
            convergence=1; %true
        else
            convergence=0;  %false
        end
    end

end