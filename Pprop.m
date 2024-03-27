function [sOUT]=Pprop(x0, xU,xLf, vU,vLf,wU,wL,misc)


disp(' ')
disp('----Prop Begins----')

% cProp=0.2;
% rProp = 2;
% RPM = 300;
lb=[0.001 0.001 2 ];
ub= [6 6 20000];
A =[];
b = [];
Aeq = [];
beq = [];


constraints=@(x)  mycon(x,xU,xLf, vU,vLf,wU,wL,misc);
func=@(x) myfun(x,xU,xLf, vU,vLf,wU,wL,misc);

options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',900,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr');% ,'PlotFcn','optimplotfvalconstr');
%Run optimization for PS
[xopt,fvalopt, FLAG, OUTPUT] = fmincon(func,x0,A,b,Aeq,beq,lb,ub,constraints,options);

[~,sOUT]=myfun(xopt,xU,xLf, vU,vLf,wU,wL,misc);
sOUT.xopt=xopt;
sOUT.funcCount = OUTPUT.funcCount;
disp('______Prop ends_________')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Objective
    function [fATC,sOUT]=myfun(x,xU,xL, vU,vL,wU,wL,misc)
        % cProp=0.2;
        % rProp = 2;
        % RPM = 300;
        cProp = x(1); rProp = x(2); RPM = x(3);
        
        omega = RPM * 2*pi/60;
        massW=misc(1);
        
        [Phover, FOM, Ct, Cp, Vtip, sigma] = hoverPowerMomentum(rProp,cProp,omega, massW*9.81);
        mprop =propMass(rProp, massW*1.7*9.81);
        MTip = 0.65;
        ToverW = 1.7;
        %get original objective
        foriginal =0 ;
        
        %Assemble c: target - response
        
        c(1) = diffc(xU(1),rProp);
        c(2) = diffc(xU(2),mprop);
        
        %Assemble v and w.Make it consistently ordered
        %get all phi
        v = vU; w=wU;
        phi= v.*c+(w.*c).^2;
        
        %ATC objective
        fATC=foriginal+ sum(phi);
        
        % constraints
        constraintleq(1) =0.75/FOM-1;
        constraintleq(2) = sigma-0.25 ;
        constraintleq(3) = Vtip*sqrt(ToverW)/340.2940  -MTip;
        constraintleq(4) = -Ct;
        constraintleq(5) = -Cp;
constraintleq(6) = 0.1/sigma-1;
        if any(~isfinite(constraintleq))
            constraintleq = 200e12;
        end        
        %Pack all data to a struct
        sOUT.cProp=cProp;
        sOUT.rProp = rProp;
        sOUT.RPM = RPM;
        sOUT.Phover = Phover;
        sOUT. FOM = FOM;
        sOUT. Ct = Ct;
        sOUT.Cp = Cp;
        sOUT.Vtip = Vtip;
        sOUT.sigma = sigma;
        sOUT.mProp = mprop;
        
        sOUT.constraints= constraintleq;
        sOUT.x = x;
        sOUT.xU = xU;
        sOUT.xL = xL;
        sOUT.vU = vU;
        sOUT.vL = vL;
        sOUT.wU = wU;
        sOUT.wL = wL;
        sOUT.misc = misc;
        sOUT.v = v;
        sOUT.w = w;
        sOUT.c=c;
        sOUT.phi=phi;
        sOUT.foriginal=foriginal;
        sOUT.fATC = fATC;
        
        save Ppropdata
    end

%Constraint
    function [cn,ceqn] = mycon(x,xU,xL, vU,vL,wU,wL,misc)
        
        load Ppropdata constraintleq
        
        cn =constraintleq;
        
        ceqn=[];
    end


    function c= diffc(aT,aR)
        c=(aT-aR)/aT ; %target - response
%                 c=abs(c);
    end


end