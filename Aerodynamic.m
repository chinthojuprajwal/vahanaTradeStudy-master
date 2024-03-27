function [ CD,CL, CDi, CD0, Cl, cd0,clmax,yc]  = Aerodynamic(span,croot,rootaoa)
%find sectional cl,cd vs AOA(degree) from csv file
xfoildata = readmatrix('N0012_Sample.csv', 'HeaderLines',2);
xfoil.AOA = xfoildata(:,1); xfoil.CL = xfoildata(:,2); xfoil.CD = xfoildata(:,3); xfoil.CM = xfoildata(:,4); 
[xfoil.sectional_Clmax, xfoil.sectional_Clmax_index]  = max(xfoil.CL);
xfoil.sectional_Clmax_AOA = xfoil.AOA(xfoil.sectional_Clmax_index); 
   
N=10;    %no of panels on left side
lambda = 1; %taper ratio        
%span = 10;
%croot = 1.5;  %chord at root

xopt=rootaoa*ones(1,N); %   % geometric AOA in degree

% Find parameters at optimum design point
[alpharad,NS] = expandvector(xopt*pi/180,N);
[ CD,CL, CDi, CD0, Cl, cd0,clmax,yc] = NLLT(lambda, alpharad,xfoil,NS,span,croot); 
 
end

    
    
function [CD,CL, CDi, CD0, Cl, cd0,clmax,yc] = NLLT(lambda, alpha_geometric, xfoil,N,b,croot)
    %disp(['NLLT running...'])
 %   N = 150;                        %Number of panels
 %   b = 10;                         %Wing span
    yv = linspace(-b/2,b/2,N+1);    %y position of vortex shedding
%     xtheta= linspace(0.00001*pi,0.99999*pi,N+1);
%     yv = -b/2*cos(xtheta);
    sp = diff(yv);                  %Spacing of panels
    yc = sp/2+yv(1:end-1);          %y position of control points
    c=zeros(1,numel(yc));
    c(yc>=0) = croot*(1-(1-lambda)/(b/2)*yc(yc>=0));              %Chord distribution 
    c(yc<0) = croot*(1-(1-lambda)/(b/2)*(-yc(yc<0)));
    
    Sref = sum(c.*sp);              %Reference area
    alpha = alpha_geometric;          %Angle of attack distribution . This is in radian
    Cla = 2*pi;                     %Airfoil lift-curve slope linear
    AR = b^2/Sref;                  %Aspect ratio
    Gamma_res = 1;                  %Residual of Gamma iteration
    Gamma_tol = 1e-8;               %Convergence tolerance
    relax = 0.01;                    %Relaxation factor

    A = zeros(N,N);                 %Aerodynamic Influence Coefficient matrix
    bvec = zeros(N,1);              %RHS of linear system of equations

    %Establish AIC matrix and RHS vector for initial gamma distribution
    for i=1:N
        for j=1:N
            A(i,j)=(1/(4*pi)*(1/(yc(i)-yv(j))))-(1/(4*pi)*(1/(yc(i)-yv(j+1))));
        end
        A(i,i)=A(i,i) + 1/(pi*c(i));
        bvec(i) = alpha(i);
    end

    %Solve for Gamma/Vinf
    Gamma=(A\bvec)';

    %Calculate CL and Cl distribution
    CL = sum(2*Gamma.*sp)/Sref;
    Cl_theory = 2*Gamma./c;

    %Establish initial solution for NLLT
    Gamma_last = Gamma;
    Gamma_new = Gamma;
    
    %Set number of iterations performed to zero
    iter = 0;

    %Iterate until converged on Gamma/Vinf distribution
    while any(Gamma_res > Gamma_tol)
        
 
        %Initialize induced AOA

       
        ai = zeros(1,N);
        for i=1:N
            for j=1:N
                %Calculate induced AOA
                ai(i) = ai(i)+(Gamma_last(j)/(4*pi)*(1/(yc(i)-yv(j))))-(Gamma_last(j)/(4*pi)*(1/(yc(i)-yv(j+1))));
            end
        end

        %Determine effective AOA of airfoil sections
        aeff = alpha-ai;    %in radian   
        
        %Only calculate xfoil values on left side due to symmetry
        [aeffShrink,Ns] =shrinkvector(aeff,N);
        %Determine sectional lift coefficient (modify here to run xfoil!)
        ClShrink=interp1(xfoil.AOA, xfoil.CL, aeffShrink*180/pi,'linear'); %with aeff in degree
        
        %Expand Cl distribition
        Cl=expandvector(ClShrink,Ns);
        
        
        %Determine new Gamma/Vinf distribution
        Gamma_new = 1/2*c.*Cl;
        Gamma_new = smoothdata(Gamma_new,'gaussian',N/100);
        %Determine residual in Gamma between iterations, set new Gamma for next
        %iteration
        Gamma_res = abs((Gamma_new - Gamma_last)./Gamma_new);
        Gamma_last = Gamma_last + relax*(Gamma_new-Gamma_last);

        %Increment iteration number
        iter = iter + 1;
    end

    CL = sum(2*Gamma_new.*sp)/Sref;
    CDi = sum(2*Gamma_new.*ai.*sp)/Sref;
    cd0 = interp1(xfoil.AOA, xfoil.CD, aeff*180/pi,'spline');

    CD0 = sum(cd0.*c.*sp)/Sref;
    CD = CD0+CDi;
    
    
    clmax=max(Cl);
    
end

function [ve,NS] = expandvector(x,N)
    %create symmetry and expand vector
    NS=2*N;
    vL = x; % Left side
    vR = flip(vL);   %Right side
    ve = [vR vL];   %expand to symmetric
end

function [x,N] = shrinkvector(ve,NS)
    %create symmetry and shrink  vector
    N=NS/2;
    xR = ve(1:N); % Right side
    xL = ve(N+1:end);
    x = xL;   %left half only
end

