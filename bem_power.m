function P=bem_power(rProp,Vtip,ct)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

chord=0.10*rProp;
%pitch distance in meters.
pb=pi/12;
pitch=2*pi*rProp*tan(pb);
%diameter of the propeller (meters)
dia=2*rProp;
%tip radius
R=rProp;
%engine speed in RPM
RPM=Vtip*60/(rProp*2*pi);
%thickness to chord ratio for propeller section (constant with radius)
tonc=0.12*chord;
%standard sea level atmosphere density (kg/m^3)
rho=1.225;
%RPM --> revs per sec
n=RPM/60.0;
%rps --> rads per sec
omega=n*2.0*pi;
% 2 Bladed Propeller
B=3;
% use 10 blade segments (starting at 10% R (hub) to R)
xs=0.1*R;
xt=R;
rstep=(xt-xs)/10;
r1=[xs:rstep:xt];
%calculate results for a range of velocities from 1 to 60 m/s
for V=1:1
    %initialise sums thrust (N) torque (Nm)
    thrust=0.0;
    torque=0.0;
    %loop over each blade element
    for j=1:size(r1,2)
        rad=r1(j);
        %calculate local blade element setting angle
        theta=atan(pitch/2/pi/rad);
        %calculate solidity
        sigma=2.0*chord/2.0/pi/rad;
        %guess initial values of inflow and swirl factor
        a=0.1;
        b=0.01;
        %set logical variable to control iteration
        finished=false;
        %set iteration count and check flag
        sum=1;
        itercheck=0;
        while (~finished)
            %axial velocity
            V0=V*(1+a);
            %disk plane velocity
            V2=omega*rad*(1-b);
            %flow angle
            phi=atan2(V0,V2);
            %blade angle of attack
            alpha=theta-phi;
            % lift coefficient
            cl=6.2*alpha;
            %drag coefficient
            cd=(0.008-0.003*cl+0.01*cl*cl);
            %local velocity at blade
            Vlocal=sqrt(V0*V0+V2*V2);
            %thrust grading
            DtDr=0.5*rho*Vlocal*Vlocal*B*chord*(cl*cos(phi)-cd*sin(phi));
            %torque grading
            DqDr=0.5*rho*Vlocal*Vlocal*B*chord*rad*(cd*cos(phi)+cl*sin(phi));
            %momentum check on inflow and swirl factors
            tem1=DtDr/(4.0*pi*rad*rho*V*V*(1+a));
            tem2=DqDr/(4.0*pi*rad*rad*rad*rho*V*(1+a)*omega);
            %stabilise iteration
            anew=0.5*(a+tem1);
            bnew=0.5*(b+tem2);
            %check for convergence
            if (abs(anew-a)<1.0e-5)
                if (abs(bnew-b)<1.0e-5)
                    finished=true;
                end
            end
            a=anew;
            b=bnew;
            %increment iteration count
            sum=sum+1;
            %check to see if iteration stuck
            if (sum>500)
                finished=true;
                itercheck=1;
            end
        end
        thrust=thrust+DtDr*rstep;
        torque=torque+DqDr*rstep;
    end
%     t(V)=thrust/(rho*n*n*dia*dia*dia*dia);
%     q(V)=torque/(rho*n*n*dia*dia*dia*dia*dia);
%     J(V)=V/(n*dia);
%     eff(V)=J(V)/2.0/pi*t(V)/q(V);
%     icheck(V)=itercheck;
    P=torque*n*B;
end



end

