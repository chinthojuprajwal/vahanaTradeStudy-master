function [m_gb] = mass_gb(rpm,rProp,power)
tip_mach=0.65;
omega=tip_mach*340.294/rProp;
rpm_rotor=omega*60/(2*pi);
% if rpm/5000<=1
%     gr=5000/rpm;
% else
%     gr=rpm/5000;
% end
% 
% m_gb=(100/6)*(gr);
hp=power*1.34102/1000;
k=94;
n_rotor=8;
m_gb=8*k*0.453592*((hp/8)^0.76)*(rpm^0.13)/(rpm_rotor^0.89);

end

