function eta = motor_eta(rpm)
    load('motor_eff.mat')
    t=50000./(rpm*0.10472) ;
    eta=griddata(rpm1,trq1,eff,rpm,t)/100;
    
end

