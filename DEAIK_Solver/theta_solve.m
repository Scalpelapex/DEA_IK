function theta_ = theta_solve(x,y,z)
    theta_temp = 2*atan((sqrt(x.^2+y.^2))./z);
    if theta_temp >= 0
        theta_ = theta_temp;
    else
        theta_ = theta_temp + 2*pi;
    end
end