function b_similar = b_preparation(s0,theta_P1)
    zm = @(thetam)(s0./thetam.*(sin(thetam)));
    rm = @(thetam1)(s0/thetam1.*(1-cos(thetam1)));
    z_p1 = zm(theta_P1);
    %z_min = zm(theta_P2);
    r_max = rm(theta_P1);
    %r_p2 = rm(theta_P2);
    %%%%%%%%%-求b-%%%%%%%
    N=200; %把0-thetaP1分成N等分。
    theta_i = theta_P1/N:theta_P1/N:theta_P1;
    %theta_j = theta_P1:(theta_P2-theta_P1)/N:theta_P2;
    syms b;
    %Ra_error=zeros(N,1);
    Rb_error=cell(N,1);
    for ii = 1:1:N
        zm_i = zm(theta_i(ii));
        rm_i = rm(theta_i(ii));
        r_thetai = r_max*sqrt(1-(zm_i-z_p1).^2/((s0-z_p1-b).*(zm_i-z_p1)+(s0-z_p1).*b));
        error_b = r_thetai-rm_i;
        %error_b
        Rb_error{ii,1} = error_b;
        %Rb_error{ii,1}
    end
    temp_cell_sum = 0;
    for ii_for = 1:N
        temp_cell_sum = temp_cell_sum + Rb_error{ii_for,1}; 
    end
    b_similar = vpasolve(temp_cell_sum==0,b);%最小二乘法求b的近似值
end
    %%%%%%%%%%-至此，求出了H1和H2两个函数的近似(即：准备阶段结束)-%%%%%%%%%%%%
    
    
    
    
    
        
        
    
    