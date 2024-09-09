clc;
clear;
tic;
LENGTH = 200;
index = 0.6;
s1=LENGTH*index;
s2=LENGTH*(1-index);
%SOLVE_PE - You can make your own points bt SOLVE_PE.m
load Case_EndPointsOfContinuum.mat;
D_P = zeros(N_samplepoint,1);
% -Preparation- %
%%% -The frist segment- %%%
x1 = @(theta11,fai11)(s1/theta11*(1-cos(theta11))*cos(fai11));
y1 = @(theta11,fai11)(s1/(1-cos(theta11))*sin(fai11));
z1 = @(theta11)(s1./theta11.*(sin(theta11)));
r1 = @(theta11)s1/theta11.*(1-cos(theta11));
theta1_P1 = fzero(@(theta001)(theta001*sin(theta001)+cos(theta001)-1),[0.1,pi]);
theta1_P2 = fzero(@(theta011)(theta011*cos(theta011)-sin(theta011)),[pi,2*pi]);
z1_p1 = z1(theta1_P1);
z1_min = z1(theta1_P2);
r1_max = r1(theta1_P1);
r1_p2 = r1(theta1_P2);
%%% -The second segment- %%%
x2 = @(theta12,fai12)(s2./theta12.*(1-cos(theta12)).*cos(fai12));
y2 = @(theta12,fai12)(s2./(1-cos(theta12)).*sin(fai12));
z2 = @(theta12)(s2./theta12.*(sin(theta12)));
r2 = @(theta12)s2/theta12.*(1-cos(theta12));
theta2_P1 = fzero(@(theta002)(theta002*sin(theta002)+cos(theta002)-1),[0.1,pi]);
theta2_P2 = fzero(@(theta012)(theta012*cos(theta012)-sin(theta012)),[pi,2*pi]);
z2_p1 = z2(theta2_P1);
z2_min = z2(theta2_P2);
r2_max = r2(theta2_P1);
r2_p2 = r2(theta2_P2);
B2_solve = @(r2_tt,z2_tt)((r2_max.^2.*(z2_tt-z2_p1).^2)./((r2_max.^2-r2_tt.^2).*(s2-z2_tt))-((s2-z2_p1).*(z2_tt-z2_p1))./(s2-z2_tt));    
%%%%%% -init b2- %%%%%% 
b2_init = b_preparation(s2,theta2_P1);
% -END PREPARATION- %
syms x;%x-xm;y-ym;z-zm;
Q=0.*x;
y = 0.*x;
% toc;
for i_count0 = 1:1300
    N=200; % Divide 0-theta_P1 into 5 parts
    End_pose_vector = zeros(N,3);%Posture of manipulator end
        % -Obtain Endpoint Pe- %
        xe = END_POSITION(i_count0,1);
        ye = END_POSITION(i_count0,2);
        ze = END_POSITION(i_count0,3);
        % -START- %
        %%% -divide index- %%%
        theta = theta1_P2/N:theta1_P2/N:theta1_P2;
        theta(N) = theta1_P2-0.001;
        %fai = 0:2*pi/M:2*pi;        
        %%% -1- -cicle theta1- %%%
%         toc;
        for ii = 1:N
            % -cicle assign theta1- %
            theta1=theta(ii);% Ergodic
            %%% - method1 - Solve the 1st segment of continuum - %%%
            z1_t = z1(theta1); % Third point
            r1_t = r1(theta1);
            if theta1 <= theta1_P1
                a1=+inf;
                b1 = (r1_max.^2.*(z1_t-z1_p1).^2)./((r1_max.^2-r1_t.^2).*(s1-z1_t))-((s1-z1_p1).*(z1_t-z1_p1))./(s1-z1_t);
                judge_ = 1;
            elseif theta1 > theta1_P1 && theta1 <= theta1_P2
                a1 = (z1_t-z1_p1)^2.*(r1_max-r1_p2)^2./(((r1_max-r1_p2)^2-(r1_t-r1_p2)^2).*(z1_t+abs(z1_min)))+(z1_p1+abs(z1_min)).*(z1_t-z1_p1)./(z1_t+abs(z1_min));
                b1=+inf;
                judge_ = 2;
            end
            %%% - Q(x) iteration- %%%
            z = z1(theta1);
            %%% - Establish Q（x）to eliminate y -%%%
            aa1=s1-z1_p1;
            bb1=b1;
            %b1
            cc1=r1_max;
            zz1=z1_p1;
            rr1=0;
            %a2
            aa2=a1;
            bb2=z1_p1+abs(z1_min);
            cc2=r1_max-r1_p2;
            zz2=z1_p1;
            rr2=r1_p2;
            C_1=(sqrt(cc1^2-(cc1^2*(z-zz1)^2)/((aa1-bb1)*(z-zz1)+aa1*bb1))+rr1)^2;
            C_2=(sqrt(cc2^2-(cc2^2*(z-zz2)^2)/((aa2-bb2)*(z-zz2)+aa2*bb2))+rr2)^2;
            C_ = [C_1;C_2];

            %%%%%%%%%%%%%%%% - Quadrant determination - %%%%%%%%%%%%%%%%%
            y(x)=sqrt(-x^2+C_(judge_));
            %%%-参数-%%%
            R=sqrt(z^2+C_(judge_));
            m2=-2*xe/R^2;
            m1=(z*(z-2*ze)+C_(judge_))/R^2;
            m0=xe;
            mm1=(-2*ye)/R^2;
            n2=2*ye/R^2;
            n0=((z^2-C_(judge_))/R^2)*ye;
            nn1=-2*xe/R^2;
            nn0=m1;
            l1=2*xe*z/R^2;
            l0=(z^2*(ze-z)-C_(judge_)*(ze+z))/R^2;
            ll0=2*ye*z/R^2;
            %%%% - Function: XEM(x)\YEM(x)\ZEM(x) are: use xm to solve xem\yem\zem-%%%%
%             XEM(x) = m2*x^2+m1*x+m0+mm1*x*y;
%             YEM(x) = n2*x^2+n0+(nn1*x+nn0)*y;
%             ZEM(x) = l1*x+l0+ll0*y;
            %%%%%%%%
            q2=m1^2+C_(judge_)*mm1^2+2*m2*m0+C_(judge_)*nn1^2-nn0^2+2*n2*n0;
            q1=2*m1*m0+2*C_(judge_)*nn1*nn0;
            q0=m0^2+n0^2+C_(judge_)*nn0^2;
            p1=2*m0*mm1+2*n0*nn1;
            p0=2*n0*nn0;
            %%%%%%%%
            %ZZ(x)=l1*x+l0+ll0*y(x);
            %f(x)=ZZ(x)-z2_p1;
            %g(x)=q2*x^2+q1*x+q0+(p1*x+p0)*y(x);
            %%%% - Iteration - %%%
            EPS2 = 0.1;  % Control the precision
            aa2_1 = s2-z2_p1;
            D1=aa2_1-b2_init;
            D2=aa2_1.*b2_init;
            D3=1./(r2_max^2);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            K3 = D3*D1*(l1*q2-ll0*p1);
            K2 = D3*D1*(l1*q1+(l0-zz1)*q2-ll0*p0)+D3*D2*q2+l1^2-ll0^2;
            K1 = D3*D1*(l1*q0+(l0-zz1)*q1+C_(judge_)*ll0*p1)+D3*D2*q1-D1*l1+2*l1*(l0-zz1);
            K0 = D3*D1*((l0-zz1)*q0+C_(judge_)*ll0*p0)+D3*D2*q0-(D1*(l0-zz1)+D2)+ll0^2*C_(judge_)+(l0-zz1)^2;
            H2 = D3*D1*(l1*p1+ll0*q2);
            H1 = D3*D1*(l1*p0+(l0-zz1)*p1+ll0*q1)+D3*D2*p1+2*l1*ll0;
            H0 = D3*D1*((l0-zz1)*p0+ll0*q0)+D3*D2*p0-D1*ll0+2*ll0*(l0-zz1);
%             toc;
            Q(x) = K3*x^3+K2*x^2+K1*x+K0+(H2*x^2+H1*x+H0)*y;
%             toc;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Q(x)=f^2/(D1*f+D2)+D3*g-1;
            %Q(x) = f^2+D3*D1*f*g+D3*D2*g-(D1*f+D2);
            %Q(X) = K3*x^3+K2*x^2+K1*x+K0+(H2*x^2+H1*x+H0)*y;
            theta_m_0 = theta1_P1.*rand();
            fai_m_0 = 2*pi*rand();
            %theta_m_0 = theta1_P1*0.2;
            %fai_m_0 = 2*pi*0.2;
            xm_0 = x1(theta_m_0,fai_m_0);
            %%%% -!!!!-!!!!-!!!!- %%%%
            N_End_xm = newtoniteration1(Q,xm_0,0.01);%%%%
            %%%% - Whether there is a solution ? - %%%%
            if all(N_End_xm == 'wujie')||all(N_End_xm == 'fasan')
                continue;
            end
            xem_new = m2*N_End_xm^2+m1*N_End_xm+m0+mm1*N_End_xm*y(N_End_xm);
            yem_new = n2*N_End_xm^2+n0+(nn1*N_End_xm+nn0)*y(N_End_xm);
            zem_new = l1*N_End_xm+l0+ll0*y(N_End_xm);
            r2_tt_new = sqrt(xem_new^2+yem_new^2);
            b2_last = b2_init;
            b2_new = B2_solve(r2_tt_new,zem_new);
            %b2_new = (r2_max.^2.*(zem_new-z2_p1).^2)./((r2_max.^2-r2_tt_new.^2).*(s2-zem_new))-((s2-z2_p1).*(zem_new-z2_p1))./(s2-zem_new);
            d_B = norm(b2_new-b2_last);
            k=1;m=1;
%             toc;
            while d_B>=EPS2
                % Update D1，D2 of Q(x)
                D1=aa2_1-b2_new;
                D2=aa2_1.*b2_new;
                m=m+1; % The count of Iteration of b2
                % Q(x)=f^2/(D1*f+D2)+D3*g-1;
                % Newton Iteration
                xm_0 = N_End_xm;
                N_End_xm = newtoniteration1(Q,xm_0,0.01);
%                 xem_new = XEM(N_End_xm);
%                 yem_new = YEM(N_End_xm);
%                 zem_new = ZEM(N_End_xm);
                xem_new = m2*N_End_xm^2+m1*N_End_xm+m0+mm1*N_End_xm*y(N_End_xm);
                yem_new = n2*N_End_xm^2+n0+(nn1*N_End_xm+nn0)*y(N_End_xm);
                zem_new = l1*N_End_xm+l0+ll0*y(N_End_xm);
                r2_tt_new = sqrt(xem_new^2+yem_new^2);
                b2_last = b2_new;
                b2_new = B2_solve(r2_tt_new,zem_new);
                % the result of b2
                d_B = norm(b2_new-b2_last);
                if m>1000   % Iteration Cutoff to prevent non-convergence
                    N_END_xm = 0;
                    break;
                end
            end
%             toc;
            m=0;%clear
            xm = N_End_xm;
            theta1_consult = theta1;
            fai1_consult = acos(xm/r1(theta1));   
            theta2_consult = 2*atan(sqrt(xem_new^2+yem_new^2)/zem_new);
            fai2_consult = atan(yem_new/xem_new);
            End_pose_vector(ii,:) = real(End_posture(theta1_consult,fai1_consult,theta2_consult,fai2_consult));
    end    
    % -Dexterity analysis- %
    %%% - Given a set <theta,fat> corresponding to (xm，ym，zm) - %%%%
    toc;
    Mt=30; % Divide
    Mf=60; % Divide
    t_H=linspace(-1,1,Mt);
    t=asin(t_H./1);
    p=linspace(-pi,pi,Mf); 
    [elevation,azimuth]=meshgrid(t,p);
    xxx=cos(azimuth).*cos(elevation);
    yyy=sin(azimuth).*cos(elevation);
    zzz=sin(elevation);
    %surf(xxx,yyy,zzz,'FaceAlpha','0');% Draw the dexterity ball
    %%% - Return the posture of the end - %%%
    Aq_count = 0;% The number of the faces that are drawed(red)
    [azimuth_fai,elevation_theta,rho] = cart2sph(End_pose_vector(:,1),End_pose_vector(:,2),End_pose_vector(:,3));
    which_segment = zeros(N,2);
    %%% - Calculate the dexterity of the i-st point - %%%
    for ii0 = 1:1:N
    %%% Find the position which is needed to draw；
        num_t = length(t(t<elevation_theta(ii0)));
        num_f = length(p(p<azimuth_fai(ii0)));
        which_qujian = [num_t num_f];
        if ~ismember(which_qujian,which_segment,'rows')
            Aq_count =  Aq_count + 1; % Draw red
        end
        %which_segment((i_count_patch-1)*N+ii0,:) = which_qujian;
        which_segment(ii0,:) = which_qujian;
        %d_theta = t(num_t+1)-t(num_t);
        %d_fai = p(2)-p(1);
        %theta_test=[t(num_t) t(num_t) t(num_t)+d_theta t(num_t)+d_theta];
        %fai_test = [p(num_f) p(num_f)+d_fai p(num_f)+d_fai p(num_f)];
        %x_ = cos(theta_test).*cos(fai_test);
        %y_ = cos(theta_test).*sin(fai_test);
        %z_ = sin(theta_test);
            %patch(x_,-y_,z_,'red');
        %patch(x_,y_,z_,'red');%%% Visulization
    end
    As_all = Mt.*Mf;
    %%% - The dexterity index of the points - %%%     
    D_P(i_count0) = Aq_count./As_all;
    save('index06_DP_1_1300.mat','D_P');
end
%D_t = sum(D_P); % The sum
toc;
% save('index05_DP_3301_4000.mat','D_P','End_pose_vector');