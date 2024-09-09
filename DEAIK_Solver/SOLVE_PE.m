clc;
clear;
LENGTH = 200;
index = 0.5;
s1=LENGTH*index;
s2=LENGTH*(1-index);
%%%-1-%%%
theta1_min_space = 0; theta1_max_space = 2*pi;
theta2_min_space = 0; theta2_max_space = 2*pi;
fai1_min_space = 0; fai1_max_space = 2*pi;
fai2_min_space = 0; fai2_max_space = 2*pi;
%N_ = 500000*0.4;
N_ = 100;
theta1_space = theta1_min_space + (theta1_max_space-theta1_min_space)*rand(N_,1);
theta2_space = theta2_min_space + (theta2_max_space-theta2_min_space)*rand(N_,1);
fai1_space = fai1_min_space + (fai1_max_space-fai1_min_space)*rand(N_,1);
fai2_space = fai2_min_space + (fai2_max_space-fai2_min_space)*rand(N_,1);
END_POSITION = zeros(N_,3);
% END_POSITION = [];
N_samplepoint = 0;
for n=1:1:N_
    TEMP_POSITION = End_position_solve(theta1_space(n),theta2_space(n),fai1_space(n),fai2_space(n),s1,s2);
%     %%%%Select the positive potions
%     TEMP_POSITION1 = [TEMP_POSITION(1),TEMP_POSITION(2)];
%     if all(TEMP_POSITION1 > 0)
%         END_POSITION = [END_POSITION;TEMP_POSITION];
%         N_samplepoint = N_samplepoint + 1;
%     end
    END_POSITION(n,1)=TEMP_POSITION(1);
    END_POSITION(n,2)=TEMP_POSITION(2);
    END_POSITION(n,3)=TEMP_POSITION(3);
end
% scatter3(END_POSITION(:,1),END_POSITION(:,2),END_POSITION(:,3),0.5,'blue')
scatter3(END_POSITION(:,1),END_POSITION(:,2),END_POSITION(:,3),0.5,'blue')

xlabel('X');
ylabel('Y');
zlabel('Z');
view(90,0);
% save('END_POSITION_SAVED_INDEX06.mat','END_POSITION','N_samplepoint');