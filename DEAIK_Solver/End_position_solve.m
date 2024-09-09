function matrix_end = End_position_solve(t1,f1,t2,f2,s1,s2)
T1 = [cos(f1)^2*(cos(t1)-1)+1 sin(f1)*cos(f1)*(cos(t1)-1) cos(f1)*sin(t1) s1*cos(f1)*(1-cos(t1))/t1;
      sin(f1)*cos(f1)*(cos(t1)-1) cos(f1)^2*(1-cos(t1))+cos(t1) sin(f1)*sin(t1) s1*sin(f1)*(1-cos(t1))/t1;
      -cos(f1)*sin(t1) -sin(f1)*sin(t1) cos(t1) s1*sin(t1)/t1;
      0 0 0 1];
T2 = [cos(f2)^2*(cos(t2)-1)+1 sin(f2)*cos(f2)*(cos(t2)-1) cos(f2)*sin(t2) s2*cos(f2)*(1-cos(t2))/t2;
      sin(f2)*cos(f2)*(cos(t2)-1) cos(f2)^2*(1-cos(t2))+cos(t1) sin(f2)*sin(t2) s2*sin(f2)*(1-cos(t2))/t2;
      -cos(f2)*sin(t2) -sin(f2)*sin(t2) cos(t2) s2*sin(t2)/t2;
      0 0 0 1];
End_ = T1*T2*[0;0;0;1];
x = End_(1);
y = End_(2);
z = End_(3);
matrix_end = [x,y,z];
end