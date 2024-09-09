%function [x,y,z] = End_posture(t1,f1,t2,f2)
function MATRIX_XYZ = End_posture(t1,f1,t2,f2)
T1 = [cos(f1)^2*(cos(t1)-1)+1 sin(f1)*cos(f1)*(cos(t1)-1) cos(f1)*sin(t1);
      sin(f1)*cos(f1)*(cos(t1)-1) cos(f1)^2*(1-cos(t1))+cos(t1) sin(f1)*sin(t1);
      -cos(f1)*sin(t1) -sin(f1)*sin(t1) cos(t1)];
T2 = [cos(f2)^2*(cos(t2)-1)+1 sin(f2)*cos(f2)*(cos(t2)-1) cos(f2)*sin(t2);
      sin(f2)*cos(f2)*(cos(t2)-1) cos(f2)^2*(1-cos(t2))+cos(t1) sin(f2)*sin(t2);
      -cos(f2)*sin(t2) -sin(f2)*sin(t2) cos(t2)];
End_ = T1*T2*[0;0;1];
x = End_(1);
y = End_(2);
z = End_(3);
MATRIX_XYZ = [x,y,z];
end
