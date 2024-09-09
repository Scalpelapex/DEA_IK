function MatrixP=PositionEnd(T1,T2,F1,F2)
%正运动学求解
%   返回工作空间中三个坐标值
    S1 = 100;
    S2 = 100;
    P1 = cos(T1)*sin(F1)*(S2/T2)*(1-cos(T2))*cos(F2)-sin(F1)*(S2/T2)*(1-cos(T2))*sin(F2)+sin(T1)*cos(F2)*(S2/T2)*sin(T2)+(S1/T1)*(1-cos(T1))*cos(F1);
    P2 = cos(T1)*sin(F1)*(S2/T2)*(1-cos(T2))*cos(F2)+cos(F1)*(S2/T2)*(1-cos(T2))*sin(F2)+sin(T1)*sin(F1)*(S2/T2)*sin(T2)+(S1/T1)*(1-cos(T1))*sin(F1);
    P3 = -sin(F1)*(S2/T2)*(1-cos(T2))*cos(F2)+cos(T1)*(S2/T2)*sin(T2)+(S1/T1)*sin(T1);
    MatrixP = [P1 , P2 , P3];
end
