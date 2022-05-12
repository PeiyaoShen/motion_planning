syms Q1 Q2 Q3 P1 P2 P3
P = [P1;
     P2;
     P3];
Q = diag([Q1 Q2 Q3]);

res = P' * Q * P;