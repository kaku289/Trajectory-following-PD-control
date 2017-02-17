Ixx = 4.000278350026442e+03;
Iyy = 4.310344827586207e+03;
Izz = 2.675413556732506e+03;

Kpphi = 74.9948; Kdphi = 0.2750;
Kptheta = 18.5600; Kdtheta = 0.1392;
Kppsi = 22.4264; Kdpsi = 0.1869;

A = [zeros(3,3), eye(3);
     -Kpphi*Ixx 0 0 -Kdphi*Ixx 0 0;
     0 -Kptheta*Iyy 0 0 -Kdtheta*Iyy 0;
     0 0 -Kppsi*Izz 0 0 -Kdpsi*Izz];
 eig(A)
 
 A = [zeros(3,3) eye(3);
      zeros(3,6)];
 B = [zeros(3,3);diag([Ixx;Iyy;Izz])];
 p = [-200;-200;-300;-400;-500;-600];
 K = place(A,B,p)