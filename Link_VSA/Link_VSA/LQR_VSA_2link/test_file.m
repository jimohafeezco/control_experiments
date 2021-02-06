syms m1 m2 l1 l2 x1 x2 dx1 dx2 g


beta0= -m2*l1*0.5*l2*x2*(dx2)^2-2*m2*l1*l2*0.5*x2*dx2*dx1;
beta1=-m2*l1*0.5*l2*x2*(dx1)^2;

beta= [beta0; beta1];
dbeta_dx= jacobian(beta,[x1, x2, dx1, dx2])