%1D_Point_Collocation
%function [f] = DEF(n,n1,rest,A1,B,C,n3,L,ty,H2,A5,A6,A7,point) %()
%while (conv1-conv2)== 10e-4 
    %Point collocation METHOD
syms d x y z op o eq;
n = input('No of parameters = '); e = EQN(n); disp('Parametric eqn ='); disp(e);
n1  = input('Input [No_of_DE ] = ');
rest = input('Remaining terms of function = ');B= sym('o',[n1,1]);C1=0;C = sym('eq',[n1,1]); A = sym('d',[n1,1]);A1 = sym('op',[n1,1]);
for i = 1:n1
   fprintf('Operator of %d st DE\n',i);A1(i,1)= input('=');fprintf('order of %d st DE\n',i);B(i,1)= input('=');fprintf('Coeeficients of %d st DE\n',i);C_C(i,1)=input('=');C(i,1) = DE2([e A1(i,1) B(i,1) C_C(i,1)]);C1 = C1+C(i,1);
end
disp(C1)
%no of points where residue is zero
n3 = input('No of boundary condition = ');n2 = n-n3;L = input('Length = ');t = linspace(0,L,n2+1);
ty = zeros(n,1);%Input Boundary condtn
for i=1:n3
    fprintf('Type of %d st Boundary condition[ex u(0) = 1, diff(u(0))= 2]\n',i);
ty(i,1) = input('');
end 
j=0;
for i=1:n3
    if ty(i,1) == 1
         fprintf('%d st Boundary condition of type = 1\n',i);
        H1(i,1) = input('[x] =');
        H1(i+n3,1) = input('[y] =');
        if i>1
            i=1;
        j = j+i;
        else
         j = j+i;   
        end
    else
        fprintf('%d st Boundary condition of type = 2\n',i);
        H2(i,1) = input('[x] =');H2(i+n3,1) = input('[y] =');A5(i,1) = input('[operator] = ');
        A6(i,1) = input('[order ] = '); A7(i,1) = input('[coefficient] = ');
      end
end
k = n3-j;syms m;M1 = sym('m',[n2,1]); %Substituting value.
for i=1:n2
    M2(i,1) = subs(rest,x,t(1,i+1));M1(i,1)= subs(C1,x,t(1,i+1));   
end
if j == k
for i0 = 1:j
    if mod(i0,2) == 1
     B1(i0,1) = subs(e,x,H1(i0,1))-H1(i0+n3,1);
    else
        i0 = i0+1;
        B1(i0-1,1) = subs(e,x,H1(i0,1))-H1(i0+n3,1); 
    end
end
for i0 = 1:k
    if mod(i0,2) == 1
            B2(i0,1) = subs(DE2([e A5(i0+1,1) A6(i0+1,1) A7(i0+1,1)]),x,H2(i0+1,1))-H2(i0+1+n3,1);
      else
        i0 = i0+1;
            B2(i0-1,1) = subs(DE2([e A5(i0+1,1) A6(i0+1,1) A7(i0+1,1)]),x,H2(i0+1,1))-H2(i0+1+n3,1);
  end
end
else
    if j>k
    for i0 = 1:j
     B1(i0,1) = subs(e,x,H1(i0,1))-H1(i0+n3,1);B2 = 0;
    end
        else
        for i0 = 1:k
            B2(i0,1) = subs(DE2([e A5(i0+1,1) A6(i0+1,1) A7(i0+1,1)]),x,H2(i0,1))-H2(i0+n3,1);B1 = 0;
        end
    end
end
 for i=1:n2
     B3(i,1) = M1(i,1)+M2(i,1);
 end
 if B1 == 0
 [U1,V1] = equationsToMatrix([B2; B3]);
 else
     [U1,V1] = equationsToMatrix([B1; B2; B3]);
 end
 if B2 == 0
 [U1,V1] = equationsToMatrix([B1; B3]);
 else
     [U1,V1] = equationsToMatrix([B1; B2; B3]);
 end
 if det(U1)== 0
     disp('Matrix of coeffs = ');disp(U1);disp('Value of constants ='); disp(V1);disp('det(Matrix) = 0');
 else
 Z = inv(U1)*V1;disp('Matrix of coeffs = ');disp(U1);disp('Value of constants =');disp(V1);syms f(x); f(x)=0;   h=0;   %eqn formation for final field variable   %Initialize f(x)=0   %Formation of Field Variable
for i=1:n
  h = Z(i,1)*x^(i-1);f(x) = f(x)+h;
end
fprintf('Field Variable = ');disp(f(x));point=input('soln at how many points = ');t=linspace(0,L,point+2);C=f(t);
for i=1:point+2
    fprintf('field variable at {%f} = %f \n',t(1,i),C(1,i))
end
plot(t,C,'b');title('Point Collocation Method');xlabel('Points');ylabel('Field Variable');
 end
