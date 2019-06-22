%Dummy file for Field Variable Initialization


%function [out] = EQN(e);
function [e] = EQN(n123)
syms x c; h123 = 0;A123 = sym('c',[n123,1]);
for i = 1:n123 
    h123 = h123+A123(i,1)*x.^(i-1);
end
e = h123;