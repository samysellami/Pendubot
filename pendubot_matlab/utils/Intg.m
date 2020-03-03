function I = Intg(s,s_d,s_0,s_d0)
%for L = -1.5, g=9.81
% alfa = cos(s - (3*sin(s))/2) - (3*cos(s))/2 + 1;
% beta = (3*sin(s))/2;
% gamma = -(981*sin(s - (3*sin(s))/2))/100;



if s == s_0
    I = s_d^2 - s_d0^2;
    return
end

n=100;
h=(s-s_0)/n; xi=s_0:h:s;
f = zeros(n+1,1);
for i=1:n+1
    f(i) = intg_fi(s_0,xi(i));
end
% f = @(s) intg_f(s,s_0);
int1 = simpsons(f,s_0,s,100);


% See 24-25 in "Constructive tool..." 2005
I = s_d^2 - intg_psi(s_0,s)*(s_d0^2 - int1);

end

