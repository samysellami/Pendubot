function I = Intg(s,s_d,s_0,s_d0)
%for L = -1.5, g=9.81
% alfa = cos(s - (3*sin(s))/2) - (3*cos(s))/2 + 1;
% beta = (3*sin(s))/2;
% gamma = -(981*sin(s - (3*sin(s))/2))/100;

if s == s_0
    I = s_d^2 - s_d0^2;
    return
end

function ret = f(s)
    ret = intg_fi(s_0,s);
end

% g = @(x) intg_fi(s_0, x);
% g =  @(s) s.^2 + s.^3;

int1 = simpsons(@f,s_0,s,1000); 

% See 24-25 in "Constructive tool..." 2005
I = s_d^2 - intg_psi(s_0,s)*(s_d0^2 - int1);

end

