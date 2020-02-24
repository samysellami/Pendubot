function psi = intg_psi(s0,s)
%psi as in paper: Constructive tool...2005
%for L = -1.5, g=9.81

if s0 == s
    psi = 1;
    return
end

f1=@(s) ( beta_fcn(s) ./ alpha_fcn(s));
int1 = simpsons(f1,s0,s,1000);
psi = exp(-2*int1);

end
