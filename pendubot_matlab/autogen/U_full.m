function out1 = U_full(s,s_d,y,y_d,v)
%U_FULL
%    OUT1 = U_FULL(S,S_D,Y,Y_D,V)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    03-Mar-2020 18:02:04

t2 = s.*2.0;
t3 = sin(s);
t4 = y_d.^2;
t5 = cos(s);
out1 = -(t5.^2.*8.772719429425975e-3-2.6256165e-2).*(v+((sin(s+y).*-3.97141173e8+sin(t2+y).*1.15095825e41-sin(y).*6.173466316269231e41+t3.*t4.*4.129175e39+s_d.^2.*t3.*4.129175e39+t4.*sin(t2).*2.933125e39+s_d.*t3.*y_d.*8.25835e39).*1.0)./(cos(t2).*2.933125e39-1.4624170565e40));
