function [qw] = wrapToPi2(q)
%WRAPTOPI2 Summary of this function goes here
%   Detailed explanation goes here
qw = wrapToPi(q);
if qw == -pi
    qw = pi;
end
end

