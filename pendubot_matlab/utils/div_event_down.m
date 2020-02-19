function [position,isterminal,direction] = div_event_down(t,x)
    q = x(1:2);

    if (abs(q(2)-pi) >= pi/4 )
        k = 0;
    else
        k = 1;
    end
    
    position = k; % The value that we want to be zero
    isterminal = 1;  % Halt integration 
    direction = 0;
end