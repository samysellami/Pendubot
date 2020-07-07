function visualize_pendulum(q)
    MARKER_SIZE = 75; %marker size of the scatter function
    
    figure

    for i = 1:1:length(q)
        clf
        qi = q(:,i);
        o0 = [0,0]';
        o1 = o0 + [sin(qi);-cos(qi)];        
        hold on
        % Points of the COM
        scatter(o1(1),o1(2),MARKER_SIZE,'filled')

        line([o0(1) o1(1)],[o0(2) o1(2)],'Color','k','LineWidth',1.5)
        axis equal
        ylim([-1.2 1.2])
        xlim([-1 1])
        grid on
        pause(1e-1)
    end
end
