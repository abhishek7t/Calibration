function [] = plotC( Cset, Rset, scale )
    Colors = ['g', 'b', 'k' , 'm', 'c', 'r'];

    figure;
    hold on; grid on

    for i = 1 : size(Cset,2)
        % Show the ith camera
        plotCamera('Location',Cset{i}*scale,'Orientation',Rset{i},'Opacity',0, 'Color', Colors(i));
    end

    view(360,0);
    axis square;
    axis equal;
    hold off
end