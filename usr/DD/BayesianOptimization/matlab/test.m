   [circXY(:,1),circXY(:,2)] = pol2cart(linspace(0,2*pi,50)', 1);
    sqXY = [-1 -1;1 -1;1 1;-1 1; -1 -1];
    C = {[sqXY*5 ones(5,1)]           % Start with a small square
        [circXY*40 ones(50,1)*30]     % Blend to a large circle
        [sqXY*20 ones(5,1)*65]        % Blend to a large square
        [circXY*10 ones(50,1)*99]};   % Blend to a small circle
    X = linspace(-40, 40, 200);
    Y = linspace(-40, 40, 200);
    Z = linspace(0, 100, 400);
    BW = blendedPolymask(C,X,Y,Z);
    figure, patch(isosurface(X,Y,Z,BW,0.5),'FaceColor','g','EdgeColor','none','FaceAlpha',0.5)
    view(3), camlight, hold on, axis image