function [xBest,yBest] = calcXBest(x,y)
    yBest = y(1);
    xBest = x(1,:);
    for i = 1 : size(x,1);
        if(y(i) < yBest)
            yBest = y(i);
            xBest = x(i,:);
        end;
    end;
end

