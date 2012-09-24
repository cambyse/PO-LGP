% animation for a simple pendulum
function twolink_anim_maccepa(Tout, Qout, model, xt, dt_anim, ...
                                          flipped_model)
close all;

if (nargin < 4 || isempty(xt))
    draw_target = 0;
else
    draw_target = 1;
end

x = zeros(2,2); % tip position of the link

if(nargin<5)
    dt_anim = 0.01;
end

if(nargin<6)
    flipped_model = 0;
end

blue = [0 0 1];
dodger_blue = [30 144 255]/255;
gray = [0.5 0.5 0.5];

T = Tout(1):dt_anim:Tout(length(Tout));
Q = interp1(Tout(1:end), Qout(1:end,:), T);

% start animation
for j=1:1:length(T);

    clf;
    hold on;
    
    th=Q(j,1:2);
    thm=Q(j,5:6);
    t=T(j);
    
    x = twolink_calcpos(t, th, model);

    % robot arm
    axis_x = model.l(1)+model.l(2);
    axis_y = axis_x/5;

    plot([-5 5],[ 0 0], 'Linewidth', 0.5, 'Color','r');
    plot([ 0 0],[-5 5], 'Linewidth', 0.5, ...
         'Color','r');    

    plot(0,0,'ko');
    plot(x(1,1),x(2,1),'ko','MarkerSize', 8); 
    plot(x(1,2),x(2,2),'ko');
    plot([0 x(1,1) x(1,2)],[0 x(2,1) x(2,2)], ...
              'Linewidth', 1.5, 'Color', 'Black');
    
    % maccepa
    if(~flipped_model)
        plot_maccepa_schematic3 ...
            (th(2),thm',t,model,x(1:2,1),th(1)-pi/2, blue,0,0,1);
    else % fipped model
        plot_maccepa_schematic3 ...
            (-th(2),[-thm(1);thm(2)],t,model,x(1:2,1),...
             (th(1)+th(2)-pi/2-pi), blue,0,0,1);        
    end

    % time
    text(axis_x*0.7,axis_y*0.5 ,sprintf('t=%.2f',t));      
    
    %%% target
    if (draw_target)
        plot(xt(1), xt(2), 'k+', 'MarkerSize', 12);
    end

    axis equal;
    axis([-axis_x axis_x -axis_x axis_y]);
    
    box on;
    grid on;
    drawnow;

    if (j==1)
        disp('press any key to start animation');
        pause;
    end
    
end

hold off;



