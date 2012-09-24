% plot maccepa schematic
function plot_maccepa_schematic3(q, u, t, model, origin, angle, ...
                                 color, boxon, ...
                                 linkon, drumline_on, ...
                                 linewidthfactor)
if nargin == 6
    color = [0 0 1];
    boxon = 0;
    linkon = 0;
    drumline_on = 0;
    linewidthfactor = 1/1.5;
elseif nargin == 7
    boxon = 0;
    linkon = 0;
    drumline_on = 0;
    linewidthfactor = 1/1.5;
elseif nargin == 8
    linkon = 0;
    drumline_on = 0;
    linewidthfactor = 1/1.5;
elseif nargin == 9
    drumline_on = 0;
    linewidthfactor = 1/1.5;
elseif nargin == 10
    linewidthfactor = 1/1.5;
end

u1 = u(1);
u2 = u(2);

if(linkon)
    L = model.link_length;
end
B = model.lever_length;
C = model.pin_displacement;
r = model.drum_radius;

drum_rel_loc = 0.05;
drum_center = [-C-drum_rel_loc; 0];

x0 = [0;0];

%%%%% tip of the link
if(linkon)
    link_pos = [L*cos(q); L*sin(q)];
    link_pos2 = -0.4*[L*cos(q); L*sin(q)];
end

%%%%% tip of the lever
lever_pos = [-B*cos(u1-q);B*sin(u1-q)];

%%%%% pin location
pin_pos = [-C; 0];

%%%%% tip of the spring
tip_spring_pos = [-C - r*u2; 0];

%%%%% draw spring 
l0 = C-B;  % rest length
h = l0/10; % spring width (for drawing)
n_dl = 30; % number of divisions for drawing 

spring_vec = (pin_pos-lever_pos);
spring_angle = atan2(spring_vec(2,1),spring_vec(1,1));
A = sqrt(B^2+C^2-2*B*C*cos(q-u1));
l = A + r*u2; % length

% center
x{2} = [(0:l/n_dl:l);zeros(1,n_dl+1)]';
% top
x{1}(:,1) = x{2}(:,1);
x{1}(:,2) = x{2}(:,2)+h;
% bottom
x{3}(:,1) = x{2}(:,1);
x{3}(:,2) = x{2}(:,2)-h;

% knot points on the spring
sp0 = 50;
sp1 = 10;
sp2 = 35;

spring_tip_ind = find(x{2}(:,1)<=A, 1, 'last');

for i=1:round(sp1*(n_dl+1)/sp0)
    if(i<=spring_tip_ind)
        z{i} = x{2}(i,:);
    end
end

for i=round(sp1*(n_dl+1)/sp0)+1:round(sp2*(n_dl+1)/sp0)
    if(i<=spring_tip_ind)    
        if(mod(i,2)==0)
            z{i} = x{1}(i,:);
        else
            z{i} = x{3}(i,:);
        end
    end
end

for i=round(sp2*(n_dl+1)/sp0):n_dl+1
    if(i<=spring_tip_ind)    
        z{i} = x{2}(i,:);
    end
end

for i=1:n_dl+1
    if(i<=spring_tip_ind)
        zz{1}(i) = z{i}(:,1);
        zz{2}(i) = z{i}(:,2);
    end
end

% rotate and shift the center for spring
Rot_spring = [cos(spring_angle) -sin(spring_angle);...
       sin(spring_angle) cos(spring_angle)];

for i=1:n_dl+1
    if(i<=spring_tip_ind)
        spring_body2{i} = Rot_spring*z{i}' + lever_pos;
        
        spring_body{1}(i)=spring_body2{i}(1,:);
        spring_body{2}(i)=spring_body2{i}(2,:);

    end
end

%%%% draw drum
tmpth=linspace(0,2*pi,100);
drum_angle_line_len = 0.05;
drum(1,:) = r*sin(tmpth)+drum_center(1);
drum(2,:) = r*cos(tmpth)+drum_center(2);

drum_tip = [-r*sin(u(2));-r*cos(u(2))] + drum_center;
drum_tip2 = drum_angle_line_len*[-sin(u(2));-cos(u(2))] + drum_center;

tmpalpha = asin(-r/drum_rel_loc);
drum_tip3 = [-r*sin(tmpalpha);-r*cos(tmpalpha)] + drum_center;

drum_tip0 = drum_angle_line_len*[-sin(0);-cos(0)] + drum_center;

%%% box
boxpos{1} = [-0.3;0.05];
boxpos{2} = [0.05;0.05];
boxpos{3} = [0.05;-0.05];
boxpos{4} = [-0.3;-0.05];

%%% shift the origin and rotate for drawing
Rot = [cos(angle) -sin(angle);...
       sin(angle) cos(angle)];

%x0
x0 = Rot*x0 + origin;

% box
for i=1:4
    boxpos{i} = Rot*boxpos{i} + origin;
end

% link 
if(linkon)
    link_pos = Rot*link_pos + origin;
    link_pos2 = Rot*link_pos2 + origin;
end

% lever
lever_pos = Rot*lever_pos + origin;
% spring
tip_spring_pos = Rot*tip_spring_pos + origin;

% pin
pin_pos = Rot*pin_pos + origin;

% drum
drum_center = Rot*drum_center + origin;
drum_tip = Rot*drum_tip + origin;
drum_tip2 = Rot*drum_tip2 + origin;
drum_tip3 = Rot*drum_tip3 + origin;
drum_tip0 = Rot*drum_tip0 + origin;

for j=1:size(drum,2)
    drum(:,j) = Rot*drum(:,j) + origin;
end

% spring body
for i=1:n_dl+1
    if(i<=spring_tip_ind)
        spring_body2{i} = Rot*spring_body2{i} + origin;
        
        spring_body{1}(i)=spring_body2{i}(1,:);
        spring_body{2}(i)=spring_body2{i}(2,:);

    end
end
%%%%%%%%%%%%% plot %%%%%%%%%%%%%%%
% box
gray = [0.4 0.4 0.4];
gray2 = [0.9 0.9 0.9];
gray3 = [0.7 0.7 0.7];

if (boxon)
    ghandle(1)=fill([boxpos{1}(1) boxpos{4}(1) boxpos{3}(1) boxpos{2}(1)]',...
                    [boxpos{1}(2) boxpos{4}(2) boxpos{3}(2) boxpos{2}(2)]',...
                    gray2, 'EdgeColor', gray);
end

% link
if (linkon)
    ghandle(2)=line([x0(1) link_pos(1)],[x0(2) link_pos(2)], ...
                    'Color',color, 'LineWidth', 2*linewidthfactor);
end
% opposite to link

if (linkon)
ghandle(3)=line([x0(1) link_pos2(1)],...
                 [x0(2) link_pos2(2)], ...
                 'Color', gray, 'LineWidth', 0.8*linewidthfactor);
end
% lever
ghandle(4)=line([x0(1) lever_pos(1)],[x0(2) lever_pos(2)], ...
     'Color', color, 'LineWidth', 3*linewidthfactor);

% spring (stretched part)
ghandle(5)=line([spring_body{1} pin_pos(1)],...
                [spring_body{2} pin_pos(2)], 'Color', color,'LineWidth', ...
                1.5*linewidthfactor);

% pivot
ghandle(7)=plot(x0(1),x0(2),'o', 'Color', color);
ghandle(8)=plot(lever_pos(1),lever_pos(2),'o', 'Color', color);
ghandle(9)=plot(pin_pos(1), pin_pos(2),'o', 'Color', color);

% drum
ghandle(12) = plot(drum(1,:), drum(2,:), 'Color', color,...
                   'Linewidth', 1.5*linewidthfactor);

if(drumline_on)
ghanlde(17) = plot([drum_center(1) drum_tip0(1)],...
                   [drum_center(2) drum_tip0(2)], ...
                   'Color',gray3,'LineWidth', 1.5*linewidthfactor);

ghanlde(13) = plot([drum_center(1) drum_tip2(1)],...
                   [drum_center(2) drum_tip2(2)], ...
                   'Color',gray,'LineWidth', 1.5*linewidthfactor);
end
ghanlde(14) = plot([drum_center(1) drum_tip(1)],...
                   [drum_center(2) drum_tip(2)], ...
                   'LineWidth', 2*linewidthfactor, ...
                   'Color', color);
ghandle(15) = plot(drum_center(1),drum_center(2),'.', 'Color', color);
ghanlde(16) = plot([pin_pos(1) drum_tip3(1)],...
                   [pin_pos(2) drum_tip3(2)], ...
                   'LineWidth', 1.5*linewidthfactor, ...
                   'Color', color);

