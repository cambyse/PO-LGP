clear;
myformat = '%s\n';


P1 = [1.1 -1.8 2.6]; % border point 1
P2 = [1.1 -1 0.8]; % border point 2
P3 = [0.364831 -2.54747 1.83117]; % robot start point

numY = 10;
numZ = 15;

j = 1;

for iterY=1:numY
  for iterZ=1:numZ
    % copy basefile which is equivilent for all scenes
    filename = ['../scenes/mobstacle/scene',num2str(j)];
    copyfile('robot.ors',filename);
    fid = fopen(filename,'a');
    
    % append line for goalRef
    goalRef = ['body goalRef  { type=1 size=[0 0 0 .03] X=<T t(',num2str(mean([P1;P2]),'%g '),')> color=[1 0 0] fixed }'];
    fprintf(fid, myformat, goalRef);
    
    % append line for goal
    goalPos = P1+(P2-P1).*[0,(iterY-1)/(numY-1),(iterZ-1)/(numZ-1)];
    goal = [ 'body goal  { type=1 size=[0 0 0 .005] X=<T t(',num2str(goalPos,'%g '),')> color=[0 1 0] fixed }'];
    fprintf(fid, myformat, goal);
    
    % append line for obstacle
    obsPos = P3-(P3-goalPos)*0.5;
    obs = ['body obstacle { type=1 size=[0 0 0 0.1] X=<T t(',num2str(obsPos,'%g '),')> color=[1 0 0] contact, fixed  }'];
    fprintf(fid, myformat, obs);
    
    direction = randn(3,1)';
    vel = 2;
    goaldir = [ 'body dir  { type=1 size=[0 0 0 .005] X=<T t(',num2str(vel*direction,'%g '),')> color=[0 1 0] fixed }'];
    fprintf(fid, myformat, goaldir);
    
    direction = randn(3,1)';
    vel = 2;
    obsdir = [ 'body obsdir  { type=1 size=[0 0 0 .005] X=<T t(',num2str(vel*direction,'%g '),')> color=[0 1 0] fixed }'];
    fprintf(fid, myformat, obsdir);
    
    
    fclose(fid);
    j = j+1
  end
end
return
