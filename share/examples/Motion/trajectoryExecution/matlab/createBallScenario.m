clear;
myformat = '%s\n';

P1 = [0.4 -1.8 2.6]; % border point 1
P2 = [1.1 -1 0.8]; % border point 2

numS = 150;

j = 1;
for j=1:numS
      % copy basefile which is equivilent for all scenes
      filename = ['../scenes/ball/scene',num2str(j)];
      copyfile('robot.ors',filename);
      fid = fopen(filename,'a');
      
      % append line for goalRef
      goalRef = ['body goalRef  { type=1 size=[0 0 0 .03] X=<T t(',num2str(mean([P1;P2]),'%g '),')> color=[1 0 0] fixed }'];
      fprintf(fid, myformat, goalRef);
      
      % append line for goal
      goal = [ 'body goal  { type=1 size=[0 0 0 .005] X=<T t(',num2str(mean([P1;P2]),'%g '),')> color=[0 1 0] fixed }'];
      fprintf(fid, myformat, goal);
      
      % append line for gaol direction
      direction = randn(3,1)';
      vel = 2;
      goaldir = [ 'body dir  { type=1 size=[0 0 0 .005] X=<T t(',num2str(vel*direction,'%g '),')> color=[0 1 0] fixed }'];
      fprintf(fid, myformat, goaldir);
      % append line for obstacle
      % body col  { type=1 size=[0 0 0 0.1] X=<T t( 0.6 -1.6 1.3)> color=[1 0 0] contact, fixed  }
      fclose(fid);
      j = j+1;
end
return
