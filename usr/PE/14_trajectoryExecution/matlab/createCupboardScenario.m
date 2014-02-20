clear;
myformat = '%s\n';

P1 = [0.4 -1.8 2.6]; % border point 1
P2 = [1.1 -1 0.8]; % border point 2

numX = 3;
numY = 5;
numZ = 10;

j = 1;
for iterX=1:numX
  for iterY=1:numY
    for iterZ=1:numZ
      % copy basefile which is equivilent for all scenes
      filename = ['../scenes/cupboard/scene',num2str(j)];
      copyfile('robot.ors',filename);
      fid = fopen(filename,'a');
      
      % append line for goalRef
      goalRef = ['body goalRef  { type=1 size=[0 0 0 .03] X=<T t(',num2str(mean([P1;P2]),'%g '),')> color=[1 0 0] fixed }'];
      fprintf(fid, myformat, goalRef);
      
      % append line for goal
      goalPos = P1+(P2-P1).*[(iterX-1)/(numX-1),(iterY-1)/(numY-1),(iterZ-1)/(numZ-1)];
      goal = [ 'body goal  { type=1 size=[0 0 0 .005] X=<T t(',num2str(goalPos,'%g '),')> color=[0 1 0] fixed }'];
      fprintf(fid, myformat, goal);
      
      % append line for obstacle
      % body col  { type=1 size=[0 0 0 0.1] X=<T t( 0.6 -1.6 1.3)> color=[1 0 0] contact, fixed  }
      fclose(fid);
      j = j+1;
    end
  end
end
return
