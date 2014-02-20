function createLatexTable(taskName,resNames,resMeans,resStd)

fid = fopen(['latex/',taskName],'w');
myformat = '%s\n';
nRes = size(resMeans,1);

fprintf(fid, myformat, ['\cline{1-',num2str(nRes+2),'}']);
fprintf(fid, myformat, ['\multicolumn{1}{ |c| }{\multirow{3}{*}{',taskName,'} } &']);

pfcRes = '\multicolumn{1}{ |c| }{AMEX} & ';
mpcRes = '\multicolumn{1}{ |c| }{} & CR & ';
dmpRes = '\multicolumn{1}{ |c| }{} & DMP & ';
formatSpec = '%10.2e\n';

dmpRes = [dmpRes,num2str(resMeans(1,1),single(2)),' & '];
pfcRes = [pfcRes,num2str(resMeans(1,2),single(2)),' & '];
mpcRes = [mpcRes,num2str(resMeans(1,3),single(2)),' & '];

for i=2:nRes
  dmpRes = [dmpRes,num2str(resMeans(i,1),formatSpec)];
  pfcRes = [pfcRes,num2str(resMeans(i,2),formatSpec)];
  mpcRes = [mpcRes,num2str(resMeans(i,3),formatSpec)];
  
  if ~isnan(resStd(i,1))
    dmpRes = [dmpRes,' $\pm$',num2str(resStd(i,1),formatSpec)];
    pfcRes = [pfcRes,' $\pm$',num2str(resStd(i,2),formatSpec)];
    mpcRes = [mpcRes,' $\pm$',num2str(resStd(i,3),formatSpec)];
  end
  dmpRes = [dmpRes,' & '];
  pfcRes = [pfcRes,' & '];
  mpcRes = [mpcRes,' & '];
end

fprintf(fid, myformat, [pfcRes,'\\']);
fprintf(fid, myformat, [dmpRes,'\\']);
fprintf(fid, myformat, [mpcRes,'\\']);
fprintf(fid, myformat, ['\cline{1-',num2str(nRes+2),'}']);
fclose(fid);

fid = fopen('latex/header','w');
% & & Success Rate & Sum of Squared Accelerations & Final Time &  Computational Time \\
fprintf(fid, myformat, '\begin{tabular}{cc|c|c|c|c|l}');
fprintf(fid, myformat, '\cline{3-6}');
header = '&';
for i=1:nRes
  header = [header,' & ',resNames{i}];
end
header = [header,'\\'];
fprintf(fid, myformat, header);

end