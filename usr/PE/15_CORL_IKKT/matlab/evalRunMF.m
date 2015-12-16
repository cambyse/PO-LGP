clear;
myDefs;
addpath('/home/englerpr/Dropbox/research/code/gpml/');
addpath('../2_CORL_MF/matlab_interface/');


folder = ['../data/button3/PE_'];
files = dir([folder,'mf*.dat']);
names = {files.name};

for i=1:length(names)
 load([folder,names{i}]);
end

X = load([folder,'X.dat']);
Y = load([folder,'Y.dat']);
YS = load([folder,'YS.dat']);

paramLimit = load([folder,'Limit.dat']);
nParam = size(paramLimit,1);

buttonDefs;
constraintExploration;
constraintExplorationPlot;