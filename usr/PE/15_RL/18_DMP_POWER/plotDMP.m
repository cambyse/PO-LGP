clear;
myDefs;
folder = ['data/door2/'];
files = dir([folder,'*.dat']);
names = {files.name};

for i=1:length(names)
 load([folder,names{i}]);
end