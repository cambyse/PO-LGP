C = 5;
N = 10000;
D = 2;

data = [];
clusters = zeros(C,D);

for cli = 1:C
  clusters(cli,:) = 200 * rand(1,D);
  v = rand(1) * 10;
  d = ones(N,1) * clusters(cli,:) + v * randn(N, D);
  data = [data; d];
end

clf
scatter(data(:,1), data(:,2),100,'.g'); 
hold on;
scatter(clusters(:,1), clusters(:,2),1000,'.r'); 
hold off;

output = fopen('z.data','w');
for row = 1:N*C
  for col = 1:D
    fprintf(output, '%3f ', data(row, col));
  end
  fprintf(output, '\n');
end
fclose(output);
