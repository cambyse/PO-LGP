function [X y] = load_data(filename, d)
	
	% alternativ load benutzen ??
	f = fopen(filename);
	data = fscanf(f , [ '%f'] );
	data = reshape(data, d+1, []).';
	X = data(:,1:d);
	y = data(:,d+1);
	fclose(f);