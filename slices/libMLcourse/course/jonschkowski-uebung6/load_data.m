function X = load_data(filename, d)
	
	% alternativ load benutzen ??
	f = fopen(filename);
	data = fscanf(f , ['%f'] );
	X = reshape(data, d, []).';
	fclose(f);