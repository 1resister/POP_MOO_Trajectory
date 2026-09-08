function path = load_csv_path(filename, closed)
%LOAD_CSV_PATH Load numeric X/Y[/Z] CL points from a CSV file [mm].
arguments
    filename (1,:) char
    closed (1,1) logical = false
end
data = readmatrix(filename);
if size(data, 2) < 2
    error('POP_MOO:InvalidCSV', 'CSV must contain at least X and Y columns.');
end
data = data(all(isfinite(data(:, 1:min(3, size(data,2)))), 2), :);
points = data(:, 1:min(3, size(data,2)));
if size(points, 2) == 2
    points(:,3) = 0;
end
if closed && norm(points(1,:) - points(end,:)) > 1e-12
    points(end+1,:) = points(1,:);
end
path = preprocess_polyline(points, closed);
path.type = 'csv';
path.source = filename;
end
