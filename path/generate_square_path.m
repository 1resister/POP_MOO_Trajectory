function path = generate_square_path(cfg)
%GENERATE_SQUARE_PATH Create the 20 mm closed XY square CL polyline.
L = cfg.path.square_side;
points = [0 0 0; L 0 0; L L 0; 0 L 0; 0 0 0];
path = preprocess_polyline(points, true);
path.type = 'square';
path.description = sprintf('Closed square %.6g mm x %.6g mm', L, L);
end
