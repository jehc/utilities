function scan = read_scan (file)
    f = fopen (file);
    rows = fread(f, 1, 'uint32');
    cols = fread(f, 1, 'uint32');
    data = fread(f, rows*cols, 'float32');
    scan = reshape (data, cols, rows)';
end