%load /tmp/kmatzen/initial_solution.mat
%x0 = x;

drows = 1600;
dcols = 2400;

tic
d = dir('~/output/*.raw');
depth_array_data = cell (numel(d), 1);
depth_array_cols = cell (numel(d), 1);
depth_array_rows = zeros (numel(d), 1);
images = numel(d);
parfor i=1:images
    f = fopen (['~/output/' d(i).name]);
    drows = fread (f, 1, 'uint32');
    dcols = fread (f, 1, 'uint32');
    depth = fread (f, drows*dcols, 'float32');
    depth = reshape (depth, dcols, drows)';
    depth = depth(:);
    depth_array_cols {i} = find (depth < 12);
    depth_array_rows (i) = numel(depth_array_cols {i});
    depth_array_data {i} = depth(depth_array_cols {i});
    fclose (f);
    depth = [];
end
display ('Loaded data')
toc
clear d

tic
sparse_cols = cell2mat (depth_array_cols);
clear depth_array_cols
%save /tmp/kmatzen/sparse_cols.mat sparse_cols
%clear sparse_cols
sparse_data = cell2mat (depth_array_data);
clear depth_array_data
sparse_rows = zeros (numel(sparse_data), 1);
position = 1;
for i=1:numel(depth_array_rows)
    sparse_rows (position:position+depth_array_rows (i)-1) = i;
    position = position + depth_array_rows (i);
end
clear depth_array_rows

%load /tmp/kmatzen/sparse_cols.mat
depth_data = sparse (sparse_rows, sparse_cols, sparse_data, images, 2400*1600);
clear sparse_rows sparse_cols sparse_data
display ('Created sparse data')
toc

tic
m = zeros (1, size(depth_data,2));
parfor i=1:size(depth_data, 2)
    nonzero_indices = find(depth_data(:,i));
    if (numel(nonzero_indices) > 0)
        values = sort(depth_data(nonzero_indices,i));
        m(i) = values(min([3 numel(values)]));
    end
end
display ('Found min')
toc

tic
depth_data (bsxfun(@gt, depth_data, m + 0.1)|bsxfun(@lt, depth_data, m - 0.1)) = 0;
display ('Filtered by min')
toc

tic
m = zeros (1, size(depth_data, 2));
parfor i=1:size(depth_data, 2)
    nonzero_indices = find(depth_data(:,i));
    if (numel(nonzero_indices) > 0)
        m(i) = median(depth_data(nonzero_indices,i));
    end
end
clear depth_data
f=fopen('result.raw');
fread(f, 1, 'uint32');
fread(f, 1, 'uint32');
existing = fread(f, 2400*1600, 'float32');
existing = reshape (existing, 2400, 1600)';
existing = existing(:)';
mask = abs(existing - m) < 0.02;
m = m.*mask;
save m.mat m
display ('Found mean')
toc

tic
depth_indices = find (m);
depth_values = m(depth_indices);
unary_num = numel(depth_indices);
display ('Constructed A data terms')
toc

k = 1e-2;
c = 1e-2;

tic
binary_horz_num = (dcols-1)*drows;
binary_vert_num = dcols*(drows-1);

rows = unary_num + binary_horz_num + binary_vert_num;
cols = dcols*drows;

vert_cols = zeros (binary_vert_num, 2);
position = 1;
for i=1:dcols
    vert_cols (position:position+drows-2, 1) = sub2ind ([drows dcols], 2:drows, i*ones(drows-1,1)');
    vert_cols (position:position+drows-2, 2) = sub2ind ([drows dcols], 1:drows-1, i*ones(drows-1,1)');
    position = position + drows - 1;
end

horz_cols = [drows+1:dcols*drows; 1:drows*(dcols-1)]';

image = double(imread('~/test.jpg'));

horz_weights = reshape(exp(-c*sum((image (:, 2:end, :) - image (:, 1:end-1, :)).^2, 3)), [], 1);

vert_weights = reshape(exp(-c*sum((image (2:end, :, :) - image (1:end-1, :, :)).^2, 3)), [], 1);
display ('Created smoothness terms')
toc

clear image

tic
A = sparse ([1:unary_num...
             unary_num+1:unary_num+binary_horz_num...
             unary_num+1:unary_num+binary_horz_num...
             unary_num+binary_horz_num+1:rows...
             unary_num+binary_horz_num+1:rows],...
            [depth_indices';...
             horz_cols(:,1);...
             horz_cols(:,2);...
             vert_cols(:,1);...
             vert_cols(:,2)],...
            [sqrt(k)*ones(unary_num, 1);...
             sqrt(horz_weights);...
             -sqrt(horz_weights);...
             sqrt(vert_weights);...
             -sqrt(vert_weights)],...
            rows,...
            cols);
display ('Created A')
toc

tic
b = sparse (1:unary_num,...
            ones(unary_num, 1),...
            sqrt(k)*depth_values,...
            rows,...
            1);
display ('Created b')
toc

tic          
%x = A\b;
x = pcg (A'*A, A'*b, 1e-7, 10000);%, sparse([]), sparse([]), sparse(m'));
display ('pcg done')
toc

clear m

result = reshape (full(x), drows, dcols);

f = fopen('result2.raw', 'wb');
fwrite (f, drows, 'uint32');
fwrite (f, dcols, 'uint32');
result = result';
fwrite (f, result(:), 'float32');
fclose (f);
display ('Done')

exit
