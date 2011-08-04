function solution = nonlinear_upsampling (image_filename, scan_filename, focal_length)
    dbstop if error
    addpath(genpath('/hydra/S2/snavely/projects/reconstruct/bpsfm/code/refine/'));

    scan = read_scan (scan_filename);
    image = imread (image_filename);

    function x0 = initial_solution (x)
        x0 = x;
        I = find (x == 12);
        while numel(I) > 0
            Isub = I(mod(I-1,1600) == mod(I-2,1600));
            x0(Isub) = x0 (Isub-1);
            I = find (x0 == 12);
            Isub = I(mod(I-1,1600) == mod(I, 1600));
            x0(Isub) = x0 (Isub+1);
            I = find (x0 == 12);
            Isub = I(I <= numel(x) - 1600);
            x0(Isub) = x0 (Isub + 1600);
            I = find (x0 == 12);
            Isub = I(I > 1600);
            x0(Isub) = x0 (Isub - 1600);
            I = find (x0 == 12);
        end
    end

    K = [focal_length 0 1200; 0 focal_length 800; 0 0 1];
    
    display ('Tranforming points')
    tic
    [j i] = ind2sub (size(scan), 1:numel(scan));
    transformed = K\[i-1; j-1; ones(1, numel(scan))];
    toc
    
    display ('Creating smoothness indices')
    tic
    horz_indices1 = zeros(1,1597*2396);
    horz_indices2 = zeros(1,1597*2396);
    horz_indices3 = zeros(1,1597*2396);
    horz_indices4 = zeros(1,1597*2396);
    for i=2:2397
      horz_indices1 ((i-2)*1597+1:(i-1)*1597) = sub2ind(size(scan), 3:1599, (i+1)*ones(1597,1)');
      horz_indices2 ((i-2)*1597+1:(i-1)*1597) = sub2ind(size(scan), 2:1598, i*ones(1597,1)');
      horz_indices3 ((i-2)*1597+1:(i-1)*1597) = sub2ind(size(scan), 2:1598, (i+1)*ones(1597,1)');
      horz_indices4 ((i-2)*1597+1:(i-1)*1597) = sub2ind(size(scan), 3:1599, (i+2)*ones(1597,1)');
    end
    
    vert_indices1 = zeros(1, 1596*2397);
    vert_indices2 = zeros(1, 1596*2397);
    vert_indices3 = zeros(1, 1596*2397);
    vert_indices4 = zeros(1, 1596*2397);
    for i=2:2398
      vert_indices1 ((i-2)*1596+1:(i-1)*1596) = sub2ind(size(scan), 3:1598, i*ones(1596,1)');
      vert_indices2 ((i-2)*1596+1:(i-1)*1596) = sub2ind(size(scan), 2:1597, i*ones(1596,1)');
      vert_indices3 ((i-2)*1596+1:(i-1)*1596) = sub2ind(size(scan), 3:1598, (i+1)*ones(1596,1)');
      vert_indices4 ((i-2)*1596+1:(i-1)*1596) = sub2ind(size(scan), 4:1599, (i+1)*ones(1596,1)');
    end
    
    diag_indices1 = zeros(1, 1597*2397);
    diag_indices2 = zeros(1, 1597*2397);
    diag_indices3 = zeros(1, 1597*2397);
    diag_indices4 = zeros(1, 1597*2397);
    for i=2:2398
      diag_indices1 ((i-2)*1597+1:(i-1)*1597) = sub2ind(size(scan), 2:1598, i*ones(1597,1)');
      diag_indices2 ((i-2)*1597+1:(i-1)*1597) = sub2ind(size(scan), 2:1598, (i+1)*ones(1597,1)');
      diag_indices3 ((i-2)*1597+1:(i-1)*1597) = sub2ind(size(scan), 3:1599, (i+1)*ones(1597,1)');
      diag_indices4 ((i-2)*1597+1:(i-1)*1597) = sub2ind(size(scan), 3:1599, i*ones(1597,1)');
    end
    toc

    rho = @(x) x;
    phi = @(x) x;
    
    depth_indices = find(scan < 12);
    
    k = 1;
    c = 1e-4;
    
    vert_filtered = double(sum(imfilter(image, 1/4*fspecial('sobel')).^2,3));
    vert_smooth = sqrt(reshape(exp(-c*vert_filtered(3:1598,2:2398)), [], 1));
    horz_filtered = double(sum(imfilter(image, 1/4*fspecial('sobel')).^2,3));
    horz_smooth = sqrt(reshape(exp(-c*horz_filtered(2:1598,3:2398)), [], 1));

    function cost = objective (x)
        % Data term
%        display ('Data term')
%        tic
        data_penalty = sqrt(k)*rho(x(depth_indices) - scan(depth_indices));
%        toc
        
        % Smoothness term
%        display ('Smoothness term')
        points = bsxfun (@times, x(:)', transformed);

%        display ('Smoothness term - horz')
%        tic
        a = points(:, horz_indices2) - points(:, horz_indices1);
        b = points(:, horz_indices3) - points(:, horz_indices1);
        c = points(:, horz_indices4) - points(:, horz_indices1);
        unnormA = bsxfun (@cross, a, b);
        unnormB = bsxfun (@cross, b, c);
        AB = unnormA.*unnormB;
        norm = bsxfun (@rdivide, AB, sqrt(sum(unnormA.^2).*sum(unnormB.^2)));
        d = sum(norm)';
        d (d > 1) = 1;
        d (d < 0) = 0;
        horz_penalty = phi(acos(d)).*horz_smooth;
%        toc
        
%        display ('Smoothness term - vert')
%        tic
        a = points(:, vert_indices2) - points(:, vert_indices1);
        b = points(:, vert_indices3) - points(:, vert_indices1);
        c = points(:, vert_indices4) - points(:, vert_indices1);
        unnormA = bsxfun (@cross, a, b);
        unnormB = bsxfun (@cross, b, c);
        AB = unnormA.*unnormB;
        norm = bsxfun (@rdivide, AB, sqrt(sum(unnormA.^2).*sum(unnormB.^2)));
        d = sum(norm)';
        d (d > 1) = 1;
        d (d < 0) = 0;
        vert_penalty = phi(acos(d)).*vert_smooth;
%        toc

%        display ('Smoothness term - diag')
%        tic
        a = points(:, diag_indices2) - points(:, diag_indices1);
        b = points(:, diag_indices3) - points(:, diag_indices1);
        c = points(:, diag_indices4) - points(:, diag_indices1);
        unnormA = bsxfun (@cross, a, b);
        unnormB = bsxfun (@cross, b, c);
        AB = unnormA.*unnormB;
        norm = bsxfun (@rdivide, AB, sqrt(sum(unnormA.^2).*sum(unnormB.^2)));
        d = sum(norm)';
        d (d > 1) = 1;
        d (d < 0) = 0;
        diag_penalty = phi(acos(d));
%        toc
 
        cost = [data_penalty; horz_penalty; vert_penalty; diag_penalty];
    end

    display ('Generating initial solution')
    tic
    x0 = initial_solution (scan);
    toc

    display ('Generating Jacobian sparsity pattern')
    tic
    length1 = numel(depth_indices);
    length2 = length1 + numel(horz_indices1);
    length3 = length2 + numel(vert_indices1);
    length4 = length3 + numel(diag_indices1);
    jacobPattern = sparse ([1:length1,...
                            length1+1:length2,...
                            length1+1:length2,...
                            length1+1:length2,...
                            length1+1:length2,...
                            length2+1:length3,...
                            length2+1:length3,...
                            length2+1:length3,...
                            length2+1:length3,...
                            length3+1:length4,...
                            length3+1:length4,...
                            length3+1:length4,...
                            length3+1:length4],...
                           [depth_indices',...
                            horz_indices1,...
                            horz_indices2,...
                            horz_indices3,...
                            horz_indices4,...
                            vert_indices1,...
                            vert_indices2,...
                            vert_indices3,...
                            vert_indices4,...
                            diag_indices1,...
                            diag_indices2,...
                            diag_indices3,...
                            diag_indices4],...
                           ones(1, numel(depth_indices)+4*numel(horz_smooth)+4*numel(vert_smooth)+4*numel(diag_indices1)),...
                           length4,...
                           numel(scan));
    toc

    options = optimset('PlotFcns', @optimplotfval, 'JacobPattern', jacobPattern, 'PrecondBandWidth', 1, 'Diagnostics', 'on', 'Display', 'iter-detailed');

    display ('Running lsqnonlin');
    tic
    global refine_optlevel
    refine_optlevel = 2;
    x = lsqnonlindave (@objective, x0(:), [], [], options);
%    x = lsqnonlin (@objective, x0(:), [], [], options);
    toc
    display ('Done');
    beep

    solution = reshape (x, size(image, 1), size(image, 2));
end
