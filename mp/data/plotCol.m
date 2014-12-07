#!/usr/bin/env octave -fq

if (nargin <= 3)
    printf('usage %s filename colY colX y_label x_label [format=eps]', program_name());
    exit(0);
endif

arg_list = argv ();

filename = arg_list{1}
[pathstr, name, ext] = fileparts(filename);

figure ('visible', 'off');

colY = str2num(arg_list{2})
colX = str2num(arg_list{3})
y_label = arg_list{4}
x_label = arg_list{5}

A = load(filename);

if(colX <= 0)
    h = plot(A(:,colY));
else
    h = plot(A(:,colX), A(:,colY));
end

t = findall (0, '-property', 'fontname'); 
set (t, 'fontname', 'Courier');
set(h, 'linewidth', 4);
axis('tight');

% set(gca(), 'linewidth', 2, 'xtick', [0:round(size(A,1)/50)*10:size(A,1)]);
%set(gca(), 'linewidth', 2, 'ytick', [-180:30:180]);

set(gca,'fontsize',24);
ylabel(y_label,'fontsize',24);
xlabel(x_label,'fontsize',24);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save figure to file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

format = 'eps';
if (nargin > 5)
    format = arg_list{6};
endif

output = strcat(name, '_', num2str(colY), '_', num2str(colX), ext,  '.', format);

printf('save figure to %s\n', output);

print(output, strcat('-d',format), '-color','-S800,600');