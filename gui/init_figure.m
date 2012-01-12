function fh = init_figure(ftitle, menubar, toolbar, color)
% initializes a figure
%   ftitle - string title
%   menubar - bool flag to display the menu bar
%   toolbar - bool flag to display the tool bar
%   color - background color

if nargin < 4
  color = 'w';
end
if nargin < 3
  toolbar = true;
end
if nargin < 2
  menubar = true;
end
if nargin < 1
  ftitle = '';
end

fh = figure();
set(fh, 'Name', ftitle);
set(fh, 'NumberTitle', 'off');
if menubar
  set(fh, 'MenuBar', 'figure');
else
  set(fh, 'MenuBar', 'none');
end
if toolbar
  set(fh, 'Toolbar', 'figure');
else
  set(fh, 'Toolbar', 'none');
end
set(fh, 'Color', color);
