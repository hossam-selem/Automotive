% Example: Loading the Excel data
data = readmatrix('Book1.xlsx');

% Assuming first column is breakpoints, second column is data
breakpoints = data(:,1);
torque_values = data(:,2);

% Assign to Simulink block (manual or script)
set_param('EV/Torque/NEDC', 'BreakpointsForDimension1', 'breakpoints');
set_param('EV/Torque/NEDC', 'Table', 'torque_values');
