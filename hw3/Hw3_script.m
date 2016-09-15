%{
NOTES:

***Test1 regular appears broken -- none matching number of lines***

To load a csv file:
1) open the file location from the address bar of matlab
2) csvread('test1_regular_twistCommand.csv',1)

To load a specific column:
result = variable(:,[columnNum])

Ex:
t_in = ans(:,[1])
out = test1_regular_twistStatus(:,[4])

To shift values:
minimum = min(t_in)
t_in = t_in - minimum
t_out = t_out - minimum 


%}
%Test1_regular has inconsistent lengths 

test2_regular_twistCommand  = csvread('regular/test2_regular_twistCommand.csv')
test2_regular_twistStatus   = csvread('regular/test2_regular_twistStatus.csv')

test2_regular_t_in          = test2_regular_twistCommand(:,1)
test2_regular_in            = test2_regular_twistCommand(:,2)

test2_regular_t_out         = test2_regular_twistStatus(:,1)
test2_regular_out           = test2_regular_twistStatus(:,4)

% Shift times
test2_time_minimum          = min(test2_regular_t_in)
test2_regular_t_in          = test2_regular_t_in - test2_time_minimum
test2_regular_t_out         = test2_regular_t_out - test2_time_minimum

%{
% Trim sizes of in, out to smaller of the two 
size = min(size(test2_regular_in,1), size(test2_regular_out,1))
test2_regular_t_in = test2_regular_t_in(1:size,:)
test2_regular_in = test2_regular_in(1:size,:)

test2_regular_t_out = test2_regular_t_out(1:size,:)
test2_regular_out = test2_regular_out(1:size,:)
%}

% Outlier removal using Find
%   TODO


% plot 
figure
plot(test2_regular_t_in, test2_regular_in, 'b', test2_regular_t_out, test2_regular_out, 'r-.', 'LineWidth', 2)
grid on

% Set the axis limits
%axis([0 7 -4 4])

% Add title and axis labels
title('Plot for Test2, Regular configuration')
xlabel('time')
ylabel('value')