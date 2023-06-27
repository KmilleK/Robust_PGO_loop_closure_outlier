% Step 1: Define the functions
L2 = @(x) x.^2;
L1 = @(x) abs(x);

Charbonnier=@(x) sqrt(x.^2+1)-1;
Cauchy=@(x)  log(1+x.^2);
Geman=@(x)  (x.^2/2)./(1+x.^2);


% Step 2: Define the range of x-values
x = linspace(-10, 10, 100); % Range from -π to π with 100 points

% Step 3: Evaluate the functions
y1 = L1(x);
y2 = L2(x);
y3= huber(x);
y4=Charbonnier(x);
y5=Cauchy(x);
y6=Geman(x);

% Step 4: Plot the functions

plot(x, y1, 'blue-');  % Blue line
hold on;
plot(x, y2, 'red-');  % Red line 
hold on;
plot(x,y3, 'color',[0.4660 0.6740 0.1880]);  % Blue line
hold on;
plot(x, y4 ,'magenta-');  % Red line 
hold on;
plot(x, y5, 'black-');  % Blue line
hold on;
plot(x, y6, 'color', [0.3010 0.7450 0.9330]);  % Red line $
%plot(x, y6, 'red-');  % Red line $

hold on;
% Add labels and a legend
xlabel('x');
ylabel('\rho');
legend('L1', 'L2','Huber','Charbonnier','Cauchy','Geman');

% Optionally, you can add a title
title('Kernel function behaviour');

% Optionally, you can adjust the axis limits if needed
xlim([-11, 11]);
ylim([0, 10]);

% Turn off the hold
hold off;

function y=huber(x)
    if abs(x)<1
        y=x^2;
    else 
        y=2*abs(x)-1;
    end
end
