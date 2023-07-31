% Step 1: Define all the  functions
s=1;
L2 = @(x) x.^2/2;
L1 = @(x) abs(x);
Charbonnier=@(x,s) sqrt((x./s).^2+1)-1; 
Fair=@(x,s) s.^2.*(abs(x)./s-log(1+abs(x)./s));
Cauchy=@(x,s)  s.^2./2.*log(1+(x./s).^2);
Geman=@(x,s)  (x.^2/2)./(s+x.^2);
Welsch=@(x,s)  s.^2./2.*(1-exp(-(x./s).^2));

% Step 2: Define the range of x-values
x = linspace(-4, 4); % 

% Step 3: Evaluate the functions
y1 = L1(x);
y2 = L2(x);
y3= Charbonnier(x,s);
y4=huber(x,s);
y5=Fair(x,s);
y6=Cauchy(x,s);
y7=Geman(x,s);
y8=Welsch(x,s);
y9=Threshold(x,s);
y10=Tukey(x,s);

% Step 4: Plot the functions
plot(x, y1);  % Blue lineplot(x, y1, 'blue-');  
hold on;
plot(x, y2);  % Blue line
hold on;
plot(x, y3);  % Blue line
hold on;
plot(x, y4);  % Blue line
hold on;
plot(x, y5);  % Blue line
hold on;
plot(x, y6);  % Blue lineplot(x, y1, 'blue-');  
hold on;
plot(x, y7);  % Blue line
hold on;
plot(x, y8);  % Blue line
hold on;
plot(x, y9);  % Blue line
hold on;
plot(x, y10);  % Blue line


xlabel('x');
ylabel('\rho');
legend('L1','L2','Charbonnier','huber','Fair','Cauchy','Geman','Welsch','Threshold','Tukey')
% Optionally, you can add a title


% Optionally, you can adjust the axis limits if needed
xlim([-2, 2]);
ylim([0, 2]);

% Turn off the hold
hold off;

function y=huber(x,s)
    
    for i=1:length(x)
        if abs(x(i))<=s
            y(i)=x(i)^2/2;
        else 
            y(i)=s*(abs(x(i))-s/2);
        end
    end
end

function y=Threshold(x,s)
    
    for i=1:length(x)
        if abs(x(i))<=s
            y(i)=x(i)^2/2;
        else 
            y(i)=s^2/2;
        end
    end
end

function y=Tukey(x,s)
    
    for i=1:length(x)
        if abs(x(i))<=s
            y(i)=s^2*(1-(1-(x(i)/s)^2)^3);
        else 
            y(i)=s^2/6;
        end
    end 
end
