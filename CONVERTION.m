T = xlsread('CASE2_xyplot.xlsx'); %Reading the data from file
x = T(:,1); %selecting all the rows of 1st coloumn ie, x
y = T(:,2); %selecting all the rows of 2nd coloumn ie, y
n = 1000;

for i=1:n
    [theta,rho] = cart2pol(x,y);
end
xlswrite('C:\Users\Shankha Banerjee\Desktop\FINAL\CASE2_r_theta.xls', [theta,rho]);
