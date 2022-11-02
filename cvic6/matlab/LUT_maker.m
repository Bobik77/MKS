clear all;
close all;

order = 20; %polynom order
resolution = 1024; %12bit
r1 = 10; %kOhm
t = importfile("C:\Users\xvanek39\Documents\MKS\cvic6\matlab\ntc.csv", [1, Inf]);
temp = t.temp;
r = t.val;

ad = (r)./(r+r1).*2^10;
ad2 = 0:1023;
poly = polyfit(ad, temp, 10);
t2 = round(polyval(poly, ad2), 1);

figure(1)
subplot(3,1,1)
plot(temp,r,'xr');
hold on;
grid on;
xlabel("Temperature");
ylabel("Resistivity");
subplot(3,1,2)
plot(ad);
xlabel("ADC\_val");
ylabel("Temp")
subplot(3,1,3)
plot(ad2,t2);
ylabel("Temp")
xlabel("ADC\_val");

dlmwrite('data.dlm', t2*10, ',');

