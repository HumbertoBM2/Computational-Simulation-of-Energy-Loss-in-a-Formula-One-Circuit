clear all;
clc; 
xi = 10 ; yi = 170;  % punto inicial de la curva
xf = 280 ; yf = 120; % punto final de la curva

xr = [xi;130;230;xf]; % puntos en x por donde debe pasar la curva
yr = [yi;200;50;yf]; % puntos en y por donde debe pasar la curva

p = polyfit(xr,yr,3); % ajuste de curva de grado 3
x1 = linspace(xi,xf);
f1 = polyval(p,x1);
% grafica y formato de la misma
plot(xr,yr,'hexagram', 'Color', 'black','MarkerSize',8,'MarkerFaceColor','black')
hold on
grid on
plot(x1,f1,'r--', 'Color','blue')
daspect([1 1 1]);
title('Zona de curvas de la pista')
q = polyder(p); % derivada del polinomio que modela la zona de curvas
q2 = polyder(q); % segunda derivada del polinomio que modela la zona de curvas
pendiente(1,1) = polyval(q, xi);

f1d = polyval(q,xi:xf);
f2d = polyval(q2,xi:xf);

fun = @(xr) sqrt(1+(q(1)*xr.^2+q(2)*xr+q(3)).^2); % formula sin integrar de la longitud de la curva

L = integral(fun, xi, xf); % Integración de la formula para la longitud de la curva

R = ((1+f1d.^2).^(3/2))./abs(f2d); % formula para el radio de curvatura




Fm = 100;
m = 900;
At = 0.3;
cd = 0.85;
mus = 0.9;
g = 9.8;
Densidad = 1.225;
v(1,1) = 30;
a(1,1) = (Fm-(1/2)*Densidad*At*cd*v(1,1)^2)/m;
ang(1,1) = atand(pendiente(1,1));
vx(1,1) = v(1,1)*cosd(ang(1,1));
vy(1,1) = v(1,1)*sind(ang(1,1));
x(1,1) = xi;
y(1,1) = yi;

h = 0.001;

for i = 1:100000
    x(i+1,1) = x(i,1)+vx(i,1)*h;
    y(i+1,1) = y(i,1)+vy(i,1)*h;
    pendiente(i+1,1) = polyval(q,x(i+1));

    ang(i+1,1) = atand(pendiente(i+1,1));

    v(i+1,1) = v(i,1)+a(i,1)*h;
    vx(i+1,1) = v(i+1,1)*cosd(ang(i+1,1));
    vy(i+1,1) = v(i+1,1)*sind(ang(i+1,1));

    f1dfor = polyval(q,x(i+1));
    f2dfor = polyval(q2,x(i+1));
    Rfor = ((1+f1dfor^2)^(3/2))/abs(f2dfor);

    Vmax = sqrt(mus*g*Rfor)

    if v(i+1) > Vmax
        f = msgbox("Derrape");
        break
    end

    a(i+1,1) = (Fm-(1/2)*Densidad*At*cd*v(i+1,1)^2)/m;

    if x(i+1,1)>xf
        break
    end
end

plot(x,y,'*')

xd(1,1) = x(end);
yd(1,1) = y(end);
md = polyval(q,xd);
angd = atand(md);
Vd(1,1) = v(end);
muk = 0.3;
ad(1,1) = (-muk*m*g-(1/2)*Densidad*At*cd*Vd(1,1)^2)/m;
vxd(1,1) = Vd(1,1)*cosd(angd);
vyd(1,1) = Vd(1,1)*sind(angd);

for i = 1 : 2000000
    xd(i+1,1) = xd(i,1) +vxd(i,1)*h;
    yd(i+1,1) = yd(i,1) +vyd(i,1)*h;

    Vd(i+1,1) = Vd(i,1)+ad(i,1)*h;
    vxd(i+1,1) = Vd(i+1,1)*cosd(angd);
    vyd(i+1,1) = Vd(i+1,1)*sind(angd);

    ad(i+1,1) = (-muk*m*g-(1/2)*Densidad*At*cd*Vd(i+1,1)^2)/m;
    if Vd(i+1,1) <= 0
        break
    end
end

plot(xd,yd,'-')

Rmin = min(R); % Radio de curvatura minimo
Rmax = max(R); % Radio de curvatura maximo

syms x % Se declara la variable x
% se define el polinomio que modela la zona de curvas
funpol = ((1801*x^3)/17820000)-((20203*x^2)/445500)+((169267*x)/35640)+(565519/4455);
funpolprima = diff(funpol); % Se deriva el polinomio
maxmins = solve(funpolprima==0,x); % Se iguala el resultado de la derivacion a 0
                                   % y se despeja para x





