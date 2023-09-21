clear all
close all
clc
x1gradas = 10;
y1gradas = 170;
x2gradas = 280;
y2gradas = 120;

x3gradas = 130;
y3gradas = 200;
x4gradas = 230;
y4gradas = 50;

Ygradas = [y1gradas; y2gradas; y3gradas; y4gradas];
Xgradas = [x1gradas^3 x1gradas^2 x1gradas 1; x2gradas^3 x2gradas^2 x2gradas 1; x3gradas^3 x3gradas^2 x3gradas 1; x4gradas^3 x4gradas^2 x4gradas 1];
Agradas = inv(Xgradas)*Ygradas;

xgradas = linspace(x1gradas,x2gradas,201);

fcurvagradas = Agradas(1)*xgradas.^3 + Agradas(2)*xgradas.^2 +Agradas(3)*xgradas +Agradas(4);

dxgradas = [];
dygradas = [];
dsgradas = [];
der2xgradas = [];
der2ygradas =[];
Rgradas = [];
cgradas = [];

for i = 1:length(xgradas)-1
    dxgradas = [dxgradas,xgradas(i+1)-xgradas(i)];
    dygradas = [dygradas,fcurvagradas(i+1)-fcurvagradas(i)];
    dsgradas = [dsgradas,sqrt(dxgradas(i)^2 + dygradas(i)^2)];
    der1xgradas = dxgradas./dsgradas;
    der1ygradas = dygradas./dsgradas; 
end


for i = 1:length(der1xgradas)-1
    der2xgradas = [der2xgradas, (der1xgradas(i+1)-der1xgradas(i))/dsgradas(i)];
    der2ygradas = [der2ygradas, (der1ygradas(i+1)-der1ygradas(i))/dsgradas(i)];
% Radio de curvatura
    Rgradas = [Rgradas,1/sqrt(der2xgradas(i).^2+der2ygradas(i).^2)];
end

longitudgradas = sum(dsgradas);
Radmingradas = min(Rgradas);



for j = 2:length(der1ygradas)-1
    if (der1ygradas(j)>0 && der1ygradas(j-1)<0) | (der1ygradas(j)>0 && der1ygradas(j+1)<0)
        cgradas=[cgradas,j];% Guarda los índices donde esto ocurre
    end
end

indice1gradas = find(Rgradas==min(Rgradas(cgradas(1)-5:cgradas(1)+5)));
indice2gradas = find(Rgradas==min(Rgradas(cgradas(2)-5:cgradas(2)+5)));
R1gradas = Rgradas(indice1gradas);
R2gradas = Rgradas(indice2gradas);

t1gradas=find(xgradas>(xgradas(indice1gradas)-R1gradas-0.6) & xgradas<(xgradas(indice1gradas)-R1gradas+0.6));% Este debe ser solo un índice
t2gradas=find(xgradas>(xgradas(indice2gradas)-R2gradas-0.6) & xgradas<(xgradas(indice2gradas)-R2gradas+0.6));


m1gradas = der1ygradas(t1gradas)/der1xgradas(t1gradas);
b1gradas = fcurvagradas(t1gradas) - m1gradas*xgradas(t1gradas); % Punto de corte
coorx1gradas = [xgradas(t1gradas-26):xgradas(t1gradas+20)];
tangente1gradas = m1gradas*coorx1gradas + b1gradas;% REcta tangente al punto critico

m2gradas = der1ygradas(t2gradas)/der1xgradas(t2gradas);
b2gradas = fcurvagradas(t2gradas) - m2gradas*xgradas(t2gradas); % Punto de corte
coorx2gradas = [xgradas(t2gradas-26):xgradas(t2gradas+20)];
tangente2gradas = m2gradas*coorx2gradas + b2gradas;


perpendicular1gradas = -(1/m1gradas)*coorx1gradas + (fcurvagradas(t1gradas)+xgradas(t1gradas)/m1gradas);
perpendicular2gradas = -(1/m2gradas)*coorx2gradas + (fcurvagradas(t2gradas)+xgradas(t2gradas)/m2gradas);

paralela1gradas = tangente1gradas+23;% Recta a 20 unidades de la recta tangente al punto critico
paralela2gradas = tangente2gradas-23;


 
figure(1)
hold on 
%plot(xgradas,fcurvagradas,'LineWidth',2)
plot(coorx1gradas,tangente1gradas,'black','LineWidth',2)
plot(coorx2gradas,tangente2gradas,'black','LineWidth',2)
%plot(coorx1gradas,paralela1gradas,'g','LineWidth',2)
%plot(coorx2gradas,paralela2gradas,'g','LineWidth',2)
plot(coorx1gradas,perpendicular1gradas,'black','LineWidth',2)
plot(coorx2gradas,perpendicular2gradas,'black','LineWidth',2)

plot(xgradas(t1gradas),fcurvagradas(t1gradas),'o','MarkerFaceColor','red','MarkerSize',6)
plot(xgradas(t2gradas),fcurvagradas(t2gradas),'o','MarkerFaceColor','red','MarkerSize',6)

punto1 = 60;
punto2 = 300;

punto3 = 4.15361;
punto4 = 243.7138;

punto5 = -4.224197604174;
punto6 = 251.2098386066039;

punto7 = 52.6567518574205;
punto8 = 307.6498504754727;

punto9= 168.5857808645159;
punto10 = 71.6697939358922;

punto11 = 227.7527419724195;
punto12 = 16.9324342207233;

punto13 = 220.44968256758;
punto14 = 9.435308434908;

punto15 = 161.346511139683;
punto16 = 64.7247973200413;

grada1 = [punto1 punto3];
grada2 = [punto2 punto4];

grada3 = [punto3 punto5];
grada4 = [punto4 punto6];

grada5 = [punto1 punto7];
grada6 = [punto2 punto8];

grada7 = [punto5 punto7];
grada8 = [punto6 punto8];

grada9 = [punto9 punto11];
grada10 = [punto10 punto12];

grada11 = [punto15 punto13];
grada12 = [punto16 punto14];

grada13 = [punto9 punto15];
grada14 = [punto10 punto16];

grada15 = [punto11 punto13];
grada16 = [punto12 punto14];

plot (grada1, grada2,'g','LineWidth',2)
plot(grada3, grada4,'g','LineWidth',2)
plot(grada5, grada6,'g','LineWidth',2)
plot(grada7, grada8,'g','LineWidth',2)
plot(grada9, grada10,'g','LineWidth',2)
plot(grada11, grada12,'g','LineWidth',2)
plot(grada13, grada14,'g','LineWidth',2)
plot(grada15, grada16,'g','LineWidth',2)

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
m = 789;
At = 0.3;
cd = 0.85;
mus = 0.9;
g = 9.8;
Densidad = 1.225;
v(1,1) = 9.71;
a(1,1) = (Fm-(1/2)*Densidad*At*cd*v(1,1)^2)/m;
ang(1,1) = atand(pendiente(1,1));
vx(1,1) = v(1,1)*cosd(ang(1,1));
vy(1,1) = v(1,1)*sind(ang(1,1));
x(1,1) = xi;
y(1,1) = yi;

Eki = (1/2)*m*(v(1,1)^2);
Ekn(1,1) = Eki; 

h = 0.01;

for i = 1:100000
    x(i+1,1) = x(i,1)+vx(i,1)*h;
    y(i+1,1) = y(i,1)+vy(i,1)*h;
   

    pendiente(i+1,1) = polyval(q,x(i+1));

    ang(i+1,1) = atand(pendiente(i+1,1));

    v(i+1,1) = v(i,1)+a(i,1)*h;
    vx(i+1,1) = v(i+1,1)*cosd(ang(i+1,1));
    vy(i+1,1) = v(i+1,1)*sind(ang(i+1,1));
    Ekn(i+1,1) = (1/2)*m*(v(i+1,1)^2);
    f1dfor = polyval(q,x(i+1));
    f2dfor = polyval(q2,x(i+1));
    Rfor = ((1+f1dfor^2)^(3/2))/abs(f2dfor);

    Vmax = sqrt(mus*g*Rfor)

    if v(i+1) > Vmax
        f = warndlg('Se originó un derrape','Advertencia');
        break
    end

    a(i+1,1) = (Fm-(1/2)*Densidad*At*cd*v(i+1,1)^2)/m;

    if x(i+1,1)>xf
        break
    end
end

plot(x,y,'*','Color','cyan')
Ekd(1,1) = Ekn(end);
xd(1,1) = x(end);
yd(1,1) = y(end);
md = polyval(q,xd);
angd = atand(md);
Vd(1,1) = v(end);
muk = 0.3;
ad(1,1) = (-(1/2)*Densidad*At*cd*Vd(1,1)^2); %%%%%%%%%%%%%%%%
vxd(1,1) = Vd(1,1)*cosd(angd);
vyd(1,1) = Vd(1,1)*sind(angd);

for i = 1 : 2000000
    xd(i+1,1) = xd(i,1) +vxd(i,1)*h;
    yd(i+1,1) = yd(i,1) +vyd(i,1)*h;

    Vd(i+1,1) = Vd(i,1)+ad(i,1)*h;
    vxd(i+1,1) = Vd(i+1,1)*cosd(angd);
    vyd(i+1,1) = Vd(i+1,1)*sind(angd);
    Ekd(i+1,1) = (1/2)*m*(Vd(i+1,1)^2);
    ad(i+1,1) = (-(1/2)*Densidad*At*cd*Vd(i+1,1)^2); %%%%%%%%%%%%%%%
    if Vd(i+1,1) <= 0
        break
    end
end


Eperdida = Ekd(end)-Eki

plot(xd,yd,'red','LineWidth',2)

Rmin = min(R); % Radio de curvatura minimo
Rmax = max(R); % Radio de curvatura maximo

syms x % Se declara la variable x
% se define el polinomio que modela la zona de curvas
funpol = ((1801*x^3)/17820000)-((20203*x^2)/445500)+((169267*x)/35640)+(565519/4455);
funpolprima = diff(funpol); % Se deriva el polinomio
maxmins = solve(funpolprima==0,x); % Se iguala el resultado de la derivacion a 0
                                   % y se despeja para x












