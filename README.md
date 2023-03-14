# Actividad-4

Dana Marian Rivera Oropeza - A00830027 Daniela Berenice Hernández de Vicente - A01735346 Alejandro Armenta Arellano - A01734879

## Modelo de Energía y Lagrangiano

Después de limpiar el entorno, creamos las variables simbólicas para realizar el cálculo de las masas y matrices de inercia en este caso por ser un robot tipo péndulo se requieren los ángulos, de igual manera tenemos que l1=longitud de eslabones y lc=distancia al centro de masa de cada eslabón, de igual manera declaramos los tiempos. De igual manera las variables simbólicas dependeran del caso del robot a realizar.

###### Robot 1GDL Péndulo
``` matlab
%Declaración de variables simbólicas
syms th1(t) t  %Angulos de cada articulación
syms th1p(t) %Velocidades de cada articulación
syms th1pp(t) %Aceleraciones de cada articulación
syms m1 Ixx1 Iyy1 Izz1  %Masas y matrices de Inercia
syms t1 %Tiempos
syms l1 lc1  %l=longitud de eslabones y lc=distancia al centro de masa de cada eslabón
syms pi g
``` 
###### Robot 2GDL Rotacional
``` matlab
%Declaración de variables simbólicas
syms th1(t) th2(t)  t  %Angulos de cada articulación
syms th1p(t) th2p(t) %Velocidades de cada articulación
syms th1pp(t) th2pp(t) %Aceleraciones de cada articulación
syms m1 m2 m3 Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 %Masas y matrices de Inercia
syms t1 t2 %Tiempos
syms l1 l2 lc1 lc2 %l=longitud de eslabones y lc=distancia al centro de masa de cada eslabón
syms pi g 
``` 
###### Robot 3GDL Cartesiano
``` matlab
%Declaración de variables simbólicas
syms l1(t) l2(t) l3(t) h1(t) t %Angulos de cada articulación
syms l1p(t) l2p(t) l3p(t) t %Angulos de cada articulación
syms l1pp(t) l2pp(t) l3pp(t) t %Angulos de cada articulación
syms m1 m2 m3 Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 %Masas y matrices de Inercia
syms t1 t2 t3%Tiempos
syms lc1 lc2 lc3  %l=longitud de eslabones y lc=distancia al centro de masa de cada eslabón
syms pi g
``` 
Se genera la configuración del robot dependiendo del robot a realizarse.
###### Robot 1GDL Péndulo
``` matlab
RP=[0];
``` 
###### Robot 2GDL Rotacional
``` matlab
RP=[0 0 0];
``` 
###### Robot 3GDL Cartesiano
``` matlab
RP=[0 1 1];
``` 

Se crea el vector de coordenadas articulares dependiendo del robot a realizarse.
###### Robot 1GDL Péndulo
``` matlab
  Q= [th1];
``` 
###### Robot 2GDL Rotacional
``` matlab
  Q= [th1; th2; th3];
``` 

###### Robot 3GDL Cartesiano
``` matlab
Q= [th1, l2, l3];
``` 
Se crea el vector de velocidades articulares y el vector de aceleraciones articulares, correspondiente al robot a utilizar
###### Robot 1GDL Péndulo
``` matlab
  Qp= [th1p];
  Qpp= [th1pp];
``` 
###### Robot 2GDL Rotacional
``` matlab
Qp= [th1p; th2p];
Qpp= [th1pp; th2pp];
``` 

###### Robot 3GDL Cartesiano
``` matlab
Qp= [l1p; l2p; l3p];
Qpp= [l1pp; l2pp; l3pp];
``` 
Se establece el número de grado de libertad del robot siendo en todos los casos el mismo.

``` matlab
GDL= size(RP,2);
GDL_str= num2str(GDL);
```

Se realizan los vectores con las funciones para modelar las posiciones y la rotación de cada articualación en donde dependerá del robot a analizar.
###### Robot 1GDL Péndulo
``` matlab
%Articulación 1 
P(:,:,1)= [l1*cos(th1); l1*sin(th1);0];

R(:,:,1)= [cos(th1) -sin(th1)  0;
           sin(th1)  cos(th1)  0;
           0         0         1];
``` 
###### Robot 2GDL Rotacional
``` matlab
%Articulación 1 
P(:,:,1)= [l1*cos(th1); l1*sin(th1);0];

R(:,:,1)= [cos(th1) -sin(th1)  0;
           sin(th1)  cos(th1)  0;
           0         0         1];
%Articulación 2
P(:,:,2)= [l2*cos(th2); l2*sin(th2);0];

R(:,:,2)= [cos(th2) -sin(th2)  0;
           sin(th2)  cos(th2)  0;
           0         0         1];
``` 

###### Robot 3GDL Cartesiano
``` matlab
%Articulación 1 (Articulacion en x con respecto a y)
P(:,:,1)= [0; 0; l1];

R(:,:,1)= [0  0   1;
          -1  0   0;
           0  1   0];

%Articulación 2 (x)
P(:,:,2)= [0; 0; l2];

R(:,:,2)= [0  0  -1;
           1  0   0;
           0 -1   0];

%Articulación 3 (y)
P(:,:,3)= [0; 0; l3];

R(:,:,3)= [1 0 0;
           0 1 0;
           0 0 1];
```  
Posteriormente se crea un vector inicializado en ceros.
 ``` matlab
Vector_Zeros= zeros(1, 3);
```  
Se generan nuestras matrices globales y locales.
 ###### Robot 1GDL Péndulo
``` matlab
%Inicializamos las matrices de transformación Homogénea locales
A(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las matrices de transformación Homogénea globales
T(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las posiciones vistas desde el marco de referencia inercial
PO(:,:,GDL)= P(:,:,GDL); 
%Inicializamos las matrices de rotación vistas desde el marco de referencia inercial
RO(:,:,GDL)= R(:,:,GDL); 
``` 
###### Robot 2GDL Rotacional
``` matlab
%Inicializamos las matrices de transformación Homogénea locales
A(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las matrices de transformación Homogénea globales
T(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las posiciones vistas desde el marco de referencia inercial
PO(:,:,GDL)= P(:,:,GDL); 
%Inicializamos las matrices de rotación vistas desde el marco de referencia inercial
RO(:,:,GDL)= R(:,:,GDL); 
``` 

###### Robot 3GDL Cartesiano
``` matlab
%Inicializamos las matrices de transformación Homogénea locales
A(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las matrices de transformación Homogénea globales
T(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las posiciones vistas desde el marco de referencia inercial
PO(:,:,GDL)= P(:,:,GDL); 
%Inicializamos las matrices de rotación vistas desde el marco de referencia inercial
RO(:,:,GDL)= R(:,:,GDL); 
 ``` 
 
 En el ciclo for ue se muestra a continuación se ingresan las matrices de cada una de las articulaciones de manera global y local, dependiendo del robot analizado.
 ###### Robot 1GDL Péndulo
``` matlab
for i = 1:GDL
    i_str= num2str(i);
   %disp(strcat('Matriz de Transformación local A', i_str));
    A(:,:,i)=simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
   %pretty (A(:,:,i));

   %Globales
    try
       T(:,:,i)= T(:,:,i-1)*A(:,:,i);
    catch
       T(:,:,i)= A(:,:,i);
    end
%     disp(strcat('Matriz de Transformación global T', i_str));
    T(:,:,i)= simplify(T(:,:,i));
%     pretty(T(:,:,i))

    RO(:,:,i)= T(1:3,1:3,i);
    PO(:,:,i)= T(1:3,4,i);
    %pretty(RO(:,:,i));
    %pretty(PO(:,:,i));
end
``` 
###### Robot 2GDL Rotacional
``` matlab
for i = 1:GDL
    i_str= num2str(i);
   %disp(strcat('Matriz de Transformación local A', i_str));
    A(:,:,i)=simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
   %pretty (A(:,:,i));
   %Globales
    try
       T(:,:,i)= T(:,:,i-1)*A(:,:,i);
    catch
       T(:,:,i)= A(:,:,i);
    end
%     disp(strcat('Matriz de Transformación global T', i_str));
    T(:,:,i)= simplify(T(:,:,i));
%     pretty(T(:,:,i))
    RO(:,:,i)= T(1:3,1:3,i);
    PO(:,:,i)= T(1:3,4,i);
    %pretty(RO(:,:,i));
    %pretty(PO(:,:,i));
end
``` 

###### Robot 3GDL Cartesiano
``` matlab
for i = 1:GDL
    i_str= num2str(i);
   %disp(strcat('Matriz de Transformación local A', i_str));
    A(:,:,i)=simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
   %pretty (A(:,:,i));
   %Globales
    try
       T(:,:,i)= T(:,:,i-1)*A(:,:,i);
    catch
       T(:,:,i)= A(:,:,i);
    end
%     disp(strcat('Matriz de Transformación global T', i_str));
    T(:,:,i)= simplify(T(:,:,i));
%     pretty(T(:,:,i))
    RO(:,:,i)= T(1:3,1:3,i);
    PO(:,:,i)= T(1:3,4,i);
    %pretty(RO(:,:,i));
    %pretty(PO(:,:,i));
end
 ``` 
  
Se obtiene el Jacobiano lineal de forma diferencial y parciales con respecto a cada uno de los ejes, dependiendo del robot a analizar.
 ###### Robot 1GDL Péndulo
``` matlab
%Calculamos el jacobiano lineal de forma diferencial
%disp('Jacobiano lineal obtenido de forma diferencial');
%Derivadas parciales de x respecto a th1 y th2
Jv11= functionalDerivative(PO(1,1,GDL), th1);
%Derivadas parciales de y respecto a th1 y th2
Jv21= functionalDerivative(PO(2,1,GDL), th1);
%Derivadas parciales de z respecto a th1 y th2
Jv31= functionalDerivative(PO(3,1,GDL), th1);
``` 
###### Robot 2GDL Rotacional
``` matlab
%Calculamos el jacobiano lineal de forma diferencial
%disp('Jacobiano lineal obtenido de forma diferencial');
%Derivadas parciales de x respecto a th1 y th2
Jv11= functionalDerivative(PO(1,1,GDL), th1);
Jv12= functionalDerivative(PO(1,1,GDL), th2);
%Derivadas parciales de y respecto a th1 y th2
Jv21= functionalDerivative(PO(2,1,GDL), th1);
Jv22= functionalDerivative(PO(2,1,GDL), th2);
%Derivadas parciales de z respecto a th1 y th2
Jv31= functionalDerivative(PO(3,1,GDL), th1);
Jv32= functionalDerivative(PO(3,1,GDL), th2);
``` 
###### Robot 3GDL Cartesiano
``` matlab
%Calculamos el jacobiano lineal de forma diferencial
%disp('Jacobiano lineal obtenido de forma diferencial');
%Derivadas parciales de x respecto a th1 y th2
Jv11= functionalDerivative(PO(1,1,GDL), l1);
Jv12= functionalDerivative(PO(1,1,GDL), l2);
Jv13= functionalDerivative(PO(1,1,GDL), l3);
%Derivadas parciales de y respecto a th1 y th2
Jv21= functionalDerivative(PO(2,1,GDL), l1);
Jv22= functionalDerivative(PO(2,1,GDL), l2);
Jv23= functionalDerivative(PO(2,1,GDL), l3);
%Derivadas parciales de z respecto a th1 y th2
Jv31= functionalDerivative(PO(3,1,GDL), l1);
Jv32= functionalDerivative(PO(3,1,GDL), l2);
Jv33= functionalDerivative(PO(3,1,GDL), l3);
 ``` 
Se crea la matríz del Jacobiano lineal, el resultado será diferente para cada uno de los robots analizados.
 ###### Robot 1GDL Péndulo
``` matlab
jv_d=simplify([Jv11;
              Jv21;
              Jv31]);
``` 
###### Robot 2GDL Rotacional
``` matlab
jv_d=simplify([Jv11 Jv12;
               Jv21 Jv22;
               Jv31 Jv32]);
``` 
###### Robot 3GDL Cartesiano
``` matlab
jv_d=simplify([Jv11 Jv12 Jv13;
              Jv21 Jv22 Jv23;
              Jv31 Jv32 Jv33]);
 ``` 
 
 Se calcula eljacobiano lineal de forma analítica dependiendo de la configuración del robot analizado.
  ###### Robot 1GDL Péndulo
``` matlab
Jv_a(:,GDL)=PO(:,:,GDL);
Jw_a(:,GDL)=PO(:,:,GDL);

for k= 1:GDL
    if RP(k)==0 
       %Para las juntas de revolución
        try
            Jv_a(:,k)= cross(RO(:,3,k-1), PO(:,:,GDL)-PO(:,:,k-1));
            Jw_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)= cross([0,0,1], PO(:,:,GDL));%Matriz de rotación de 0 con respecto a 0 es la Matriz Identidad, la posición previa tambien será 0
            Jw_a(:,k)=[0,0,1];%Si no hay matriz de rotación previa se obtiene la Matriz identidad
         end
     else
%         %Para las juntas prismáticas
        try
            Jv_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)=[0,0,1];
        end
            Jw_a(:,k)=[0,0,0];
     end
 end    
``` 
###### Robot 2GDL Rotacional
``` matlab
Jv_a(:,GDL)=PO(:,:,GDL);
Jw_a(:,GDL)=PO(:,:,GDL);
for k= 1:GDL
    if RP(k)==0 
       %Para las juntas de revolución
        try
            Jv_a(:,k)= cross(RO(:,3,k-1), PO(:,:,GDL)-PO(:,:,k-1));
            Jw_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)= cross([0,0,1], PO(:,:,GDL));%Matriz de rotación de 0 con respecto a 0 es la Matriz Identidad, la posición previa tambien será 0
            Jw_a(:,k)=[0,0,1];%Si no hay matriz de rotación previa se obtiene la Matriz identidad
         end
     else
%         %Para las juntas prismáticas
        try
            Jv_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)=[0,0,1];
        end
            Jw_a(:,k)=[0,0,0];
     end
 end    
``` 
###### Robot 3GDL Cartesiano
``` matlab
Jv_a(:,GDL)=PO(:,:,GDL);
Jw_a(:,GDL)=PO(:,:,GDL);
for k= 1:GDL
    if RP(k)==0 
       %Para las juntas de revolución
        try
            Jv_a(:,k)= cross(RO(:,3,k-1), PO(:,:,GDL)-PO(:,:,k-1));
            Jw_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)= cross([0,0,1], PO(:,:,GDL));%Matriz de rotación de 0 con respecto a 0 es la Matriz Identidad, la posición previa tambien será 0
            Jw_a(:,k)=[0,0,1];%Si no hay matriz de rotación previa se obtiene la Matriz identidad
         end
     else
        %Para las juntas prismáticas
        try
            Jv_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)=[0,0,1];
        end
            Jw_a(:,k)=[0,0,0];
     end
 end    
 ``` 
 Se crean las submatrices de Jacobianos y posteriormente una matriz completa.
   ###### Robot 1GDL Péndulo
``` matlab
%Obtenemos SubMatrices de Jacobianos
Jv_a= simplify (Jv_a);
Jw_a= simplify (Jw_a);

%Matriz de Jacobiano Completa
Jac= [Jv_a;
      Jw_a];
Jacobiano= simplify(Jac);
``` 
###### Robot 2GDL Rotacional
``` matlab
%SubMatrices de Jacobianos
Jv_a= simplify (Jv_a);
Jw_a= simplify (Jw_a);

%Matriz de Jacobiano Completa
Jac= [Jv_a;
      Jw_a];
Jacobiano= simplify(Jac);
``` 
###### Robot 3GDL Cartesiano
``` matlab
%SubMatrices de Jacobianos
Jv_a= simplify (Jv_a);
Jw_a= simplify (Jw_a);
%Matriz de Jacobiano Completa
Jac= [Jv_a;
      Jw_a];
Jacobiano= simplify(Jac);
 ``` 
Se obtienen los vectores de Velocidades Lineales y Angulares mediante el Jacobiano Lineal y Ángular.
   ###### Robot 1GDL Péndulo
``` matlab
V=simplify (Jv_a*Qp);
W=simplify (Jw_a*Qp);
``` 
###### Robot 2GDL Rotacional
``` matlab
V=simplify (Jv_a*Qp);
W=simplify (Jw_a*Qp);
``` 
###### Robot 3GDL Cartesiano
``` matlab
V=simplify (Jv_a*Qp);
W=simplify (Jw_a*Qp);
 ``` 
##### Energía Cinética

Se establece la distancia del origen del eslabón a su centro de masa vectores de posición respecto al centro de masa, dependiedno del robot analizado.
###### Robot 1GDL Péndulo
``` matlab
P01=subs(P(:,:,1)/2, l1, lc1);
``` 
###### Robot 2GDL Rotacional
``` matlab
P01=subs(P(:,:,1)/2, l1, lc1);
P12=subs(P(:,:,2)/2, l2, lc2);
``` 
###### Robot 3GDL Cartesiano
``` matlab
P01=subs(P(:,:,1)/2, l1, lc1);
P12=subs(P(:,:,2)/2, l2, lc2);
P23=subs(P(:,:,3)/2, l3, lc3);
 ``` 
Se crean las matrices de inercia para cada eslabón, la cantidad de eslabones a implementar dependerá del robot analizado.
###### Robot 1GDL Péndulo
``` matlab
I1=[Ixx1 0 0; 
    0 Iyy1 0; 
    0 0 Izz1];
``` 
###### Robot 2GDL Rotacional
``` matlab
I1=[Ixx1 0 0; 
    0 Iyy1 0; 
    0 0 Izz1];

I2=[Ixx2 0 0; 
    0 Iyy2 0; 
    0 0 Izz2];
``` 
###### Robot 3GDL Cartesiano
``` matlab
I1=[Ixx1 0 0; 
    0 Iyy1 0; 
    0 0 Izz1];
    
I2=[Ixx2 0 0; 
    0 Iyy2 0; 
    0 0 Izz2];
    
I3=[Ixx3 0 0; 
    0 Iyy3 0; 
    0 0 Izz3];
 ``` 
###### Función de energía cinética
En este punto se extraen las velocidades lineales en cada eje al igual que las velocidades angular en cada ángulo de Euler, dependiendo del robot analizado.
###### Robot 1GDL Péndulo
``` matlab
%velocidades lineales en cada eje
V=V(t);
Vx= V(1,1);
Vy= V(2,1);
Vz= V(3,1);

%velocidades angular en cada ángulo de Euler
W=W(t);
W_pitch= W(1,1);
W_roll= W(2,1);
W_yaw= W(3,1);
``` 
###### Robot 2GDL Rotacional
``` matlab
%velocidades lineales en cada eje
V=V(t);
Vx= V(1,1);
Vy= V(2,1);
Vz= V(3,1);

%velocidades angular en cada ángulo de Euler
W=W(t);
W_pitch= W(1,1);
W_roll= W(2,1);
W_yaw= W(3,1);
``` 
###### Robot 3GDL Cartesiano
``` matlab
%velocidades lineales en cada eje
V=V(t);
Vx= V(1,1);
Vy= V(2,1);
Vz= V(3,1);

%velocidades angular en cada ángulo de Euler
W=W(t);
W_pitch= W(1,1);
W_roll= W(2,1);
W_yaw= W(3,1);
 ```
###### Velocidades para cada eslabón.
Esto dependerá de la configuración del robot a analizar, en este caso para el primer robot solo sera necesario conseguir la matriz completa de Jacobiano mientras que en el segundo y tercer robot será necesario hacer un proceso mucho más extenso.
###### Robot 1GDL Péndulo
``` matlab
%Matriz de Jacobiano Completa
Jac1= [Jv_a;
      Jw_a];
Jacobiano1= simplify(Jac1);
```
###### Robot 2GDL Rotacional
``` matlab
%Eslabón 1
%Jacobiano lineal de forma analítica
Jv_a1(:,GDL-1)=PO(:,:,GDL-1);
Jw_a1(:,GDL-1)=PO(:,:,GDL-1);
for k= 1:GDL-1
    if RP(k)==0 
       %Para las juntas de revolución
        try
            Jv_a1(:,k)= cross(RO(:,3,k-1), PO(:,:,GDL-1)-PO(:,:,k-1));
            Jw_a1(:,k)= RO(:,3,k-1);
        catch
            Jv_a1(:,k)= cross([0,0,1], PO(:,:,GDL-1));%Matriz de rotación de 0 con respecto a 0 es la Matriz Identidad, la posición previa tambien será 0
            Jw_a1(:,k)=[0,0,1];%Si no hay matriz de rotación previa se obtiene la Matriz identidad
         end
     else
%Para las juntas prismáticas
        try
            Jv_a1(:,k)= RO(:,3,k-1);
        catch
            Jv_a1(:,k)=[0,0,1];
        end
            Jw_a1(:,k)=[0,0,0];
     end
 end    
%SubMatrices de Jacobianos
Jv_a1= simplify (Jv_a1); %Jacobiano lineal obtenido de forma analítica
Jw_a1= simplify (Jw_a1);%Jacobiano ángular obtenido de forma analítica

%Matriz de Jacobiano Completa
Jac1= [Jv_a1;
      Jw_a1];
Jacobiano1= simplify(Jac1);
``` 
###### Robot 3GDL Cartesiano
``` matlab
%Eslabón 1
%jacobiano lineal de forma analítica
Jv_a1(:,GDL-2)=PO(:,:,GDL-2);
Jw_a1(:,GDL-2)=PO(:,:,GDL-2);
for k= 1:GDL-2
    if RP(k)==0 
       %Para las juntas de revolución
        try
            Jv_a1(:,k)= cross(RO(:,3,k-1), PO(:,:,GDL-2)-PO(:,:,k-1));
            Jw_a1(:,k)= RO(:,3,k-1);
        catch
            Jv_a1(:,k)= cross([0,0,1], PO(:,:,GDL-2));%Matriz de rotación de 0 con respecto a 0 es la Matriz Identidad, la posición previa tambien será 0
            Jw_a1(:,k)=[0,0,1];%Si no hay matriz de rotación previa se obtiene la Matriz identidad
         end
     else
%         %Para las juntas prismáticas
        try
            Jv_a1(:,k)= RO(:,3,k-1);
        catch
            Jv_a1(:,k)=[0,0,1];
        end
            Jw_a1(:,k)=[0,0,0];
     end
 end    
%SubMatrices de Jacobianos
Jv_a1= simplify (Jv_a1);%Jacobiano lineal obtenido de forma analítica
Jw_a1= simplify (Jw_a1);%Jacobiano ángular obtenido de forma analítica
%Matriz de Jacobiano Completa
Jac1= [Jv_a1;
      Jw_a1];
Jacobiano1= simplify(Jac1);

%Eslabón 2
%Calculamos el jacobiano lineal de forma analítica
Jv_a2(:,GDL-1)=PO(:,:,GDL-1);
Jw_a2(:,GDL-1)=PO(:,:,GDL-1);
for k= 1:GDL-1
    if RP(k)==0 
       %Para las juntas de revolución
        try
            Jv_a2(:,k)= cross(RO(:,3,k-1), PO(:,:,GDL-1)-PO(:,:,k-1));
            Jw_a2(:,k)= RO(:,3,k-1);
        catch
            Jv_a2(:,k)= cross([0,0,1], PO(:,:,GDL-1));%Matriz de rotación de 0 con respecto a 0 es la Matriz Identidad, la posición previa tambien será 0
            Jw_a2(:,k)=[0,0,1];%Si no hay matriz de rotación previa se obtiene la Matriz identidad
         end
     else
%         %Para las juntas prismáticas
        try
            Jv_a2(:,k)= RO(:,3,k-1);
        catch
            Jv_a2(:,k)=[0,0,1];
        end
            Jw_a2(:,k)=[0,0,0];
     end
 end    
%SubMatrices de Jacobianos
Jv_a2= simplify (Jv_a2);%Jacobiano lineal obtenido de forma analítica
Jw_a2= simplify (Jw_a2);%Jacobiano ángular obtenido de forma analítica
%Matriz de Jacobiano Completa
Jac2= [Jv_a2;
      Jw_a2];
Jacobiano2= simplify(Jac2);
 ```

Se obtienen los vectores de Velocidades Lineales y Angulares.
###### Robot 1GDL Péndulo
``` matlab
 %Velocidad lineal obtenida mediante el Jacobiano lineal del Eslabón 1
Qp=Qp(t);
V1=simplify (Jv_a*Qp(1));
 %Velocidad angular obtenida mediante el Jacobiano angular del Eslabón 1
W1=simplify (Jw_a*Qp(1));
```
###### Robot 2GDL Rotacional
``` matlab
 %Velocidad lineal obtenida mediante el Jacobiano lineal del Eslabón 1
Qp=Qp(t);
V1=simplify (Jv_a1*Qp(1));
 %Velocidad angular obtenida mediante el Jacobiano angular del Eslabón 1
W1=simplify (Jw_a1*Qp(1));
``` 
###### Robot 3GDL Cartesiano
``` matlab
%Velocidad lineal obtenida mediante el Jacobiano lineal del Eslabón 1
Qp=Qp(t);
V1=simplify (Jv_a1*Qp(1));
%Velocidad angular obtenida mediante el Jacobiano angular del Eslabón 1
W1=simplify (Jw_a1*Qp(1));

%Velocidad lineal obtenida mediante el Jacobiano lineal del Eslabón 2
V2=simplify (Jv_a2*Qp(1:2));
%Velocidad angular obtenida mediante el Jacobiano angular del Eslabón 2
W2=simplify (Jw_a2*Qp(1:2));
 ```
Posteriormente se realiza el análisis con respecto a la cantidad de eslabones de cada uno de los robots presentados.
###### Robot 1GDL Péndulo
``` matlab
%Eslabón 1
V1_Total= V1+cross(W1,P01);
K1= (1/2*m1*(V1_Total))'*(1/2*m1*(V1_Total)) + (1/2*W1)'*(I1*W1);
%Energía Cinética en el Eslabón 1
K1= simplify (K1);

K_Total= simplify (K1);
```
###### Robot 2GDL Rotacional
``` matlab
%Eslabón 1
V1_Total= V1+cross(W1,P01);
K1= (1/2*m1*(V1_Total))'*(1/2*m1*(V1_Total)) + (1/2*W1)'*(I1*W1);
%Energía Cinética en el Eslabón 1
K1= simplify (K1);

%Eslabón 2
V2_Total= V1+cross(W1,P01);
K2= (1/2*m1*(V2_Total))'*(1/2*m1*(V2_Total)) + (1/2*W1)'*(I1*W1);
%Energía Cinética en el Eslabón 2
K2= simplify (K2);

K_Total= simplify (K1+K2);
``` 
###### Robot 3GDL Cartesiano
``` matlab
%Eslabón 1
V1_Total= V1+cross(W1,P01);
K1= (1/2*m1*(V1_Total))'*(1/2*m1*(V1_Total)) + (1/2*W1)'*(I1*W1);
%Energía Cinética en el Eslabón 1
K1= simplify (K1);

%Eslabón 2
V2_Total= V2+cross(W2,P12);
K2= (1/2*m2*(V2_Total))'*(1/2*m2*(V2_Total)) + (1/2*W2)'*(I2*W2);
%Energía Cinética en el Eslabón 2
K2= simplify (K2);

%Eslabón 3
V3_Total= V+cross(W,P23);
K3= (1/2*m3*(V3_Total))'*(1/2*m3*(V3_Total)) + (1/2*W)'*(I3*W);
%Energía Cinética en el Eslabón 3
K3= simplify (K3);

K_Total= simplify (K1+K2+K3);
 ```
Se calcula la energía potencial para cada uno de los eslabones considerados (dependerá del robot analizado).
###### Robot 1GDL Péndulo
``` matlab
%Alturas respecto a la gravedad
 h1= P01(2); %Tomo la altura paralela al eje y

 U1=m1*g*h1;
```
###### Robot 2GDL Rotacional
``` matlab
%Alturas respecto a la gravedad
 h1= P01(2); %Tomo la altura paralela al eje y
 h2= P12(2); %Tomo la altura paralela al eje y

 U1=m1*g*h1;
 U2=m2*g*h2;
``` 
###### Robot 3GDL Cartesiano
``` matlab
%Alturas respecto a la gravedad
 U1=m1*g*h1;
 U2=m2*g*l2;
 ```
 
Se calcula la energía potencial potencial total con respecto a las configuraciones que varían para cada robot.
###### Robot 1GDL Péndulo
``` matlab
%Energía potencial total
 U_Total= U1
```
###### Robot 2GDL Rotacional
``` matlab
%Energía potencial total
 U_Total= U1 + U2
``` 
###### Robot 3GDL Cartesiano
``` matlab
%Energía potencial total
 U_Total= U1 + U2
 ```
 Se obtiene el langrangiano con respecto a las configuraciones que varían para cada robot.
 ###### Robot 1GDL Péndulo
``` matlab
 Lagrangiano= simplify (K_Total-U_Total);
 pretty (Lagrangiano);

 H= simplify (K_Total+U_Total);
  pretty (H)
```
###### Robot 2GDL Rotacional
``` matlab
 Lagrangiano= simplify (K_Total-U_Total);
 pretty (Lagrangiano);

 H= simplify (K_Total+U_Total);
  pretty (H)
``` 
###### Robot 3GDL Cartesiano
``` matlab
%Energía potencial total
 U_Total= U1 + U2
 ```
Posteriormente se obtiene la energía potencial para cada uno de los eslabones con respecto a las configuraciones que varían para cada robot, esta isntrucción solo se ejecuta para el segundo y tercer robot.
###### Robot 2GDL Rotacional
``` matlab
 %Alturas respecto a la gravedad
 h1= P01(2); %Tomo la altura paralela al eje z
 h2= P12(2); %Tomo la altura paralela al eje y

 U1=m1*g*h1;
 U2=m2*g*h2;
``` 
###### Robot 3GDL Cartesiano
``` matlab
%Alturas respecto a la gravedad
 U1=m1*g*h1;
 U2=m2*g*l2;
 ```
Se obtiene la energía potencial total con respecto a las configuraciones que varían para cada robot, esta isntrucción solo se ejecuta para el segundo y tercer robot.
###### Robot 2GDL Rotacional
``` matlab
U_Total= U1 + U2
``` 
###### Robot 3GDL Cartesiano
``` matlab
U_Total= U1 + U2
 ```
Se obtiene el Lagrangiano con respecto a las configuraciones que varían para cada robot, esta isntrucción solo se ejecuta para el segundo y tercer robot.
###### Robot 2GDL Rotacional
 Lagrangiano= simplify (K_Total-U_Total);

 H= simplify (K_Total+U_Total);
  pretty (H)
``` 
###### Robot 3GDL Cartesiano
``` matlab
 Lagrangiano= simplify (K_Total-U_Total);

 H= simplify (K_Total+U_Total);
  pretty (H)
 ```

Para finalizar se cierra con la instrucción toc, la cual nos ayuda a calcular el tiempo de ejecución de los archivos, cada código cuenta con esta misma instrucción
``` matlab
toc
 ```

## Resultados:
### Robot 1GDL Péndulo
![image](https://user-images.githubusercontent.com/99983026/224862443-84d53271-a20b-4ffd-ae03-f0c690d9e460.png)

### Robot 2GDL Rotacional
![image](https://user-images.githubusercontent.com/99983026/224864155-b7cc014d-f936-4e82-b020-12f0088447be.png)

### Robot 3GDL Cartesiano
![image](https://user-images.githubusercontent.com/99983026/224864208-7689bf35-b4b3-4435-9440-8710541aca67.png)
