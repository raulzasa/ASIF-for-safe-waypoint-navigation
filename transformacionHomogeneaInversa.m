function P_local = transformacionHomogeneaInversa(P_global, A1, A2)
    % Calcular la dirección de la recta en radianes
    theta_recta = atan2(A2(2) - A1(2), A2(1) - A1(1));

    % Construir la matriz de rotación 2x2
    R = [cos(theta_recta), -sin(theta_recta); 
         sin(theta_recta), cos(theta_recta)];

    % Construir la matriz de translación 2x1
    T = A1;
    if size(T,1)==1
        T=A1';
    end
    % Construir la matriz de transformación homogénea 3x3
    T_homogenea = [R, T; 0, 0, 1];

    % Calcular la inversa de la matriz de transformación homogénea
    T_homogenea_inversa = inv(T_homogenea);

    % Extraer las coordenadas x, y y la orientación theta de P_global
    x_global = P_global(1);
    y_global = P_global(2);
    theta_global = P_global(3);

    % Aplicar la inversa de la transformación homogénea para obtener las coordenadas locales
    P_local_homogeneo = T_homogenea_inversa * [x_global; y_global; 1];

    % Extraer las coordenadas x_local, y_local y calcular la orientación theta_local
    x_local = P_local_homogeneo(1);
    y_local = P_local_homogeneo(2);
    theta_local = atan2(sin(theta_global - theta_recta),cos(theta_global - theta_recta));
    P_local = [x_local,y_local,theta_local];
    % Devolver las coordenadas locales
end

