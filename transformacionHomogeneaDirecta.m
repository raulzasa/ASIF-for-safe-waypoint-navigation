function P_global = transformacionHomogeneaDirecta(P_local, A1, A2)
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

    % Agregar una fila a P_local para hacerla homogénea (3x1)
    P_local_homogeneo = [P_local(1:2)'; 1];

    % Aplicar la transformación homogénea directa
    P_global_homogeneo = T_homogenea * P_local_homogeneo;

    % Extraer las coordenadas x, y y la orientación theta
    x_global = P_global_homogeneo(1);
    y_global = P_global_homogeneo(2);
    % theta_global = P_local(3) + theta_recta;
    theta_global = atan2(sin(P_local(3) + theta_recta),cos(P_local(3) + theta_recta));


    % Devolver las coordenadas globales
    P_global = [x_global; y_global; theta_global];
end
