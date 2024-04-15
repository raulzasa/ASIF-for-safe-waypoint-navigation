function vertices = calcularVerticesPoligonoRegular(R, C, N)
    if N < 3
        error('Un polígono regular debe tener al menos 3 lados.');
    end

    % Calcular el ángulo entre cada vértice
    angulo = 2 * pi / N;

    % Inicializar el vector de vértices
    vertices = zeros(N, 2);

    % Calcular las coordenadas de los vértices
    for i = 1:N
        x = R * cos(angulo * (i - 2)) + C(1);
        y = R * sin(angulo * (i - 2)) + C(2);
        vertices(i, :) = [x, y];
    end

     % Asegurarse de que el lado inferior sea horizontal
    diferencia_x = vertices(1, 1) - vertices(2, 1);
    diferencia_y = vertices(1, 2) - vertices(2, 2);

    angulo_rotacion = atan2(diferencia_y, diferencia_x);

    vertices = rotarVertices(vertices, -angulo_rotacion);

    function vertices_rotados = rotarVertices(vertices, angulo)
        matriz_rotacion = [cos(angulo), -sin(angulo); sin(angulo), cos(angulo)];
        vertices_rotados = (matriz_rotacion * vertices.').';
    end
end
