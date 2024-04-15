function result = intervalIntersection(interval1, interval2)
    % Obtener los extremos de los intervalos
    start1 = interval1(1, 1);
    end1 = interval1(1, 2);
    start2 = interval2(1, 1);
    end2 = interval2(1, 2);
    
    % Calcular los extremos del intervalo intersección
    startIntersect = max(start1, start2);
    endIntersect = min(end1, end2);
    
    % Verificar si hay intersección o si el intervalo es inválido
    if startIntersect > endIntersect
        result = [];
    else
        result = [startIntersect, endIntersect];
    end
end
