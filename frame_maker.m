function [vecdir_unit,vecper_unit] = frame_maker(wpcurr,wpnext)

% Calcular el vector que apunta desde el punto1 al punto2
vector_director = wpnext - wpcurr;

% Calcular un vector perpendicular intercambiando las componentes X e Y y cambiando el signo de una de ellas
vector_perpendicular = [-vector_director(2), vector_director(1)];

vecdir_unit = vector_director/norm(vector_director);
vecper_unit = vector_perpendicular/norm(vector_perpendicular);

end