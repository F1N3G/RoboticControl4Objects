clc; clear; close all;

% === Parametri braț robotic ===
L1 = 5;
L2 = 3;

% === Traiectorie normală ===
x_norm = linspace(2, 6, 100);
y_norm = linspace(3, 5, 100);

% === Traiectorie optimizată prin Least Squares ===
x_data = [2, 3, 4, 5, 6]';
y_data = [3, 3.5, 4.1, 4.6, 5]';
A = [x_data, ones(size(x_data))];
coef = (A' * A) \ (A' * y_data);
a = coef(1); b = coef(2);
x_opt1 = linspace(min(x_data), max(x_data), 100);
y_opt1 = a * x_opt1 + b;

% === Traiectorie optimizată prin DFP (minimizăm variația unghiurilor) ===
% Funcția obiectivă: f(x) = sum((theta(i+1) - theta(i))^2)
% pentru unghiurile calculate din punctele traiectoriei

% Definim punctele brute
x_init = linspace(2, 6, 10);
y_init = linspace(3, 5, 10);

% Vectorul de optimizat: concatenare [x1,y1,x2,y2,...]
x0 = [x_init; y_init];
x0 = x0(:); % vector coloană

% Funcție cost: minimizarea variațiilor de unghi
fun = @(v) cost_variatie_unghiuri(v, L1, L2);

% Implementare simplificată DFP
options = optimoptions('fminunc', 'Algorithm', 'quasi-newton', 'Display', 'none');
sol = fminunc(fun, x0, options);

% Reconstruim traiectoria optimizată
sol = reshape(sol, 2, []);
x_opt2 = sol(1, :);
y_opt2 = sol(2, :);

% === Funcție generală pentru animație ===
function animeaza_brat(x_traj, y_traj, L1, L2, titlu)
    figure; axis([-10 10 -10 10]);
    hold on; grid on;
    xlabel('X'); ylabel('Y'); title(titlu);

    h1 = plot([0, 0], [0, 0], 'ro-', 'LineWidth', 3);
    h2 = plot([0, 0], [0, 0], 'bo-', 'LineWidth', 3);
    base = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    joint1 = plot(0, 0, 'ko', 'MarkerFaceColor', 'g');
    joint2 = plot(0, 0, 'ko', 'MarkerFaceColor', 'b');

    for i = 1:length(x_traj)
        xe = x_traj(i); ye = y_traj(i);
        c2 = (xe^2 + ye^2 - L1^2 - L2^2) / (2 * L1 * L2);
        if abs(c2) > 1
            continue;
        end
        s2 = sqrt(1 - c2^2);
        theta2 = atan2(s2, c2);
        k1 = L1 + L2 * cos(theta2);
        k2 = L2 * sin(theta2);
        theta1 = atan2(ye, xe) - atan2(k2, k1);

        x1 = L1 * cos(theta1); y1 = L1 * sin(theta1);
        x2 = x1 + L2 * cos(theta1 + theta2);
        y2 = y1 + L2 * sin(theta1 + theta2);

        set(h1, 'XData', [0 x1], 'YData', [0 y1]);
        set(h2, 'XData', [x1 x2], 'YData', [y1 y2]);
        set(joint1, 'XData', x1, 'YData', y1);
        set(joint2, 'XData', x2, 'YData', y2);

        pause(0.05);
    end
    hold off;
end

% === Funcție cost pentru DFP ===
function J = cost_variatie_unghiuri(v, L1, L2)
    p = reshape(v, 2, []);
    x = p(1, :);
    y = p(2, :);
    theta = zeros(1, length(x));
    for i = 1:length(x)
        xe = x(i); ye = y(i);
        c2 = (xe^2 + ye^2 - L1^2 - L2^2) / (2 * L1 * L2);
        if abs(c2) > 1
            J = inf; return;
        end
        s2 = sqrt(1 - c2^2);
        theta2 = atan2(s2, c2);
        k1 = L1 + L2 * cos(theta2);
        k2 = L2 * sin(theta2);
        theta1 = atan2(ye, xe) - atan2(k2, k1);
        theta(i) = theta1;
    end
    J = sum(diff(theta).^2); % variație totală a unghiurilor
end

% === Rularea animațiilor ===
animeaza_brat(x_norm, y_norm, L1, L2, 'Traiectorie Cinematica Inversa (normală)');
pause(2);
animeaza_brat(x_opt1, y_opt1, L1, L2, 'Traiectorie Optimizată (Least Squares)');
pause(2);
animeaza_brat(x_opt2, y_opt2, L1, L2, 'Traiectorie Optimizată (DFP - Quasi-Newton)');
% aici avem codul pentru 3 traiectorii ale bratului una normala si 2 optimizate cu LeastSquare si Quasi-Newton
