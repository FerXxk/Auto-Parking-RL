function [accel_steer, debugInfo] = pure_pursuit_control(egoPose, ReferencePose)
    % --- 1. PARÁMETROS BASE ---
    L = 2.8;
    v_ref = 2.0;

    % --- 2. TRASLACIÓN AL EJE TRASERO ---
    dist_to_rear = L / 2;
    rear_x = egoPose(1) - dist_to_rear * cos(egoPose(3));
    rear_y = egoPose(2) - dist_to_rear * sin(egoPose(3));

    % --- 3. PUNTO MÁS CERCANO ---
    distancias = sqrt((ReferencePose(:,1) - rear_x).^2 + ...
                      (ReferencePose(:,2) - rear_y).^2);
    [~, idx_closest] = min(distancias);

    % --- 4. CURVATURA ANTICIPADA (ventana solo hacia adelante) ---
    % Medimos el cambio de heading entre dos puntos futuros, no incluimos
    % nada del pasado. Esto da anticipación real al inicio de la curva.
    N = size(ReferencePose,1);
    idx_near   = min(idx_closest + 5,  N);
    idx_mid    = min(idx_closest + 15, N);
    idx_far    = min(idx_closest + 30, N);

    % Curvatura entre el punto cercano-futuro y el medio-futuro
    diff_1 = ReferencePose(idx_mid, 3) - ReferencePose(idx_near, 3);
    diff_1 = abs(atan2(sin(diff_1), cos(diff_1)));

    % Curvatura entre el punto medio-futuro y el lejano-futuro
    diff_2 = ReferencePose(idx_far, 3) - ReferencePose(idx_mid, 3);
    diff_2 = abs(atan2(sin(diff_2), cos(diff_2)));

    % Tomamos la máxima: si CUALQUIERA de las dos zonas tiene curva,
    % activamos modo curva. Esto anticipa antes y mantiene Ld bajo
    % durante toda la curva (entrada y salida).
    diff_path = max(diff_1, diff_2);

    % --- 5. Ld ADAPTATIVO CON TRANSICIÓN NO LINEAL ---
    % Usamos sqrt() para que Ld baje rápido en cuanto hay un poco de
    % curvatura, en lugar de hacerlo linealmente.
    curvatura_norm = min(1.0, diff_path / 0.30);
    curvatura_norm = sqrt(curvatura_norm);   % transición agresiva

    Ld_recta = 14.0;
    Ld_curva = 1.5;
    Ld = Ld_recta - curvatura_norm * (Ld_recta - Ld_curva);

    % Suelo mínimo
    Ld = max(Ld, 3.0);

    % --- 6. SELECCIÓN DEL PUNTO OBJETIVO ---
    target_idx = N;
    for i = idx_closest:N
        if distancias(i) >= Ld
            target_idx = i;
            break;
        end
    end
    target_pt = ReferencePose(target_idx, :);

    % --- 7. ALPHA ---
    dx = target_pt(1) - rear_x;
    dy = target_pt(2) - rear_y;
    alpha = atan2(dy, dx) - egoPose(3);
    alpha = atan2(sin(alpha), cos(alpha));

    % --- 8. LEY DE PURE PURSUIT ---
    delta = atan2(2 * L * sin(alpha), Ld);

    % --- 9. SATURACIÓN Y SALIDA ---
    u = [v_ref; max(-0.6, min(0.6, delta))];
    accel_steer = u;
    debugInfo = Ld;
end