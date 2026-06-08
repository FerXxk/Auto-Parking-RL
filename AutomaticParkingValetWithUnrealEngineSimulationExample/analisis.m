%% =======================================================================
%  ANALISIS DE METRICAS DEL CONTROLADOR EN EJECUCION
%  Mide el error de seguimiento contra la referencia real del planner.
%
%  FLUJO DE USO:
%    1. En Simulink, selecciona A MANO el controlador que quieres analizar
%       (Pure Pursuit, Stanley, MPC, LQR...).
%    2. Pon su nombre en la variable 'nombreCtrl' del PASO 2.
%    3. Ejecuta este script por SECCIONES (Ctrl+Enter).
%    4. Los resultados se guardan en un .mat; repite con otro controlador
%       cuando quieras y se iran acumulando.
%
%  SEÑALES NECESARIAS (capturadas con bloques "To Workspace",
%  Save format = Timeseries):
%    egoPose      : pose del vehiculo que entra al controlador
%    reference    : salida del bloque "Reference Preview from planner"
%    accel_steer  : comando de control del controlador        (opcional)
%    isParking    : señal del Vehicle Mode Selector            (opcional)
% =======================================================================


%% PASO 1 - SIMULAR EL MODELO CON EL CONTROLADOR SELECCIONADO
% -------------------------------------------------------------------------
% IMPORTANTE: el modelo NO se puede simular en aislamiento. Su InitFcn
% necesita variables de setup. Si el workspace esta vacio, ejecuta antes
% el script oficial RL_Parking_And_Control.m completo.
% -------------------------------------------------------------------------
mdl = "rlAutoParkingValet3D";

% --- Verificar / preparar el setup minimo que necesita el InitFcn ---
if ~exist('doTraining','var')
    warning(['El workspace no tiene las variables de setup del modelo. ', ...
             'Preparando setup minimo. Si falla, ejecuta primero el ', ...
             'script oficial RL_Parking_And_Control.m completo.']);

    Ts = 0.1;
    Tf = 50;
    doTraining = false;            %#ok<NASGU>
    freeSpotIndex = 18;            %#ok<NASGU>
    searchDist = 10;               %#ok<NASGU>

    parkingLot = ParkingLotManager;
    xRef = parkingLot.createReferenceTrajectory(Ts,Tf); %#ok<NASGU>
    info = parkingLot.getInfo();   %#ok<NASGU>

    vehiclePose = [10 -32.5 pi];   %#ok<NASGU>

    autoParkingValetParams3D;
    setupActorVehicles(mdl,freeSpotIndex);
end

% Activar el logging de señales del modelo
set_param(mdl, 'SignalLogging', 'on');
set_param(mdl, 'SignalLoggingName', 'logsout');

% --- CONFIGURACION DE LA PRUEBA DE COSTE ---
% Nombre del bloque/subsistema del controlador, tal y como aparece en el
% informe del Profiler. Se usa solo para recordarte que bloque buscar.
% Ajusta segun el controlador activo: "Pure Pursuit", "Stanley", "MPC"...
nombreBloqueCtrl = 'Stanley';   % <-- AJUSTAR

% RECOMENDACION: para una comparacion de coste justa, fija la MISMA v_ref
% en todos los controladores antes de simular.

% --- Simular con el Simulink Profiler activado ---
% En R2024b la lectura del Profiler por codigo no tiene una API fiable, asi
% que se activa el profiler, se simula, y se abre el informe para que leas
% el "Total Time" del bloque del controlador a mano (un solo numero).
fprintf('Simulando con el Profiler activado...\n');
set_param(mdl, 'Profile', 'on');
out = sim(mdl);
set_param(mdl, 'Profile', 'off');
fprintf('Simulacion completada. Abriendo informe del Profiler...\n');

% Abrir el informe del Profiler
try
    Simulink.profiler.show(mdl);
catch
    try
        profile viewer;   %#ok<PROF>
    catch
        warning(['No se pudo abrir el informe automaticamente. Abrelo ', ...
                 'desde la pestaña Debug -> Performance -> Profiler.']);
    end
end

% Extraccion robusta de las señales registradas
S = extraerSenales(out);

% Numero de pasos de la simulacion (para normalizar el coste).
% Se obtiene de la propia señal de pose: cada muestra es un paso.
n_pasos = numel(S.pose.Time);

fprintf('\n>>> ACCION MANUAL REQUERIDA <<<\n');
fprintf('En el informe del Profiler, busca el bloque "%s"\n', nombreBloqueCtrl);
fprintf('y anota su valor de "Total Time" (en segundos).\n');
fprintf('Numero de pasos de esta simulacion: %d\n', n_pasos);
fprintf('Luego ve al PASO 2 e introduce ese valor en "total_time_profiler".\n\n');


%% PASO 2 - ANALIZAR LAS METRICAS DEL CONTROLADOR
% -------------------------------------------------------------------------
% Pon aqui el nombre del controlador que has seleccionado en Simulink.
nombreCtrl = nombreBloqueCtrl;   % <-- CAMBIA esto segun el controlador activo

% --- Coste computacional: introduce el "Total Time" leido del Profiler ---
% Es el unico dato manual. El tiempo POR PASO (independiente de la
% velocidad del vehiculo) se calcula automaticamente dividiendo por n_pasos.
total_time_profiler = NaN;   % <-- INTRODUCE aqui el Total Time del bloque [s]

% t_fin_aprox: instante de fin de aproximacion. Automatico si hay isParking.
t_fin_aprox = S.tParking;      % automatico
% t_fin_aprox = 38;            % <-- alternativa manual
% t_fin_aprox = [];            % <-- analizar toda la simulacion

m = analizarControlador(S, t_fin_aprox, nombreCtrl);

% --- Añadir las metricas de coste computacional ---
m.coste_total = total_time_profiler;          % tiempo total del bloque [s]
m.n_pasos     = n_pasos;                       % numero de pasos
if ~isnan(total_time_profiler) && n_pasos > 0
    m.coste_por_paso = total_time_profiler / n_pasos;   % <- medida JUSTA
    fprintf('  Coste total del controlador : %.4f s\n', m.coste_total);
    fprintf('  Numero de pasos             : %d\n', m.n_pasos);
    fprintf('  Coste POR PASO              : %.3e s (%.2f us)\n', ...
            m.coste_por_paso, m.coste_por_paso*1e6);
else
    m.coste_por_paso = NaN;
    warning(['Coste no calculado: introduce "total_time_profiler" con el ', ...
             'valor leido del informe del Profiler y vuelve a ejecutar ', ...
             'este PASO 2.']);
end


%% PASO 3 - GUARDAR LOS RESULTADOS DE ESTE CONTROLADOR
% -------------------------------------------------------------------------
% Acumula los resultados en 'resultados_controladores.mat'. Cada vez que
% analices un controlador distinto, se añade (o se actualiza si repites).
archivoRes = 'resultados_controladores.mat';

if isfile(archivoRes)
    R = load(archivoRes);
    resultados = R.resultados;
else
    resultados = struct();
end

% Nombre de campo valido a partir del nombre del controlador
campo = matlab.lang.makeValidName(nombreCtrl);
resultados.(campo) = m;

save(archivoRes, 'resultados');
fprintf('Resultados de "%s" guardados en %s\n', nombreCtrl, archivoRes);
fprintf('Controladores acumulados: %s\n', strjoin(fieldnames(resultados), ', '));


%% PASO 4 (opcional) - COMPARAR TODOS LOS CONTROLADORES GUARDADOS
% -------------------------------------------------------------------------
% Cuando hayas analizado varios controladores (repitiendo PASOS 1-3 con
% cada uno), ejecuta esto para ver la tabla y graficas comparativas.
if isfile(archivoRes)
    R = load(archivoRes);
    compararTodos(R.resultados);
else
    warning('Aun no hay resultados guardados. Ejecuta antes los PASOS 1-3.');
end


%% PASO 5 (opcional) - GUARDAR TODAS LAS GRAFICAS COMO PNG
% -------------------------------------------------------------------------
figs = findobj('Type', 'figure');
for i = 1:numel(figs)
    nombre = sprintf('figura_%02d.png', i);
    exportgraphics(figs(i), nombre, 'Resolution', 200);
    fprintf('Guardada: %s\n', nombre);
end


% =======================================================================
% =====================  FUNCIONES AUXILIARES  =========================
% =======================================================================

function S = extraerSenales(out)
% Extrae de forma robusta las señales de la simulacion. Funciona con DOS
% metodos de captura:
%   (A) Bloques "To Workspace" -> las variables son campos de 'out'
%       (egoPose, reference, accel_steer/steering, isParking).
%   (B) Logging de señales -> las señales estan en un Dataset (logsout).
%
% Devuelve un struct S con:
%   S.pose      : timeseries de la pose del vehiculo
%   S.reference : timeseries de la referencia del planner
%   S.steering  : timeseries del comando de direccion ([] si no existe)
%   S.tParking  : instante de inicio del aparcamiento ([] si no existe)

    S = struct('pose',[], 'reference',[], 'steering',[], 'tParking',[]);

    % --- Listar los campos disponibles en 'out' ---
    if isa(out, 'Simulink.SimulationOutput')
        campos = out.who;
    elseif isstruct(out)
        campos = fieldnames(out);
    else
        campos = {};
    end

    % =====================================================================
    % METODO A: bloques "To Workspace" (campos directos de 'out')
    % =====================================================================
    getCampo = @(patron) buscarCampo(out, campos, patron);

    poseVal = getCampo({'egoPose','egopose','pose'});
    refVal  = getCampo({'reference','refTraj','ref'});
    steerVal= getCampo({'steering','accel_steer','steer','control'});
    parkVal = getCampo({'isParking','isparking','park'});

    if ~isempty(poseVal) && ~isempty(refVal)
        fprintf('Señales leidas via bloques "To Workspace".\n');
        S.pose      = aTimeseries(poseVal);
        S.reference = aTimeseries(refVal);
        if ~isempty(steerVal)
            S.steering = aTimeseries(steerVal);
        else
            warning('Sin señal de direccion: se omiten metricas de esfuerzo.');
        end
        if ~isempty(parkVal)
            pv = aTimeseries(parkVal);
            k  = find(squeeze(pv.Data) > 0.5, 1, 'first');
            if ~isempty(k)
                S.tParking = pv.Time(k);
                fprintf('Fin de aproximacion en t = %.2f s\n', S.tParking);
            end
        end
        return;
    end

    % =====================================================================
    % METODO B: logging de señales (Dataset dentro de 'out')
    % =====================================================================
    logs = [];
    for c = 1:numel(campos)
        try
            val = out.(campos{c});
            if isa(val, 'Simulink.SimulationData.Dataset')
                logs = val;
                fprintf('Logging encontrado en el campo "%s".\n', campos{c});
                break;
            end
        catch
        end
    end

    if isempty(logs) || logs.numElements == 0
        fprintf('\n--- DIAGNOSTICO ---\n');
        fprintf('Campos disponibles en "out": %s\n', strjoin(campos, ', '));
        msg = ['No se encontraron señales.\n', ...
               'OPCION RECOMENDADA: añade bloques "To Workspace" en el ', ...
               'subsistema del controlador, conectados a las señales, con ', ...
               'Variable name = egoPose / reference / accel_steer y ', ...
               'Save format = Timeseries.\n', ...
               'OPCION ALTERNATIVA: activa el logging del modelo con\n', ...
               '  set_param("rlAutoParkingValet3D",''SignalLogging'',''on'')\n', ...
               'y marca los cables (clic derecho -> Log Selected Signals), ', ...
               'luego vuelve a simular.'];
        error(sprintf(msg)); %#ok<SPERR>
    end

    nombres = cell(1, logs.numElements);
    for i = 1:logs.numElements
        nombres{i} = logs.getElement(i).Name;
    end
    fprintf('Señales registradas: %s\n', strjoin(nombres, ', '));

    idx = find(strcmpi(nombres,'egoPose'), 1);
    if isempty(idx), idx = find(contains(lower(nombres),'pose') & ...
                                ~contains(lower(nombres),'target'), 1); end
    if isempty(idx), error('No se encontro la señal de pose ("egoPose").'); end
    S.pose = logs.getElement(idx).Values;

    idx = find(strcmpi(nombres,'reference'), 1);
    if isempty(idx), idx = find(contains(lower(nombres),'ref'), 1); end
    if isempty(idx), error('No se encontro la señal de referencia ("reference").'); end
    S.reference = logs.getElement(idx).Values;

    idx = find(strcmpi(nombres,'steering'), 1);
    if isempty(idx), idx = find(contains(lower(nombres),'steer'), 1); end
    if ~isempty(idx)
        S.steering = logs.getElement(idx).Values;
    else
        warning('Sin señal "steering": se omiten metricas de esfuerzo.');
    end

    idx = find(strcmpi(nombres,'isParking'), 1);
    if isempty(idx), idx = find(contains(lower(nombres),'park'), 1); end
    if ~isempty(idx)
        pv = logs.getElement(idx).Values;
        k  = find(squeeze(pv.Data) > 0.5, 1, 'first');
        if ~isempty(k)
            S.tParking = pv.Time(k);
            fprintf('Fin de aproximacion en t = %.2f s\n', S.tParking);
        end
    end
end


function val = buscarCampo(out, campos, patrones)
% Busca en 'campos' el primero que coincida (exacta o parcialmente) con
% alguno de los 'patrones'. Devuelve el valor del campo, o [] si no hay.
    val = [];
    for p = 1:numel(patrones)
        % coincidencia exacta primero
        idx = find(strcmpi(campos, patrones{p}), 1);
        if isempty(idx)
            idx = find(contains(lower(campos), lower(patrones{p})), 1);
        end
        if ~isempty(idx)
            try
                val = out.(campos{idx});
                return;
            catch
            end
        end
    end
end


function ts = aTimeseries(v)
% Convierte la salida de un bloque To Workspace a timeseries de forma
% robusta (puede venir ya como timeseries, struct-with-time, o array).
    if isa(v, 'timeseries')
        ts = v;
    elseif isstruct(v) && isfield(v,'time') && isfield(v,'signals')
        ts = timeseries(v.signals.values, v.time);
    else
        % array sin tiempo: se crea un eje temporal por indice
        ts = timeseries(v);
    end
end


function pose = extraerPose(poseTS)
% Extrae la matriz de pose Np x 3 [x y theta] de la timeseries de pose.
% En este modelo llega como [3 1 Np] (columna 3x1 por instante).

    d = poseTS.Data;
    if ndims(d) == 3
        % [3 1 Np] o [1 3 Np] -> Np x 3
        d = squeeze(d);          % queda 3 x Np  (o Np x 3)
        if size(d,1) == 3
            d = d.';             % Np x 3
        end
    else
        % 2D: asegurar Np x 3
        if size(d,1) == 3 && size(d,2) ~= 3
            d = d.';
        end
    end
    pose = d;
end


function tramo = extraerTramoRef(refTS, k)
% Extrae el tramo de referencia (preview) del instante k.
% En este modelo 'reference' llega como [Npts 3 Nt]: en cada instante k
% el planner entrega un tramo de Npts puntos [x y theta].
% Devuelve una matriz Npts x 3.

    d = refTS.Data;
    if ndims(d) == 3
        r = d(:,:,k);            % Npts x 3 del instante k
    else
        r = d;                   % referencia constante (caso 2D)
    end
    r = squeeze(r);
    if size(r,2) ~= 3 && size(r,1) == 3
        r = r.';
    end
    tramo = r;
end


function delta = extraerSteering(steerTS)
% Extrae el comando de direccion delta de la timeseries de accel_steer.
% accel_steer llega como [2 1 Nt]: fila 1 = velocidad, fila 2 = direccion.
% Devuelve un vector columna Nt x 1 con SOLO la direccion.

    d = steerTS.Data;
    if ndims(d) == 3
        d = squeeze(d);          % 2 x Nt
        if size(d,1) == 2
            delta = d(2,:).';    % segunda fila = direccion -> columna
        else
            delta = d(:,2);      % segunda columna
        end
    else
        % 2D [Nt 2] o [2 Nt]
        if size(d,2) == 2
            delta = d(:,2);
        else
            delta = d(2,:).';
        end
    end
end


function m = analizarControlador(S, t_fin_aprox, nombre)
% Calcula metricas de seguimiento contra la referencia real del planner.
% La referencia es un PREVIEW DESLIZANTE: en cada instante el planner
% entrega un tramo de varios puntos, distinto en cada paso.

    % --- Pose del vehiculo (Np x 3) ---
    t   = S.pose.Time;
    pos = extraerPose(S.pose);
    tRef = S.reference.Time;

    % --- Recorte a la fase de aproximacion ---
    if ~isempty(t_fin_aprox)
        mask = t <= t_fin_aprox;
    else
        mask = true(size(t));
    end
    t   = t(mask);
    pos = pos(mask, :);
    Np  = size(pos, 1);

    x = pos(:,1);  y = pos(:,2);  theta = pos(:,3);

    % --- Error lateral y de heading instante a instante ---
    % Para cada pose, se toma el tramo de referencia (preview) del MISMO
    % instante de tiempo, se busca el punto mas cercano de ese tramo y se
    % proyecta el error sobre la normal a la ruta en ese punto.
    e_lat     = zeros(Np,1);
    e_heading = zeros(Np,1);
    ruta_ref  = zeros(Np,3);   % reconstruccion de la ruta para graficar
    for k = 1:Np
        % Emparejar el instante de pose con el indice de referencia
        [~, kr] = min(abs(tRef - t(k)));
        tramo = extraerTramoRef(S.reference, kr);

        % Punto mas cercano dentro del tramo de preview
        d = sqrt((tramo(:,1) - x(k)).^2 + (tramo(:,2) - y(k)).^2);
        [~, idx] = min(d);
        ref = tramo(idx, :);
        ruta_ref(k,:) = ref;

        dx = x(k) - ref(1);
        dy = y(k) - ref(2);
        e_lat(k) = dx * sin(ref(3)) - dy * cos(ref(3));

        eh = theta(k) - ref(3);
        e_heading(k) = atan2(sin(eh), cos(eh));
    end

    % --- Metricas de precision ---
    m.nombre       = nombre;
    m.t            = t;
    m.x            = x;
    m.y            = y;
    m.e_lat        = e_lat;
    m.e_heading    = e_heading;
    m.RMSE_lateral = sqrt(mean(e_lat.^2));
    m.error_max    = max(abs(e_lat));
    m.error_medio  = mean(abs(e_lat));
    m.std_lateral  = std(e_lat);
    m.RMSE_heading = sqrt(mean(e_heading.^2));

    % --- Metricas de esfuerzo / suavidad ---
    if ~isempty(S.steering)
        ts    = S.steering.Time;
        delta = extraerSteering(S.steering);   % solo la direccion, Nt x 1
        if ~isempty(t_fin_aprox)
            ms = ts <= t_fin_aprox;
            ts = ts(ms);  delta = delta(ms);
        end
        dt = mean(diff(ts));
        m.esfuerzo_direccion = sum(delta.^2) * dt;
        m.suavidad           = sum((diff(delta)/dt).^2) * dt;
        m.delta              = delta;
        m.t_delta            = ts;
    else
        m.esfuerzo_direccion = NaN;
        m.suavidad           = NaN;
        m.delta              = [];
        m.t_delta            = [];
    end

    % --- Resumen por consola ---
    fprintf('\n===== METRICAS: %s =====\n', nombre);
    fprintf('  RMSE lateral        : %.4f m\n',   m.RMSE_lateral);
    fprintf('  Error lateral max   : %.4f m\n',   m.error_max);
    fprintf('  Error lateral medio : %.4f m\n',   m.error_medio);
    fprintf('  Desv. tipica lateral: %.4f m\n',   m.std_lateral);
    fprintf('  RMSE heading        : %.4f rad (%.2f deg)\n', ...
            m.RMSE_heading, rad2deg(m.RMSE_heading));
    if ~isnan(m.esfuerzo_direccion)
        fprintf('  Esfuerzo direccion  : %.4f\n', m.esfuerzo_direccion);
        fprintf('  Suavidad (zigzag)   : %.4f\n', m.suavidad);
    end
    fprintf('=================================\n');

    % --- Graficas ---
    figure('Name', ['Metricas - ' nombre], 'Color', 'w', ...
           'Position', [100 100 950 720]);

    subplot(2,2,1);
    plot(ruta_ref(:,1), ruta_ref(:,2), 'm--', 'LineWidth', 1.5); hold on;
    plot(x, y, 'b-', 'LineWidth', 1.5);
    plot(x(1), y(1), 'go', 'MarkerFaceColor','g');
    plot(x(end), y(end), 'rs', 'MarkerFaceColor','r');
    axis equal; grid on;
    xlabel('X [m]'); ylabel('Y [m]');
    title('Trayectoria vs Referencia');
    legend('Referencia','Vehiculo','Inicio','Fin','Location','best');

    subplot(2,2,2);
    plot(t, e_lat, 'b-', 'LineWidth', 1.2); hold on;
    yline(0, 'k:');
    yline( m.RMSE_lateral, 'r--');
    yline(-m.RMSE_lateral, 'r--');
    grid on;
    xlabel('Tiempo [s]'); ylabel('Error lateral [m]');
    title(sprintf('Error lateral (RMSE = %.3f m)', m.RMSE_lateral));

    subplot(2,2,3);
    plot(t, rad2deg(e_heading), 'b-', 'LineWidth', 1.2); hold on;
    yline(0, 'k:'); grid on;
    xlabel('Tiempo [s]'); ylabel('Error heading [deg]');
    title('Error de orientacion');

    if ~isempty(m.delta)
        subplot(2,2,4);
        plot(m.t_delta, rad2deg(m.delta), 'b-', 'LineWidth', 1.2);
        grid on;
        xlabel('Tiempo [s]'); ylabel('Direccion delta [deg]');
        title('Comando de direccion');
    else
        text(0.5,0.5,'Sin datos de direccion', ...
             'HorizontalAlignment','center'); axis off;
    end

    sgtitle(['Metricas de seguimiento - ' nombre], 'FontWeight','bold');
end


function compararTodos(resultados)
% Compara TODOS los controladores acumulados en el struct 'resultados'.
% Cada campo de 'resultados' es la struct de metricas 'm' de un controlador.
% Genera una tabla por consola y graficas comparativas.

    campos = fieldnames(resultados);
    nC = numel(campos);
    if nC == 0
        warning('No hay controladores que comparar.');
        return;
    end

    % Recopilar metricas en vectores
    nombres   = strings(1,nC);
    rmse_lat  = zeros(1,nC);
    err_max   = zeros(1,nC);
    err_med   = zeros(1,nC);
    rmse_head = zeros(1,nC);
    esfuerzo  = nan(1,nC);
    suavidad  = nan(1,nC);
    coste     = nan(1,nC);
    for i = 1:nC
        m = resultados.(campos{i});
        nombres(i)   = string(m.nombre);
        rmse_lat(i)  = m.RMSE_lateral;
        err_max(i)   = m.error_max;
        err_med(i)   = m.error_medio;
        rmse_head(i) = m.RMSE_heading;
        if isfield(m,'esfuerzo_direccion'), esfuerzo(i) = m.esfuerzo_direccion; end
        if isfield(m,'suavidad'),           suavidad(i) = m.suavidad;           end
        if isfield(m,'coste_por_paso'),     coste(i)    = m.coste_por_paso;      end
    end

    % --- Tabla por consola ---
    fprintf('\n=================== COMPARATIVA DE CONTROLADORES ===================\n');
    fprintf('%-16s | %10s | %9s | %9s | %10s | %12s\n', ...
            'Controlador','RMSE lat','Err max','Err med','RMSE head','Coste/paso');
    fprintf('-----------------+------------+-----------+-----------+------------+-------------\n');
    for i = 1:nC
        if isnan(coste(i))
            costeStr = '     ---    ';
        else
            costeStr = sprintf('%9.2f us', coste(i)*1e6);
        end
        fprintf('%-16s | %10.4f | %9.4f | %9.4f | %10.4f | %12s\n', ...
                nombres(i), rmse_lat(i), err_max(i), err_med(i), ...
                rmse_head(i), costeStr);
    end
    fprintf('====================================================================\n');

    % --- Graficas comparativas ---
    figure('Name','Comparativa de controladores','Color','w', ...
           'Position',[120 120 1000 700]);

    % (a) Error lateral en el tiempo, superpuesto
    subplot(2,2,1);
    hold on; grid on;
    for i = 1:nC
        m = resultados.(campos{i});
        plot(m.t, m.e_lat, 'LineWidth', 1.3);
    end
    yline(0,'k:');
    xlabel('Tiempo [s]'); ylabel('Error lateral [m]');
    title('Error lateral comparado');
    legend(nombres, 'Location','best');

    % (b) RMSE lateral, error max y medio (barras agrupadas)
    subplot(2,2,2);
    datosPrec = [rmse_lat(:), err_max(:), err_med(:)];   % nC x 3
    hb = bar(1:nC, datosPrec);                            % 1:nC fuerza eje X
    set(gca, 'XTick', 1:nC, 'XTickLabel', nombres);
    xlim([0.5, nC+0.5]);
    ylabel('Error lateral [m]');
    title('Precision de seguimiento');
    legend(hb, {'RMSE','Max','Medio'}, 'Location','best');
    grid on;

    % (c) RMSE de heading
    subplot(2,2,3);
    b = bar(1:nC, rad2deg(rmse_head(:)));
    b.FaceColor = [0.4 0.6 0.3];
    set(gca, 'XTick', 1:nC, 'XTickLabel', nombres);
    xlim([0.5, nC+0.5]);
    ylabel('RMSE heading [deg]');
    title('Error de orientacion');
    grid on;

    % % (d) Coste computacional (tiempo por paso del controlador)
    % subplot(2,2,4);
    % if all(isnan(coste))
    %     text(0.5,0.5,'Sin datos de coste','HorizontalAlignment','center');
    %     axis off;
    % else
    %     b = bar(1:nC, coste(:)*1e6);     % en microsegundos
    %     b.FaceColor = [0.8 0.5 0.2];
    %     set(gca, 'XTick', 1:nC, 'XTickLabel', nombres);
    %     xlim([0.5, nC+0.5]);
    %     ylabel('Tiempo por paso [\mus]');
    %     title('Coste computacional (por paso)');
    %     grid on;
    % end

    sgtitle('Comparativa de controladores', 'FontWeight','bold');
end