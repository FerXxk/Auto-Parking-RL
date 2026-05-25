% =====================================================================
% SCRIPT 1: EJECUCIÓN COLECTIVA DE SIMULACIONES EN PARALELO (CON ERRORES)
% =====================================================================
clc; clear; close all;
fprintf('-> Preparando hilos del procesador para simulación por lotes...\n');

% 1. Parámetros obligatorios de tiempo y modelo de Simulink
Ts = 0.1;
Tf = 50;
mdl = "rlAutoParkingValet3D";

% Variables globales de configuración (se quedan en tu workspace principal)
doTraining = false;          
egoInitialPose = [40 -55 pi/2]; 
vehiclePose = [10 -32.5 pi];  
searchDist = 10;              
freeSpotIndex = 18;           

% Inicializamos el gestor del parking local
parkingLot = ParkingLotManager;
info = parkingLot.getInfo(); 

% Cargar los parámetros y buses en el Workspace principal
load("ObservationBus.mat", "ObservationBus");
autoParkingValetParams3D; 
createMPCForParking3D; 

% Asegurar que el modelo está cargado y limpio en el host antes de clonar workers
bdclose(mdl);
load_system(mdl);

% Desactivar el renderizado 3D de Unreal Engine globalmente antes de enviar a workers
set_param(mdl + "/Vehicle Dynamics and Sensing/Unreal Engine Visualization and Sensing", "enableUEViz", "off");
set_param(mdl + "/Vehicle Dynamics and Sensing/Unreal Engine Visualization and Sensing", "enablePCViz", "off");

% Configuración de bloques del agente
agentBlock = mdl + "/Controller/RL Controller/RL Agent";

% CARGA DEL AGENTE
archivoAgente = "ParkingValetAgentTrained.mat"; 
if exist(archivoAgente, "file")
    load(archivoAgente, "agent");
    fprintf('▶️ Agente [%s] cargado con éxito en memoria.\n', archivoAgente);
else
    error('❌ Archivo de agente no encontrado.');
end

% ---------------------------------------------------------------------
% 2. CREACIÓN DEL ENTORNO NATIVO
% ---------------------------------------------------------------------
obsInfo = bus2RLSpec("ObservationBus");
actInfo = rlNumericSpec([1 1], LowerLimit=-1, UpperLimit=1);
env = rlSimulinkEnv(mdl, agentBlock, obsInfo, actInfo, UseFastRestart="off");
env.ResetFcn = @autoParkingValetResetFcn3D;

% ---------------------------------------------------------------------
% 3. EJECUCIÓN PARALELA NATIVA CON SIM
% ---------------------------------------------------------------------
numEpisodiosTest = 100;
fprintf('\n🚀 Lanzando simulación paralela nativa de %d episodios...\n', numEpisodiosTest);

opcionesSim = rlSimulationOptions(...
    'MaxSteps', 200, ...
    'NumSimulations', numEpisodiosTest, ...
    'UseParallel', true);

opcionesSim.ParallelizationOptions.AttachedFiles = "ObservationBus.mat";
warning('off', 'Simulink:Commands:ChangeOnDisk');

if isempty(gcp('nocreate'))
    parpool;
end
pctRunOnAll('load("ObservationBus.mat")');

% Ejecución masiva
experienciasTotales = sim(env, agent, opcionesSim);

% ---------------------------------------------------------------------
% 4. POST-PROCESAMIENTO DE TELEMETRÍA (MÉTRICAS DE ERROR AÑADIDAS)
% ---------------------------------------------------------------------
fprintf('📊 Procesando métricas de los resultados...\n');

rewardsTotales = zeros(numEpisodiosTest, 1);
pasosPorEpisodio = zeros(numEpisodiosTest, 1);
resultadosEpisodios = zeros(numEpisodiosTest, 1); 
variacionesVolanteLocales = zeros(numEpisodiosTest, 1);
distanciasMinimasLocales = zeros(numEpisodiosTest, 1);

% Nuevas matrices para almacenar los errores de aparcamiento finales
erroresPosicionFinal = zeros(numEpisodiosTest, 1);     % Distancia en metros al centro de la plaza
erroresOrientacionFinal = zeros(numEpisodiosTest, 1);  % Desviación en radianes (o grados) del ángulo ideal

for idx = 1:numEpisodiosTest
    expIndividual = experienciasTotales(idx);
    
    % Extraer premios
    historicoPremios = expIndividual.Reward.Data;
    rewardsTotales(idx) = sum(historicoPremios);
    pasosPorEpisodio(idx) = length(historicoPremios);
    
    % Criterio de Éxito
    recompensaFinal = historicoPremios(end);
    if recompensaFinal > 50
        resultadosEpisodios(idx) = 1;
    else
        resultadosEpisodios(idx) = 0;
    end
    
    % Esfuerzo de control (Volante)
    nAcciones = fieldnames(expIndividual.Action);
    volante = expIndividual.Action.(nAcciones{1}).Data(:);
    volante = volante(~isnan(volante) & ~isinf(volante));
    if length(volante) > 1
        variacionesVolanteLocales(idx) = mean(abs(diff(volante)));
    end
    
    % Margen de Seguridad (Lidar)
    dLidar = expIndividual.Observation.lidarData.Data(:);
    lecturasObstaculos = dLidar(dLidar > 0.05 & dLidar < 6);
    if ~isempty(lecturasObstaculos)
        distanciasMinimasLocales(idx) = min(lecturasObstaculos);
    else
        distanciasMinimasLocales(idx) = NaN;
    end
    
    % -----------------------------------------------------------------
    % EXTRACCIÓN DE ERRORES DE POSE EN EL ÚLTIMO PASO (APARCADO)
    % -----------------------------------------------------------------
    % Extraemos la matriz de datos de poseInfo en el último instante de tiempo
    datosPoseFinal = expIndividual.Observation.poseInfo.Data(:,:,end);
    
    % NOTA: En este entorno, poseInfo suele entregar [ErrorLateral, ErrorLongitudinal, ErrorGuiado]
    % o bien directamente la pose absoluta [X, Y, Theta] respecto a la plaza.
    % Calculamos el error de posición euclídeo usando los dos primeros componentes:
    erroresPosicionFinal(idx) = norm(datosPoseFinal(1:2)); 
    
    % El tercer componente es el error de orientación (Heading/Yaw error)
    erroresOrientacionFinal(idx) = datosPoseFinal(3);
end

% ---------------------------------------------------------------------
% 5. GUARDADO DE MATRICES TOTALES AMPLIADO
% ---------------------------------------------------------------------
ficheroDatos = 'TD3_100_Episodios.mat';
save(ficheroDatos, ...
    'rewardsTotales', ...
    'pasosPorEpisodio', ...
    'resultadosEpisodios', ...
    'variacionesVolanteLocales', ...
    'distanciasMinimasLocales', ...
    'erroresPosicionFinal', ...      % <--- Guardado en el .mat
    'erroresOrientacionFinal');     % <--- Guardado en el .mat

% Reactivar avisos al terminar
warning('on', 'Simulink:Commands:ChangeOnDisk');
fprintf('\n✅ Simulación completada con éxito.\n');
fprintf('💾 Datos consolidados (incluyendo errores de aparcamiento) exportados a "%s"\n', ficheroDatos);