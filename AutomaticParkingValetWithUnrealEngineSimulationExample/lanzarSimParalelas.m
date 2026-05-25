% =====================================================================
% SCRIPT 1: EJECUCIÓN COLECTIVA DE SIMULACIONES EN PARALELO (CORREGIDO)
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
archivoAgente = "SAC_Parking_Agent_v2.mat"; 
if exist(archivoAgente, "file")
    load(archivoAgente, "agent");
    fprintf('▶️ Agente [%s] cargado con éxito en memoria.\n', archivoAgente);
else
    error('❌ Archivo de agente no encontrado.');
end

% ---------------------------------------------------------------------
% 2. CREACIÓN DEL ENTORNO NATIVO (Se hace una sola vez fuera del bucle)
% ---------------------------------------------------------------------
obsInfo = bus2RLSpec("ObservationBus");
actInfo = rlNumericSpec([1 1], LowerLimit=-1, UpperLimit=1);
env = rlSimulinkEnv(mdl, agentBlock, obsInfo, actInfo, UseFastRestart="off");
env.ResetFcn = @autoParkingValetResetFcn3D;

% ---------------------------------------------------------------------
% 3. EJECUCIÓN PARALELA NATIVA CON SIM (Forma recomendada por MathWorks)
% ---------------------------------------------------------------------
numEpisodiosTest = 100;

fprintf('\n🚀 Lanzando simulación paralela nativa de %d episodios...\n', numEpisodiosTest);
% 1. Configuramos las opciones con los nombres exactos que acepta tu versión de MATLAB
opcionesSim = rlSimulationOptions(...
    'MaxSteps', 200, ...
    'NumSimulations', numEpisodiosTest, ...
    'UseParallel', true);

% 2. Configuramos el sub-objeto de paralelización nativo para adjuntar el archivo
opcionesSim.ParallelizationOptions.AttachedFiles = "ObservationBus.mat";

% 3. Silenciamos el aviso del disco en tu sesión principal antes de lanzar
warning('off', 'Simulink:Commands:ChangeOnDisk');


% Forzamos a que el pool de workers cargue el bus en sus workspaces antes del sim
if isempty(gcp('nocreate'))
    parpool;
end
pctRunOnAll('load("ObservationBus.mat")');

% Ejecución masiva: MATLAB se encarga de repartir los 100 episodios entre tus hilos
experienciasTotales = sim(env, agent, opcionesSim);

% ---------------------------------------------------------------------
% 4. POST-PROCESAMIENTO DE TELEMETRÍA (Fuera del entorno paralelo)
% ---------------------------------------------------------------------
fprintf('📊 Procesando métricas de los resultados...\n');

rewardsTotales = zeros(numEpisodiosTest, 1);
pasosPorEpisodio = zeros(numEpisodiosTest, 1);
resultadosEpisodios = zeros(numEpisodiosTest, 1); 
variacionesVolanteLocales = zeros(numEpisodiosTest, 1);
distanciasMinimasLocales = zeros(numEpisodiosTest, 1);

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
end

% ---------------------------------------------------------------------
% 5. GUARDADO DE MATRICES TOTALES
% ---------------------------------------------------------------------
ficheroDatos = 'SAC_100_Episodios.mat';
save(ficheroDatos, ...
    'rewardsTotales', ...
    'pasosPorEpisodio', ...
    'resultadosEpisodios', ...
    'variacionesVolanteLocales', ...
    'distanciasMinimasLocales');

% Reactivar avisos al terminar
warning('on', 'Simulink:Commands:ChangeOnDisk');

fprintf('\n✅ Simulación completada con éxito.\n');
fprintf('💾 Datos crudos consolidados y exportados a "%s"\n', ficheroDatos);