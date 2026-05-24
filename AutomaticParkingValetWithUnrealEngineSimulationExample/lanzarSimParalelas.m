% =====================================================================
% SCRIPT 1: EJECUCIÓN COLECTIVA DE SIMULACIONES EN PARALELO
% =====================================================================
clc; clear; close all;
fprintf('-> Preparando hilos del procesador para simulación por lotes...\n');

% 1. Parámetros obligatorios de tiempo y modelo de Simulink
Ts = 0.1;
Tf = 50;
mdl = "rlAutoParkingValet3D";

% Variables globales de configuración
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

% Configuramos los nombres de los bloques de cara a los workers
agentBlock = mdl + "/Controller/RL Controller/RL Agent";

% CARGA DEL AGENTE
archivoAgente = "SAC_Parking_Agent_v2.mat"; 
if exist(archivoAgente, "file")
    load(archivoAgente, "agent");
    fprintf('▶️ Agente [%s] cargado con éxito en memoria.\n', archivoAgente);
else
    error('❌ Archivo de agente no encontrado. Asegúrate de tener "ParkingValetAgentTrained.mat" en la carpeta.');
end

% ---------------------------------------------------------------------
% 2. EJECUCIÓN PARALELA ROBUSTA MEDIANTE PARFOR
% ---------------------------------------------------------------------
numEpisodiosTest = 100;

% Pre-asignamos memoria en arrays nativos
rewardsTotales = zeros(numEpisodiosTest, 1);
pasosPorEpisodio = zeros(numEpisodiosTest, 1);
resultadosEpisodios = zeros(numEpisodiosTest, 1); 
variacionesVolanteLocales = zeros(numEpisodiosTest, 1);
distanciasMinimasLocales = zeros(numEpisodiosTest, 1);

fprintf('\n🚀 Lanzando bucle PARFOR de %d episodios simultáneos...\n', numEpisodiosTest);
fprintf('🔥 Tu CPU está procesando las matemáticas en lote. Por favor, espera...\n');

if isempty(gcp('nocreate'))
    parpool;
end

parfor idx = 1:numEpisodiosTest
    % 1. Cargar el modelo en la memoria RAM del Worker antes de tocar parámetros
    load_system("rlAutoParkingValet3D"); 
    
    % 2. Forzar borrado del visualizador fantasma para saltar el StartFcn
    if evalin('base', 'exist("visualizer", "var")')
        evalin('base', 'clear visualizer');
    end
    
    % 3. Inyectar variables exigidas en el workspace base de este Worker
    assignin('base', 'doTraining', false);
    assignin('base', 'Ts', 0.1);
    assignin('base', 'Tf', 50);
    assignin('base', 'mdl', "rlAutoParkingValet3D");
    assignin('base', 'egoInitialPose', [40 -55 pi/2]);
    assignin('base', 'vehiclePose', [10 -32.5 pi]);
    assignin('base', 'searchDist', 10);
    assignin('base', 'freeSpotIndex', 18);
    
    pLotWorker = ParkingLotManager;
    assignin('base', 'parkingLot', pLotWorker);
    assignin('base', 'info', pLotWorker.getInfo());
    
    % Desactivar por completo el renderizado 3D de Unreal Engine
    set_param("rlAutoParkingValet3D/Vehicle Dynamics and Sensing/Unreal Engine Visualization and Sensing", "enableUEViz", "off");
    set_param("rlAutoParkingValet3D/Vehicle Dynamics and Sensing/Unreal Engine Visualization and Sensing", "enablePCViz", "off");
    
    % 4. Crear el objeto de entorno local
    obsInfoWorker = bus2RLSpec("ObservationBus");
    actInfoWorker = rlNumericSpec([1 1], LowerLimit=-1, UpperLimit=1);
    envWorker = rlSimulinkEnv(mdl, agentBlock, obsInfoWorker, actInfoWorker, UseFastRestart="off");
    envWorker.ResetFcn = @autoParkingValetResetFcn3D;
    
    opcionesIndividuales = rlSimulationOptions('MaxSteps', 200, 'NumSimulations', 1, 'UseParallel', false);
    
    try
        expIndividual = sim(envWorker, agent, opcionesIndividuales);
        
        % Extraer telemetría cruda
        historicoPremios = expIndividual.Reward.Data;
        rewardsTotales(idx) = sum(historicoPremios);
        pasosPorEpisodio(idx) = length(historicoPremios);
        
        % Criterio de Éxito
        recompensaFinal = historicoPremios(end);
        if recompensaFinal > 50
            resultadosEpisodios(idx) = 1;
        elseif recompensaFinal < -40 || pasosPorEpisodio(idx) >= 200
            resultadosEpisodios(idx) = 0;
        else
            resultadosEpisodios(idx) = 1;
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
        
    catch
        rewardsTotales(idx) = 0;
        pasosPorEpisodio(idx) = 200;
        resultadosEpisodios(idx) = 0;
        distanciasMinimasLocales(idx) = NaN;
    end
end

% ---------------------------------------------------------------------
% 3. GUARDADO DE MATRICES TOTALES EN MAT-FILE (SIN CÁLCULOS ESTÁTICOS)
% ---------------------------------------------------------------------
ficheroDatos = 'SAC_100_Episodios.mat';
save(ficheroDatos, ...
    'rewardsTotales', ...
    'pasosPorEpisodio', ...
    'resultadosEpisodios', ...
    'variacionesVolanteLocales', ...
    'distanciasMinimasLocales');

fprintf('\n✅ Simulación completada de forma segura.\n');
fprintf('💾 Datos crudos consolidados y exportados a "%s"\n', ficheroDatos);