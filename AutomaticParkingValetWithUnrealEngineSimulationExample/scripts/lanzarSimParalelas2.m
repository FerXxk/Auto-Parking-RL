% =====================================================================
% SCRIPT 1: EJECUCIÓN COLECTIVA DE SIMULACIONES EN PARALELO (CORREGIDO TD3)
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
% 2. CONFIGURACIÓN DEL ENTORNO PARALELO MANUAL (ROBUSTO)
% ---------------------------------------------------------------------
numEpisodiosTest = 100; % <--- Pon aquí 28, 100 o los que quieras, no fallará
fprintf('\n🚀 Lanzando bucle PARFOR de %d episodios simultáneos...\n', numEpisodiosTest);

% Pre-asignamos memoria en vectores limpios para evitar problemas entre hilos
rewardsTotales = zeros(numEpisodiosTest, 1);
pasosPorEpisodio = zeros(numEpisodiosTest, 1);
resultadosEpisodios = zeros(numEpisodiosTest, 1); 
variacionesVolanteLocales = zeros(numEpisodiosTest, 1);
distanciasMinimasLocales = zeros(numEpisodiosTest, 1);
erroresPosicionFinal = zeros(numEpisodiosTest, 1);     
erroresOrientacionFinal = zeros(numEpisodiosTest, 1);  

% Arrancamos el pool de hilos de tu procesador
if isempty(gcp('nocreate'))
    parpool;
end

% Guardamos temporalmente el bus y lo inyectamos a la fuerza en todos los hilos
save('temp_bus.mat', 'ObservationBus');
pctRunOnAll('load("temp_bus.mat")');
warning('off', 'Simulink:Commands:ChangeOnDisk');

% ---------------------------------------------------------------------
% 3. BUCLE DE EJECUCIÓN PARALELA AISLADA
% ---------------------------------------------------------------------
parfor idx = 1:numEpisodiosTest
    % Cada hilo carga su copia limpia del modelo
    mdlLocal = "rlAutoParkingValet3D";
    load_system(mdlLocal);
    
    % Creamos un entorno de simulación local exclusivo para este hilo
    obsInfoWorker = bus2RLSpec("ObservationBus");
    actInfoWorker = rlNumericSpec([1 1], LowerLimit=-1, UpperLimit=1);
    agentBlockWorker = mdlLocal + "/Controller/RL Controller/RL Agent";
    
    envWorker = rlSimulinkEnv(mdlLocal, agentBlockWorker, obsInfoWorker, actInfoWorker, UseFastRestart="off");
    envWorker.ResetFcn = @autoParkingValetResetFcn3D;
    
    % Simulamos 1 único episodio en este hilo de forma secuencial externa
    opcionesIndividuales = rlSimulationOptions('MaxSteps', 200, 'NumSimulations', 1, 'UseParallel', false);
    
    try
        % Simulamos el episodio actual de forma aislada
        expIndividual = sim(envWorker, agent, opcionesIndividuales);
        
        % Extraer recompensas
        historicoPremios = expIndividual.Reward.Data;
        rewardsTotales(idx) = sum(historicoPremios);
        pasosPorEpisodio(idx) = length(historicoPremios);
        
        % Criterio de Éxito
        if historicoPremios(end) > 50
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
        
        % Errores de Pose Finales
        datosPoseFinal = expIndividual.Observation.poseInfo.Data(:,:,end);
        erroresPosicionFinal(idx) = norm(datosPoseFinal(1:2)); 
        erroresOrientacionFinal(idx) = datosPoseFinal(3);
        
    catch
        % En caso de error crítico en la simulación, marcamos como fallo
        resultadosEpisodios(idx) = 0;
        distanciasMinimasLocales(idx) = NaN;
        erroresPosicionFinal(idx) = NaN;
        erroresOrientacionFinal(idx) = NaN;
    end
end

% ---------------------------------------------------------------------
% 4. LIMPIEZA Y GUARDADO DE MATRICES TOTALES
% ---------------------------------------------------------------------
delete('temp_bus.mat'); % Borramos el archivo de intercambio

ficheroDatos = 'TD3_100_Episodios.mat';
save(ficheroDatos, ...
    'rewardsTotales', ...
    'pasosPorEpisodio', ...
    'resultadosEpisodios', ...
    'variacionesVolanteLocales', ...
    'distanciasMinimasLocales', ...
    'erroresPosicionFinal', ...      
    'erroresOrientacionFinal');     

% Reactivar avisos al terminar
warning('on', 'Simulink:Commands:ChangeOnDisk');
fprintf('\n✅ Simulación completada con éxito.\n');
fprintf('💾 Datos consolidados exportados a "%s"\n', ficheroDatos);