% =====================================================================
% SCRIPT 2: POST-PROCESADO ESTADÍSTICO Y GENERACIÓN DE DASHBOARD
% =====================================================================
clc; clear; close all;

ficheroDatos = 'TD3_100_Episodios.mat';

% Verificar si existe el archivo antes de procesar
if ~exist(ficheroDatos, 'file')
    error('❌ No se encuentra el archivo "%s". Ejecuta primero el Script 1.', ficheroDatos);
end

fprintf('📂 Cargando métricas históricas de "%s"...\n', ficheroDatos);
load(ficheroDatos);

numEpisodiosTest = length(rewardsTotales);

% ---------------------------------------------------------------------
% 1. CONSOLIDACIÓN MATEMÁTICA DE MÉTRICAS (KPIs)
% ---------------------------------------------------------------------
numExitos = sum(resultadosEpisodios == 1);
porcentajeExito = (numExitos / numEpisodiosTest) * 100;
porcentajeColision = 100 - porcentajeExito;

esfuerzoMedio = mean(variacionesVolanteLocales(variacionesVolanteLocales > 0));
distanciasReales = distanciasMinimasLocales(~isnan(distanciasMinimasLocales) & distanciasMinimasLocales > 0);

% ---------------------------------------------------------------------
% 2. PRESENTACIÓN DE RESULTADOS POR CONSOLA
% ---------------------------------------------------------------------
fprintf('\n================ MÉTRICAS DE VALIDACIÓN ================\n');
fprintf('Episodios Evaluados                      : %d\n', numEpisodiosTest);
fprintf('Recompensa Media en Test                 : %.2f\n', mean(rewardsTotales));
fprintf('Tasa de Éxito Comercial (Aparcado Seguro): %.1f%%\n', porcentajeExito);
fprintf('Tasa de Siniestralidad (Choques/Fallas)  : %.1f%%\n', porcentajeColision);
fprintf('Tiempo Medio de Maniobra                 : %.1f pasos de control\n', mean(pasosPorEpisodio));
fprintf('Esfuerzo Medio de Control (Volante)      : %.4f rad/paso\n', esfuerzoMedio);
if ~isempty(distanciasReales)
    fprintf('Margen de Seguridad Promedio (Lidar)     : %.2f metros\n', mean(distanciasReales));
else
    fprintf('Margen de Seguridad Promedio (Lidar)     : Fuera de rango de obstáculos\n');
end
fprintf('========================================================\n');

% ---------------------------------------------------------------------
% 3. GENERACIÓN DEL DASHBOARD GRÁFICO (REPRESENTACIÓN AVANZADA)
% ---------------------------------------------------------------------
figure('Name', 'Dashboard de Validación de Agente SAC', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 450]);

% --- Subplot 1: Rendimiento Temporal por Episodio ---
subplot(1, 2, 1);
yyaxis left
plot(rewardsTotales, '-o', 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
ylabel('Recompensa Total Acumulada'); 
ax = gca; ax.YColor = [0 0.4470 0.7410];

yyaxis right
bar(pasosPorEpisodio, 'FaceColor', [0.8500 0.3250 0.0980], 'FaceAlpha', 0.3);
ylabel('Tiempo de Maniobra (Pasos)'); 
ax = gca; ax.YColor = [0.8500 0.3250 0.0980];

grid on; 
xlabel('Número de Episodio de Test'); 
title('Rendimiento del Agente por Episodio');

% --- Subplot 2: Diagrama de Tarta (KPI de Seguridad) ---
subplot(1, 2, 2);
datosQuesito = [porcentajeExito, porcentajeColision];
etiquetas = {['Éxito Seguro (' num2str(porcentajeExito, '%.1f') '%)'], ...
             ['Siniestros/Fallas (' num2str(porcentajeColision, '%.1f') '%)']};

if porcentajeColision == 0
    p = pie(datosQuesito(1), etiquetas(1)); 
    p(1).FaceColor = [0.4660 0.6740 0.1880]; % Verde
else
    p = pie(datosQuesito, etiquetas);
    p(1).FaceColor = [0.4660 0.6740 0.1880]; % Verde Éxito
    p(3).FaceColor = [0.6350 0.0780 0.1840]; % Rojo Fallo
end
title('Tasa de Seguridad Comercial (KPI)');

fprintf('📊 Dashboard generado en pantalla con éxito.\n');