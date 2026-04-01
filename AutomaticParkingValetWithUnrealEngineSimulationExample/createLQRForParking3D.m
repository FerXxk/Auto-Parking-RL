%% Design LQR controller for tracking
Xref = parkingLot.createReferenceTrajectory(Ts, Tf);

% 1. Punto de operación NEUTRO (Ángulo 0, mirando al frente)
x_nom = [0; 0; 0]; 
v_ref = 2.5;       
u_nom = [v_ref; 0]; 

% 2. Obtener el modelo
[Ad, Bd, Cd, Dd] = vehicleStateJacobianFcnDT(Ts, x_nom, u_nom);

% 3. Matrices de pesos (Q: [Error Longitudinal, Error Lateral, Error Theta])
Q = diag([1, 15, 5]); % Le damos un 15 al error lateral para que siga la línea con fuerza
R = diag([0.1, 5]);

% 4. Calcular ganancia
K_lqr = dlqr(Ad, Bd, Q, R);
disp('Nueva Ganancia LQR calculada en marco local:');
disp(K_lqr);