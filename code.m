A1 = 5; A2 = 4; R1 = 3; R2 = 5;
% State-space matrices

% Assume our state is [h1, h2] , input is Qin, outputs are Q2, Q1, H1, H2

% State Matrix
A = [-1/(A1*R1), 1/(A1*R1);        % dh1/dt equation
    1/(A2*R1), -(1/(A2*R1) + 1/(A2*R2))];  % dh2/dt equation

% Input Matrix
B = [1/A1; 0];                     % Input only affects dh1/dt

% Output Matrix
C = [0, 1/R2;                      % q_out output
    1/R1, -1/R1;                  % q1 output
    1, 0;                         % h1 output
    0, 1];                        % h2 output

% Direct Feedthrough Matrix
D = zeros(4,1);

sys = ss(A,B,C,D,'InputName','Qin','OutputName',{'Q2','Q1','H1','H2'});

transfer_functions = {
    tf(sys(4)),  % H2/Qin
    tf(sys(3)),  % H1/Qin
    tf(sys(2)),  % Q1/Qin
    tf(sys(1))   % Q2/Qin
    };

names = {'H2(S)/Qin(S)', 'H1(S)/Qin(S)', 'Q1(S)/Qin(S)', 'Q2(S)/Qin(S)'};

for k = 1:length(transfer_functions)
    tf_current = transfer_functions{k};
    [num, den] = tfdata(tf_current, 'v');

    fprintf('\n\n\n%s = \n\n', names{k});

    % Print numerator
    for i = 1:length(num)
        power = length(num)-i;
        if num(i) == 0
            continue;
        end
        if power > 0
            if num(i) == 1
                fprintf('S^%d + ', power);
            else
                fprintf('%.4gS^%d + ', num(i), power);
            end
        else
            fprintf('%.4g', num(i));
        end
    end

    % Print denominator
    fprintf('\n------------------\n');
    for i = 1:length(den)
        power = length(den)-i;
        if den(i) == 0
            continue;
        end
        if power > 0
            if den(i) == 1
                fprintf('S^%d + ', power);
            else
                fprintf('%.4gS^%d + ', den(i), power);
            end
        else
            fprintf('%.4g', den(i));
        end
    end

end

% Stability analysis
P = pole(tf(sys(4)));
fprintf('\n\n\nPoles of H2/Qin: P0 = %.4f, P1 = %.4f\n', P(1), P(2));
is_stable = isstable(tf(sys(4)));
fprintf('Is stable: %d\n', is_stable);

figure;
pzmap(tf(sys(4)));
title('Pole-Zero Map of H2/Qin');
grid on;

% Simulate response to a step input (1 m^3/s)
t = linspace(0, 500, 10000);  % 10,000 samples over 500 seconds
u = ones(size(t));           % Step input of 1 m^3/s

[y, t_out, x] = lsim(sys, u, t);

% Plot h1
figure;
plot(t_out, y(:,3), 'b', 'LineWidth', 1.5);
grid on;
title('h1 (m)');
xlabel('Time (s)');
ylabel('h1');

% Plot h2
figure;
plot(t_out, y(:,4), 'r', 'LineWidth', 1.5);
grid on;
title('h2 (m)');
xlabel('Time (s)');
ylabel('h2');

% Plot Q1
figure;
plot(t_out, y(:,2), 'g', 'LineWidth', 1.5);
grid on;
title('Q1 (m^3/s)');
xlabel('Time (s)');
ylabel('Q1');

% Plot Q2
figure;
plot(t_out, y(:,1), 'm', 'LineWidth', 1.5);
grid on;
title('Q2 (m^3/s)');
xlabel('Time (s)');
ylabel('Q2');

% Calculate steady-state values
steady_state_values = y(end, :);
fprintf('\nSteady-state values:\n');
fprintf('h1 = %.4f m\n', steady_state_values(3));
fprintf('h2 = %.4f m\n', steady_state_values(4));
fprintf('Q1 = %.4f m^3/s\n', steady_state_values(2));
fprintf('Q2 = %.4f m^3/s\n', steady_state_values(1));

% Modify the system to have a feedback with a reference signal h_d
sys_cl = feedback(sys(4,:),1);

% Simulate response to a step input (h_d = 5 meters)
t = linspace(0,100,10000);  % 10,000 samples over 100 seconds

hd = 5 * ones(size(t));

[h2_response,t_out] = lsim(sys_cl,hd,t);

% Plot h2 response
figure;
plot(t_out, h2_response, 'r', 'LineWidth', 2);
grid on;
title('Response of h2 to desired level h_d = 5m');
xlabel('Time (s)');
ylabel('h2 (m)');

info = stepinfo(h2_response, t_out, 5);  % 5 is the desired final value

fprintf('Rise time: %.4f seconds\n', info.RiseTime);
fprintf('Peak time: %.4f seconds\n', info.PeakTime);
fprintf('Maximum overshoot: %.2f%%\n', info.Overshoot);
fprintf('Settling time: %.4f seconds\n', info.SettlingTime);

% Steady-state error
ess = abs(5 - h2_response(end));
fprintf('Steady-state error (ess): %.4f meters\n', ess);


% adding the controller to the system

Kp_values = [1, 10, 100];
hd = 5 * ones(size(t));  % Desired height

for i = 1:length(Kp_values)
    Kp = Kp_values(i);
    
    % Closed-loop transfer function with proportional controller
    sys_cl = feedback(Kp * tf(sys(4)), 1);
    
    % Simulate response
    [h2_response, t_out] = lsim(sys_cl, hd, t);
    
    % Plot response
    figure;
    plot(t_out, h2_response, 'LineWidth', 2);
    grid on;
    title(sprintf('h2 Response with Kp = %d', Kp));
    xlabel('Time (s)');
    ylabel('h2 (m)');
    
    % Step response characteristics
    info = stepinfo(h2_response, t_out, 5);  % Final value = 5
    ess = abs(5 - h2_response(end));         % Steady-state error
    
    fprintf('\n\n--- Kp = %d ---\n', Kp);
    fprintf('Rise Time: %.4f s\n', info.RiseTime);
    fprintf('Peak Time: %.4f s\n', info.PeakTime);
    fprintf('Overshoot: %.2f %%\n', info.Overshoot);
    fprintf('Settling Time: %.4f s\n', info.SettlingTime);
    fprintf('Steady-State Error: %.4f m\n', ess);
end
