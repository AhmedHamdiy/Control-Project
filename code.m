% Main script body
A1 = 5; A2 = 4; R1 = 3; R2 = 5;  % Define parameters first
% State-space matrices
A = [-1/(A1*R1), 1/(A1*R1);        % dh1/dt equation
      1/(A2*R1), -(1/(A2*R1) + 1/(A2*R2))];  % dh2/dt equation

B = [1/A1; 0];                     % Input only affects dh1/dt

C = [0, 1/R2;                      % q_out output
     1/R1, -1/R1;                  % q1 output
     1, 0;                         % h1 output
     0, 1];                        % h2 output

D = zeros(4,1);                    % No direct feedthrough
% Define state-space system
sys = ss(A,B,C,D,'InputName','Qin','OutputName',{'Q2','Q1','H1','H2'});

% Create arrays of transfer functions and their names
transfer_functions = {
    tf(sys(4)),  % H2/Qin
    tf(sys(3)),  % H1/Qin
    tf(sys(2)),  % Q1/Qin
    tf(sys(1))   % Q2/Qin
};

names = {'H2(S)/Qin(S)', 'H1(S)/Qin(S)', 'Q1(S)/Qin(S)', 'Q2(S)/Qin(S)'};

% Loop through each transfer function
for k = 1:length(transfer_functions)
    tf_current = transfer_functions{k};
    [num, den] = tfdata(tf_current, 'v');
    
    % Print transfer function name
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
P = pole(tf_H2_Qin);
fprintf('\n\n\nPoles of H2/Qin: P0 = %.4f, P1 = %.4f\n', P(1), P(2));
is_stable = isstable(tf_H2_Qin);
fprintf('Is stable: %d\n', is_stable);

% Plot pole-zero map
figure;
pzmap(tf_H2_Qin);
title('Pole-Zero Map of H2/Qin');
grid on;