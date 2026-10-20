function control_dashboard
%% =========================================================================
%% BCN3D MOVEO 4-DOF INTERACTIVE DASHBOARD WITH LIVE SERIAL TELEMETRY
%% =========================================================================
% DISPATCHED JOINTS (Sent to Arduino over Serial):
%   J1: "shoulder->arm"      [Shoulder Pitch around X-axis]
%   J2: "arm->f-arm"          [Elbow Pitch around X-axis]
%   J3: "f-arm -> wrist-r"    [Wrist Roll around Z-axis]
%   J4: "wrist-r -> wrist-p"  [Wrist Pitch around X-axis, Lower Limit: 0 rad]
%
% STATIC JOINT:
%   "base -> shoulder"        [Base Yaw locked at 0.0 deg]
%
% WORKSPACE CONSTRAINTS:
%   Target X MUST be 0.00 m.
%   Suggested Test Targets:
%     - Mid Reach:  [0.00, 0.25, 0.35] m
%     - Low Reach:  [0.00, 0.30, 0.22] m
%     - High Reach: [0.00, 0.20, 0.45] m
%% =========================================================================

clearvars -except robot urdfPath;
clc;
close all;

%% --------------------------- USER CONFIGURATION --------------------------
urdfPath   = fullfile('Assemverb2.SLDASM', 'urdf', 'Assemverb2.SLDASM.urdf');
eeName     = 'hand_link';             % End-effector link name
comPort    = "COM15";                  % Arduino USB serial port
baudRate   = 115200;                  % Serial baud rate
numFrames  = 60;                      % Trajectory animation frames

if ~isfile(urdfPath)
    error('URDF file not found at: %s', urdfPath);
end

robot = importrobot(urdfPath);
robot.DataFormat = 'column';

% Lock static base joint (Base -> Shoulder locked at 0 rad)
for i = 1:numel(robot.Bodies)
    joint = robot.Bodies{i}.Joint;
    if strcmp(joint.Name, 'base -> shoulder')
        robot.Bodies{i}.Joint.PositionLimits = [0, 0];
    end
end

qHome = homeConfiguration(robot);

%% ------------------------ BUILD UI DASHBOARD -----------------------------
fig = uifigure('Name', 'Moveo 4-DOF Control Panel & Live Telemetry', 'Position', [100, 100, 1200, 720]);

% Left Side: 3D Visualization Axis
pnlVisual = uipanel(fig, 'Position', [10, 10, 840, 700], 'Title', '3D Robot Simulation (Click & drag to rotate view)');
ax = uiaxes(pnlVisual, 'Position', [10, 10, 820, 660]);
show(robot, qHome, 'Parent', ax, 'PreservePlot', false);
grid(ax, 'on');
axis(ax, 'equal');
xlim(ax, [-0.45 0.45]);
ylim(ax, [-0.10 0.55]);
zlim(ax, [0 0.75]);
view(ax, 140, 25);

% Mouse interaction controls
ax.Interactions = [rotateInteraction];
axtoolbar(ax, {'rotate', 'pan', 'zoomin', 'zoomout', 'restoreview'});

% Right Side: Control & Live Telemetry Panel
pnlCtrl = uipanel(fig, 'Position', [860, 10, 330, 700], 'Title', 'Target & Execution Controls');

% Target Coordinate Inputs
uilabel(pnlCtrl, 'Position', [20, 640, 280, 22], 'Text', 'Target Position (meters):', 'FontWeight', 'bold');

uilabel(pnlCtrl, 'Position', [20, 605, 60, 22], 'Text', 'X (m):');
efX = uieditfield(pnlCtrl, 'numeric', 'Position', [80, 605, 220, 22], 'Value', 0.00, 'Tooltip', 'Must be 0.00 m (Base Yaw static)');

uilabel(pnlCtrl, 'Position', [20, 570, 60, 22], 'Text', 'Y (m):');
efY = uieditfield(pnlCtrl, 'numeric', 'Position', [80, 570, 220, 22], 'Value', 0.25, 'Tooltip', 'Reachable range: 0.15 to 0.40 m');

uilabel(pnlCtrl, 'Position', [20, 535, 60, 22], 'Text', 'Z (m):');
efZ = uieditfield(pnlCtrl, 'numeric', 'Position', [80, 535, 220, 22], 'Value', 0.35, 'Tooltip', 'Reachable range: 0.15 to 0.55 m');

% Action Buttons
btnSimulate = uibutton(pnlCtrl, 'push', 'Position', [20, 460, 280, 36], ...
    'Text', '1. Plan & Simulate Path', 'BackgroundColor', [0.2 0.6 1.0], 'FontColor', 'w', 'FontWeight', 'bold');

btnDispatch = uibutton(pnlCtrl, 'push', 'Position', [20, 415, 280, 36], ...
    'Text', '2. Send to Arduino', 'BackgroundColor', [0.2 0.75 0.2], 'FontColor', 'w', 'FontWeight', 'bold', 'Enable', 'off');

% Status & Live Telemetry Display
uilabel(pnlCtrl, 'Position', [20, 375, 280, 22], 'Text', 'Live Status / Telemetry Stream:', 'FontWeight', 'bold');
lblStatus = uitextarea(pnlCtrl, 'Position', [20, 95, 280, 275], 'Editable', 'off', ...
    'FontName', 'Courier', 'FontSize', 11, ...
    'Value', {
        'Ready.';
        'Base is locked at 0 deg.';
        'Wrist Pitch limits: [0.0, 1.57] rad';
        'Active Joints:';
        '  - J1: Shoulder Pitch';
        '  - J2: Elbow Pitch';
        '';
        'Enter coordinates and click Plan.'
    });

btnReset = uibutton(pnlCtrl, 'push', 'Position', [20, 40, 280, 34], ...
    'Text', 'Reset to Home Position', 'BackgroundColor', [0.85 0.85 0.85]);

%% ------------------- STORE APPLICATION STATE -----------------------------
fig.UserData = struct(...
    'solvedConfig', [], ...
    'finalErrorMM', Inf, ...
    'activeJointIdx', [], ...
    'activeJointNames', {{}});

%% ------------------- CALLBACK REGISTRATIONS ------------------------------
btnSimulate.ButtonPushedFcn = @(src, event) runSimulation();
btnDispatch.ButtonPushedFcn = @(src, event) dispatchToArduino();
btnReset.ButtonPushedFcn    = @(src, event) resetToHome();

%% ------------------- WORKER FUNCTIONS ------------------------------------
    function runSimulation()
        targetPos = [efX.Value, efY.Value, efZ.Value];
        
        lblStatus.Value = {'Solving Inverse Kinematics...'};
        drawnow;
        
        % Identify active joints and cache configuration bounds
        activeJointIdx = [];
        activeJointNames = {};
        jointBounds = zeros(numel(qHome), 2);
        
        qRow = 1;
        for i = 1:numel(robot.Bodies)
            j = robot.Bodies{i}.Joint;
            if ~strcmp(j.Type, 'fixed')
                jointBounds(qRow, :) = j.PositionLimits;
                if (j.PositionLimits(1) < j.PositionLimits(2))
                    activeJointIdx(end+1) = i; %#ok<AGROW>
                    activeJointNames{end+1} = j.Name; %#ok<AGROW>
                end
                qRow = qRow + 1;
            end
        end
        
        % Vectorized clamp to prevent boundary violations
        clampConfig = @(q) max(jointBounds(:, 1), min(jointBounds(:, 2), q));
        
        %% 2. INVERSE KINEMATICS SETUP
        ik = inverseKinematics('RigidBodyTree', robot);
        ik.SolverParameters.MaxIterations = 1500;
        ik.SolverParameters.SolutionTolerance = 1e-8;
        
        targetTform = trvec2tform(targetPos);
        
        %% 3. MULTI-BRANCH POSTURE SOLVE
        idxShoulder = find(strcmp(robot.BodyNames, 'arm_link'));
        idxElbow    = find(strcmp(robot.BodyNames, 'forearm_link'));
        idxWristR   = find(strcmp(robot.BodyNames, 'wrist_role_link'));
        idxWristP   = find(strcmp(robot.BodyNames, 'wrist_pitch_liink'));
        
        seeds = {};
        
        % Branch 1: Natural Reach (Wrist pitch set to +15 deg >= 0)
        s1 = qHome;
        s1(idxShoulder) = deg2rad(-25);
        s1(idxElbow)    = deg2rad(45);
        s1(idxWristR)   = 0;
        s1(idxWristP)   = deg2rad(15);
        seeds{end+1} = clampConfig(s1);
        
        % Branch 2: High Reach
        s2 = qHome;
        s2(idxShoulder) = deg2rad(-10);
        s2(idxElbow)    = deg2rad(30);
        s2(idxWristR)   = 0;
        s2(idxWristP)   = deg2rad(30);
        seeds{end+1} = clampConfig(s2);
        
        % Branch 3: Low Extension Reach
        s3 = qHome;
        s3(idxShoulder) = deg2rad(-40);
        s3(idxElbow)    = deg2rad(60);
        s3(idxWristR)   = 0;
        s3(idxWristP)   = deg2rad(10);
        seeds{end+1} = clampConfig(s3);
        
        weights = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        bestSol = clampConfig(qHome);
        minPosError = Inf;
        
        for s = 1:numel(seeds)
            [candidateSol, ~] = ik(eeName, targetTform, weights, seeds{s});
            actualTform = getTransform(robot, candidateSol, eeName);
            currentPos = actualTform(1:3, 4)';
            posErr = norm(targetPos - currentPos);
            if posErr < minPosError
                minPosError = posErr;
                bestSol = candidateSol;
            end
        end
        
        %% 4. POLISH SOLUTION WITH STRICT BOUNDARY CLAMPING
        % Clamp bestSol to eliminate any numerical boundary overshoot from Stage 1
        bestSol = clampConfig(bestSol);
        
        [configSol, solInfo] = ik(eeName, targetTform, weights, bestSol);
        
        % Clamp final solution to guarantee all joint limits are satisfied
        configSol = clampConfig(configSol);
        
        actualTform = getTransform(robot, configSol, eeName);
        actualPos = actualTform(1:3, 4)';
        finalErrorMM = norm(targetPos - actualPos) * 1000;
        
        %% 5. INTERPOLATE TRAJECTORY
        qStart = clampConfig(qHome);
        trajectory = zeros(numel(qStart), numFrames);
        for j = 1:numel(qStart)
            trajectory(j, :) = linspace(qStart(j), configSol(j), numFrames);
        end
        
        userData = fig.UserData;
        userData.solvedConfig     = configSol;
        userData.finalErrorMM     = finalErrorMM;
        userData.activeJointIdx   = activeJointIdx;
        userData.activeJointNames = activeJointNames;
        fig.UserData = userData;
        
        [az, el] = view(ax);
        
        cla(ax);
        show(robot, qHome, 'Parent', ax, 'PreservePlot', false);
        hold(ax, 'on');
        plot3(ax, targetPos(1), targetPos(2), targetPos(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2.5);
        plot3(ax, 0, 0, 0, 'ko', 'MarkerSize', 8, 'LineWidth', 2);
        view(ax, az, el);
        
        for k = 1:numFrames
            show(robot, trajectory(:, k), 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);
            drawnow;
            pause(0.015);
        end
        
        activeDeg = rad2deg(configSol(activeJointIdx));
        if finalErrorMM <= 15
            btnDispatch.Enable = 'on';
            lblStatus.Value = {
                'SUCCESS: Pose Solved';
                sprintf('Cartesian Error: %.2f mm', finalErrorMM);
                sprintf('Status: %s', solInfo.Status);
                '';
                'Active Angles:';
                sprintf('  J1 (Shoulder):  %6.2f deg', activeDeg(1));
                sprintf('  J2 (Elbow):     %6.2f deg', activeDeg(2));
                sprintf('  J3 (Wrist Roll):%6.2f deg', activeDeg(3));
                sprintf('  J4 (Wrist Pitch):%5.2f deg', activeDeg(4));
                '';
                'Ready to dispatch to Arduino.'
            };
        else
            btnDispatch.Enable = 'off';
            lblStatus.Value = {
                sprintf('WARNING: Error (%.2f mm)', finalErrorMM);
                'Target out of reachable range.';
                'Note: X must remain 0.00 m.';
                'Try: X=0.00, Y=0.25, Z=0.35'
            };
        end
    end

    %% ------------------- LIVE TELEMETRY SERIAL DISPATCH ------------------
    function dispatchToArduino()
        userData = fig.UserData;
        if isempty(userData.solvedConfig)
            return;
        end
        
        btnDispatch.Enable = 'off';
        btnSimulate.Enable = 'off';
        
        armJointsRad = userData.solvedConfig(userData.activeJointIdx);
        armJointsDeg = rad2deg(armJointsRad);
        
        % Packet format: <J1,J2,J3,J4>\n
        cmdString = sprintf('<%.2f,%.2f,%.2f,%.2f>\n', ...
            armJointsDeg(1), armJointsDeg(2), armJointsDeg(3), armJointsDeg(4));
        
        lblStatus.Value = {'Connecting to Arduino...'; 'Please wait.'};
        drawnow;
        
        arduino = [];
        try
            arduino = serialport(comPort, baudRate, "Timeout", 15);
            configureTerminator(arduino, "LF");
            flush(arduino);
            pause(2); % Wait for Arduino serial bootloader
            
            % Send target angles
            writeline(arduino, strtrim(cmdString));
            
            recentLines = {
                'Command Dispatched:';
                strtrim(cmdString);
                '--- LIVE TELEMETRY ---'
            };
            lblStatus.Value = recentLines;
            drawnow;
            
            % Read serial telemetry loop until motion finishes
            motionComplete = false;
            tMotionStart = tic;
            
            while ~motionComplete && (toc(tMotionStart) < 60)
                if arduino.NumBytesAvailable > 0
                    rxLine = readline(arduino);
                    rxLine = strtrim(char(rxLine));
                    
                    if ~isempty(rxLine)
                        if contains(rxLine, '[DONE]')
                            motionComplete = true;
                        end
                        
                        % Keep a rolling 12-line display buffer
                        recentLines = [recentLines; {rxLine}];
                        if numel(recentLines) > 13
                            recentLines = recentLines(end-12:end);
                        end
                        lblStatus.Value = recentLines;
                        drawnow limitrate;
                    end
                else
                    pause(0.015);
                end
            end
            
            if motionComplete
                lblStatus.Value = [lblStatus.Value; {''; 'SUCCESS: Both joints reached target.'}];
            else
                lblStatus.Value = [lblStatus.Value; {''; 'WARNING: Execution timed out.'}];
            end
            
            clear arduino;
        catch ME
            if ~isempty(arduino)
                clear arduino;
            end
            uialert(fig, ME.message, 'Serial Communication Error');
        end
        
        btnDispatch.Enable = 'on';
        btnSimulate.Enable = 'on';
    end

    function resetToHome()
        cla(ax);
        show(robot, qHome, 'Parent', ax, 'PreservePlot', false);
        view(ax, 140, 25);
        btnDispatch.Enable = 'off';
        userData = fig.UserData;
        userData.solvedConfig = [];
        userData.finalErrorMM = Inf;
        fig.UserData = userData;
        lblStatus.Value = {'Arm reset to home position.'};
    end
end