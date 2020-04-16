% Define an empty scenario

scenario = drivingScenario;
scenario.SampleTime = 0.01;

% Add a stretch of 500 meters of typical highway road with two lanes. 
% The road is defined using a set of points, where each point defines the center of the 
% road in 3-D space, and a road width.

roadCenters = [0 0; 50 0; 100 0; 250 20; 500 40];
road(scenario, roadCenters, 'lanes',lanespec(2));

% Create the ego vehicle and three cars around it: one that overtakes the ego vehicle 
% and passes it on the left, one that drives right in front of the ego vehicle and 
% one that drives right behind the ego vehicle. 
% All the cars follow the path defined by the road waypoints by using the path driving 
% policy. The passing car will start on the right lane, move to the left lane to pass, 
% and return to the right lane.

egoCar = vehicle(scenario, 'ClassID', 1);
trajectory(egoCar, roadCenters(2:end,:) - [0 1.8], 25); % On right lane

% Add a car in front of the ego vehicle
leadCar = vehicle(scenario, 'ClassID', 1);
trajectory(leadCar, [70 0; roadCenters(3:end,:)] - [0 1.8], 25); % On right lane

% Add a car that travels at 35 m/s along the road and passes the ego vehicle
passingCar = vehicle(scenario, 'ClassID', 1);
waypoints = [0 -1.8; 50 1.8; 100 1.8; 250 21.8; 400 32.2; 500 38.2];
trajectory(passingCar, waypoints, 35);

% Add a car behind the ego vehicle
chaseCar = vehicle(scenario, 'ClassID', 1);
trajectory(chaseCar, [25 0; roadCenters(2:end,:)] - [0 1.8], 25); % On right lane

sensors = cell(6,1);

% Front-facing long-range radar sensor at the center of the front bumper of the car.
% sensors{1} = radarDetectionGenerator(...
%     'SensorIndex', 1, ...
%     'Height', 0.2, ...
%     'MaxRange', 100, ...
%     'SensorLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0], ...
%     'FieldOfView', [40, 5]);
% 
% sensors{2} = radarDetectionGenerator(...
%     'SensorIndex', 2, ...
%     'Height', 0.2, ...
%     'Yaw', 62, ...
%     'MaxRange', 50, ...
%     'SensorLocation', [egoCar.Wheelbase, egoCar.FrontOverhang], ...
%     'FieldOfView', [90, 5]);
% 
% sensors{3} = radarDetectionGenerator(...
%     'SensorIndex', 3, ...
%     'Height', 0.2, ...
%     'Yaw', -62, ...
%     'MaxRange', 50, ...
%     'SensorLocation', [egoCar.Wheelbase, -egoCar.FrontOverhang], ...
%     'FieldOfView', [90, 5]);
% 
% sensors{4} = radarDetectionGenerator(...
%     'SensorIndex', 4, ...
%     'Height', 0.2, ...
%     'Yaw', 114, ...
%     'MaxRange', 50, ...
%     'SensorLocation', [0, egoCar.FrontOverhang], ...
%     'FieldOfView', [90, 5]);
% 
% sensors{5} = radarDetectionGenerator(...
%     'SensorIndex', 5, ...
%     'Height', 0.2, ...
%     'Yaw', -114, ...
%     'MaxRange', 50, ...
%     'SensorLocation', [0, -egoCar.FrontOverhang], ...
%     'FieldOfView', [90, 5]);
% 
% sensors{6} = radarDetectionGenerator(...
%     'SensorIndex', 6, ...
%     'Height', 0.2, ...
%     'Yaw', -180, ...
%     'MaxRange', 100, ...
%     'SensorLocation', [egoCar.RearOverhang, 0], ...
%     'FieldOfView', [20, 5]);

% Front-facing long-range radar sensor at the center of the front bumper of the car.
sensors{1} = radarDetectionGenerator('SensorIndex', 1, 'Height', 0.2, 'MaxRange', 174, ...
    'SensorLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0], 'FieldOfView', [20, 5]);

% Rear-facing long-range radar sensor at the center of the rear bumper of the car.
sensors{2} = radarDetectionGenerator('SensorIndex', 2, 'Height', 0.2, 'Yaw', 180, ...
    'SensorLocation', [-egoCar.RearOverhang, 0], 'MaxRange', 174, 'FieldOfView', [20, 5]);

% Rear-left-facing short-range radar sensor at the left rear wheel well of the car.
sensors{3} = radarDetectionGenerator('SensorIndex', 3, 'Height', 0.2, 'Yaw', 120, ...
    'SensorLocation', [0, egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Rear-right-facing short-range radar sensor at the right rear wheel well of the car.
sensors{4} = radarDetectionGenerator('SensorIndex', 4, 'Height', 0.2, 'Yaw', -120, ...
    'SensorLocation', [0, -egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);

% Front-left-facing short-range radar sensor at the left front wheel well of the car.
sensors{5} = radarDetectionGenerator('SensorIndex', 5, 'Height', 0.2, 'Yaw', 60, ...
    'SensorLocation', [egoCar.Wheelbase, egoCar.Width/2], 'MaxRange', 30, ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], 'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

% Front-right-facing short-range radar sensor at the right front wheel well of the car.
sensors{6} = radarDetectionGenerator('SensorIndex', 6, 'Height', 0.2, 'Yaw', -60, ...
    'SensorLocation', [egoCar.Wheelbase, -egoCar.Width/2], 'MaxRange', 30, ...
    'ReferenceRange', 50, 'FieldOfView', [90, 5], 'AzimuthResolution', 10, ...
    'RangeResolution', 1.25);

% Register actor profiles with the sensors.
profiles = actorProfiles(scenario);
for m = 1:numel(sensors)
    sensors{m}.ActorProfiles = profiles;
end

 %% Tracker implementation
 
 % Create a multiObjectTracker to track the vehicles that are close to the ego vehicle. 
 % The tracker uses the initSimDemoFilter supporting function to initialize a constant 
 % velocity linear Kalman filter that works with position and velocity.
 
tracker = multiObjectTracker(...
    'FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, ...
    'ConfirmationParameters', [4 5]);

positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

% create demo display and return a handle to the bird-eye view
BEP = createDemoDisplay(egoCar, sensors);

%% The following loop moves the vehicles, calls the sensor simulation, and performs the tracking.
toSnap = true;
while advance(scenario) && ishghandle(BEP.Parent)    
    % Get the scenario time
    time = scenario.SimulationTime;

    % Get the position of the other vehicle in ego vehicle coordinates
    ta = targetPoses(egoCar);

    % Simulate the sensors
    detections = {};
    isValidTime = false(1,6);
    for i = 1:6
        [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
        if numValidDets
            detections = [detections; sensorDets]; %#ok<AGROW>
        end
    end

    % Update the tracker if there are new detections
    if any(isValidTime)
        vehicleLength = sensors{1}.ActorProfiles.Length;
        detectionClusters = clusterDetections(detections, vehicleLength);
        confirmedTracks = updateTracks(tracker, detectionClusters, time);

        % Update bird's-eye plot
        updateBEP(BEP, egoCar, detections, confirmedTracks, positionSelector, velocitySelector);
    end

    % Snap a figure for the document when the car passes the ego vehicle
    if ta(1).Position(1) > 0 && toSnap
        toSnap = false;
        snapnow
    end
end
