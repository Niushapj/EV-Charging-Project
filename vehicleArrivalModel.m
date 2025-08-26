dt_min = 10; dt = dt_min/60;
T = 24*60/dt_min;
t = (1:T)';

%% VEHICLE ARRIVAL MODULE (Poisson with fixed dataset size)
lambda = 10;           % average arrivals per hour
Vmax   = 100;           % fixed dataset size (maximum vehicles)

% Expected arrivals per slot (dt_min minutes)
lambda_slot = lambda * dt;  

% Generate arrivals per slot
arrivals_per_slot = poissrnd(lambda_slot, T, 1);

% Convert to individual vehicle arrival times
arrivalTimes = [];
for tt = 1:T
    if arrivals_per_slot(tt) > 0
        arrivalTimes = [arrivalTimes; tt*ones(arrivals_per_slot(tt),1)];
    end
end

% Keep dataset fixed in size
if length(arrivalTimes) > Vmax
    arrivalTimes = arrivalTimes(1:Vmax);
end


V = length(arrivalTimes); % number of vehicles
departureTimes = arrivalTimes + randi([6,30],V,1);
departureTimes = min(departureTimes,T);