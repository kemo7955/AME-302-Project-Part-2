function AvgFreq = getFrequency(Time)
    for i = 1:length(Time)-1
        deltaT(i) = abs( Time(i) - Time(i+1) );
    end
    AvgT = mean(deltaT);
    AvgFreq = 1/AvgT;