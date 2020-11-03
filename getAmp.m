function Amp = getAmp(response)

maxIndex = find( islocalmax(response) );
minIndex = find( islocalmin(response) );

averageMax = mean( response(maxIndex) );
averageMin = mean( response(minIndex) );

Amp = averageMax - averageMin;