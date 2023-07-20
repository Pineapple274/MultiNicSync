function [spectrum_aoa, angles] = phaser_aoa(data, dist_antenna_lambda_ratio, n_multipath, angles) 

n_antennas = size(data, 1); 
steeringvs = makeAoASteeringVectors(n_antennas, dist_antenna_lambda_ratio, angles); 
spectrum_aoa = music_core(data, steeringvs, n_multipath);  

if nargout == 0     
    figure;
    plot(angles, spectrum_aoa); 
end