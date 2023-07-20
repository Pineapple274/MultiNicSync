nic1 = rx_7_230717_141324;
nic2 = rx_8_230717_141324;

nic1_phase = cell(1, 3);       
nic2_phase = cell(1, 3);  

packetNum = 1;
dcIndex   = 29;

for i = 1 : 1 :nic1.NumPackets     
    isExistSameOne = false;     
    for j = 1 : 1 :nic2.NumPackets          
        if(nic1.PicoScenesHeader.TaskId(i) == nic2.PicoScenesHeader.TaskId(j))
            nic1_phase{1} = [nic1_phase{1} ; nic1.Phase{1}(i, :)];
            nic1_phase{2} = [nic1_phase{2} ; nic1.Phase{2}(i, :)]; 
            nic1_phase{3} = [nic1_phase{3} ; nic1.Phase{3}(i, :)];
            nic2_phase{1} = [nic2_phase{1} ; nic2.Phase{1}(j, :)];
            nic2_phase{2} = [nic2_phase{2} ; nic2.Phase{2}(j, :)];
            nic2_phase{3} = [nic2_phase{3} ; nic2.Phase{3}(j, :)];
            isExistSameOne = true;         
        end     
    end 
end  

nic1FirstPacketPhase = nic1_phase{2}(packetNum, :); 
nic2FirstPacketPhase = nic2_phase{2}(packetNum, :);     

phase_diff = nic1FirstPacketPhase - nic2FirstPacketPhase; 
% nic2FirstPacketPhase = nic2FirstPacketPhase + phase_diff;

% clear angle;
phaser1 = nic1_phase{1}(packetNum, dcIndex);
phaser2 = nic1_phase{3}(packetNum, dcIndex);
phaser3 = nic1_phase{2}(packetNum, dcIndex);
phaser4 = wrapToPi(nic2_phase{3}(packetNum, dcIndex) + phase_diff(dcIndex));
phaser5 = wrapToPi(nic2_phase{1}(packetNum, dcIndex) + phase_diff(dcIndex));

data = [phaser1 phaser2 phaser3 phaser4 phaser5];

data = exp(1i * (data.'));

radio = 2.68 / 5.147;

multpi_path = 1;

angle = -90 : 1 : 90;

phaser_aoa(data, radio, multpi_path, angle);
