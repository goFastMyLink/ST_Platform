function [VZ, EA_deg] = poseToVZ(p, q)
% Из позы центра шарниров платформы -> точка выходного звена VZ и углы Эйлера (град)
Rot = rotEA(q(4:6));
VZ = (q(1:3)' + Rot*[0;0;p.z_off])';
EA_deg = q(4:6)*180/pi;
end
