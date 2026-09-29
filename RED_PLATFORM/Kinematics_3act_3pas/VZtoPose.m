function q = VZtoPose(p, VZ, EA_deg)
% Обратно к poseToVZ
EA = EA_deg*pi/180;
o = VZ(:) - rotEA(EA)*[0;0;p.z_off];
q = [o' EA];
end
