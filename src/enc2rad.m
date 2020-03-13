% Convert Encoder Values to Angles in Radians
function angle = enc2rad(enc)
    angle = (enc*2*pi)/4095;
end