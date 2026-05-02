function angle = normalize_angle(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end
