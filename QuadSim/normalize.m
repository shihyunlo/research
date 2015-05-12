function out = normalize(x)
out = mod(x + pi, 2*pi);
if out < 0
    out = out + 2*pi;
end
out = out - pi;
