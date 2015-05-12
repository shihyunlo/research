function out = shortest_angular_distance(from, to)
out = unroll(unroll(to) - unroll(from));
if (out > pi)
    out = -(2*pi - out);
end
out = normalize(out);
