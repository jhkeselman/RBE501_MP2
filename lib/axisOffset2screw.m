% V = AXISOFFSET2SCREW(omega, p) produces the screw axis defined by axis of
% rotation omega and the displacement from the origin p.
function S = axisOffset2screw(omega, p)
    v = -skew(omega) * p;
    S = [omega; v];
end
