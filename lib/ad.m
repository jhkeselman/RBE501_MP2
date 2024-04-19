function adV = ad(V)
    % AD Given a twist, calculate the corresponding 6x6 lie bracket matrix.
    %   ADV = AD(V) calculates the 6x6 lie bracket matrix of V.
    w = V(1:3);
    v = V(4:6);
    adV = [skew(w) zeros(3,3); skew(v) skew(w)];
end