success = 0;
failure = 0;
for i=1:100
    target_q = pi * rand(1, 6) - pi / 2;
    target = fkine(S, M, target_q, 'space');
    start_q = pi * rand(1, 6) - pi / 2;
    if size(ikine(S, M, start_q, target), 2) == 6
        success = success + 1
    else
        failure = failure + 1
        det(jacob0(S, target_q))
    end
end