success = 0;
failure = 0;
for i=1:100
    target = fkine(S, M, pi * rand(1, 6) - pi / 2, 'space');
    start_q = pi * rand(1, 6) - pi / 2;
    if size(ikine(S, M, start_q, target), 2) == 6
        success = success + 1
    else
        failure = failure + 1
    end
end