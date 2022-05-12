function [qmat] = compute_qmat(t_arr, celld2s, weight)
%compute_qmat 计算q矩阵

[r, c] = size(t_arr);
k = r - 1;
n = cell2mat(celld2s(6));

snap = celld2s(5);
snap = cell2mat(snap);
snapfac = snap(:, 1);
snappow = snap(:, 2);
snapfacmat = snapfac * snapfac';
snappowmat = zeros(n+1, n+1);
snappowt = snappow';
for i = 1 : n+1
    for j = 1 : n + 1
        a = snappow(i, 1) + snappowt(1, j);
        snappowmat(i, j) = a;
    end
end
snappowmat = snappowmat + 1;
snapfacmat = snapfacmat ./ snappowmat;

qmat = [];
for i = 2 : 1 : k+1
    t_curr = t_arr(i);
    t_last = t_arr(i-1);
    qi = snapfacmat.*(t_curr.^snappowmat) - snapfacmat.*(t_last.^snappowmat);
    wi = ones(n+1, n+1) * weight(i-1);
    qi = qi .* wi;
    qmat = blkdiag(qmat, qi);
end

end