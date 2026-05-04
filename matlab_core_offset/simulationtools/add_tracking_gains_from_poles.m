function p = add_tracking_gains_from_poles(p, poles_by_channel)
    for i = 1:numel(poles_by_channel)
        kij = gains_from_poles(poles_by_channel{i});
        for j = 1:numel(kij)
            p.(sprintf('k__%d__%d', i, j)) = kij(j);
        end
    end
end

function kij = gains_from_poles(poles)
    c = poly(poles);
    kij = fliplr(real(c(2:end)));
end