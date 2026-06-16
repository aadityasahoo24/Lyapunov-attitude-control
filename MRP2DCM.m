function C = MRP2DCM(sigma)
    mrp = sigma(:);
    sigma_sq = mrp' * mrp;
    sigma_tilde = [  0,      -mrp(3),  mrp(2);
                    mrp(3),   0,      -mrp(1);
                   -mrp(2),  mrp(1),   0     ];
    C = eye(3) + (8 * (sigma_tilde^2) - 4 * (1 - sigma_sq) * sigma_tilde) / (1 + sigma_sq)^2;
end