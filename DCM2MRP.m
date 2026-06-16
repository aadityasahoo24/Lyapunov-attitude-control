function mrp = DCM2MRP(C)
    x = sqrt(trace(C)+1);
    mrp = (1/(x*(x+2)))*[C(2,3) - C(3,2);
    C(3,1) - C(1,3);
    C(1,2) - C(2,1)];

    if norm(mrp) > 1
        mrp = -mrp/norm(mrp)^2
    end
    
end