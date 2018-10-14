for m = 1:400
    s = step(Hsin); % Generate sine waves with random phase
    x = sum(s,2);   % Generate synthetic noise by adding all sine waves
    d = step(Hfir,x) + ...  % Propagate noise through primary path
        0.1*randn(size(x)); % Add measurement noise
    if m <= 200
        % No noise control for first 200 iterations
        e = d;
    else
        % Enable active noise control after 200 iterations
        xhat = x + 0.1*randn(size(x));
        [y,e] = step(Hfx,xhat,d);
    end
    step(Hpa,e);     % Play noise signal
    step(Hsa,[d,e]); % Show spectrum of original (Channel 1)
                     % and attenuated noise (Channel 2)
end
release(Hpa); % Release audio device
release(Hsa); % Release spectrum analyzer