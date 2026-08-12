function s = formatPValue(p)
    if isnan(p),      s = 'N/A';
    elseif p < 0.001, s = '<.001***';
    elseif p < 0.01,  s = sprintf('%.3f**',p);
    elseif p < 0.05,  s = sprintf('%.3f*',p);
    else,             s = sprintf('%.3f ns',p);
    end
end
