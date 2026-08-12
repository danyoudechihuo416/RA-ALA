function p = wilcoxonSignedRank(x, y)
% Wilcoxon signed-rank 配对检验（双边）
    diffs = x(:) - y(:);
    diffs(abs(diffs) < 1e-12) = [];
    n = length(diffs);
    if n < 3, p = 1; return; end
    [~, sortIdx] = sort(abs(diffs));
    ranks = zeros(n,1);
    i = 1;
    while i <= n
        j = i;
        while j < n && abs(abs(diffs(sortIdx(j+1)))-abs(diffs(sortIdx(j)))) < 1e-10
            j = j + 1;
        end
        for k = i:j, ranks(sortIdx(k)) = mean(i:j); end
        i = j + 1;
    end
    W = min(sum(ranks(diffs>0)), sum(ranks(diffs<0)));
    if n >= 10
        z = (W - n*(n+1)/4) / sqrt(n*(n+1)*(2*n+1)/24);
        p = 2 * (1 - normcdf(abs(z)));
    else
        cnt = 0;
        for mask = 0:2^n-1
            Tp=0; Tn=0;
            for b=1:n
                if bitand(mask,2^(b-1)), Tp=Tp+b; else, Tn=Tn+b; end
            end
            if min(Tp,Tn)<=W, cnt=cnt+1; end
        end
        p = cnt/2^n;
    end
    p = max(min(p,1), 1e-300);
end

