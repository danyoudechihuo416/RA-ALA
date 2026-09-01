function provenance = validationProvenance(cohortFile)
%VALIDATIONPROVENANCE Fingerprint scientific source and optional input archive.
    root = fileparts(mfilename('fullpath'));
    files = {'UnifiedCostModel.m','computeTrackHoldingKinematics.m', ...
        'solveArrivalTimeStep.m', ...
        'PathPlanners.m','CityEnvironment.m','runRA_ALA.m', ...
        'evaluateRAALASearchFitness.m','computeApproxTArr.m','paramToPath.m', ...
        'repairRawPath.m','smoothPathSpline.m','mildSmoothPath.m', ...
        'postSmoothRepair.m','estimateEnvDifficulty.m'};
    hashes = cell(size(files));
    for i = 1:numel(files)
        source = strrep(fileread(fullfile(root,files{i})),sprintf('\r\n'),sprintf('\n'));
        hashes{i} = localHash(unicode2native(source,'UTF-8'));
    end
    provenance = struct('schema',1,'files',{files},'sha256',{hashes});
    if nargin > 0 && ~isempty(cohortFile)
        fid = fopen(cohortFile,'rb');
        if fid < 0, error('Validation:MissingCohort','Cannot read %s.',cohortFile); end
        cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
        digest = java.security.MessageDigest.getInstance('SHA-256');
        while ~feof(fid)
            bytes = fread(fid,1024*1024,'*uint8');
            if ~isempty(bytes), digest.update(bytes); end
        end
        provenance.cohort_sha256 = localHex(digest.digest());
    end
end

function hash = localHash(bytes)
    digest = java.security.MessageDigest.getInstance('SHA-256');
    digest.update(bytes);
    hash = localHex(digest.digest());
end

function hash = localHex(bytes)
    hash = lower(reshape(dec2hex(typecast(bytes,'uint8'),2).',1,[]));
end
