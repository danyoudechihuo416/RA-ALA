function [pngFile,pdfFile] = exportPublicationFigure(fig, outputFile, ppi)
%EXPORTPUBLICATIONFIGURE Export a figure for manuscript submission.
%   Writes a high-resolution PNG and a same-name vector PDF. The requested
%   PPI applies to the raster PNG; vector objects in the PDF are resolution
%   independent.

    if nargin < 3 || isempty(ppi)
        ppi = 600;
    end
    validateattributes(ppi,{'numeric'},{'scalar','finite','positive'});
    if ~isgraphics(fig)
        error('PublicationFigure:InvalidHandle','fig must be a valid graphics handle.');
    end

    [outDir,baseName,~] = fileparts(char(outputFile));
    if isempty(outDir)
        outDir = pwd;
    end
    if isempty(baseName)
        error('PublicationFigure:InvalidName','outputFile must contain a file name.');
    end
    if ~exist(outDir,'dir')
        mkdir(outDir);
    end

    pngFile = fullfile(outDir,[baseName,'.png']);
    pdfFile = fullfile(outDir,[baseName,'.pdf']);
    drawnow;

    exportgraphics(fig,pngFile,'Resolution',ppi, ...
        'BackgroundColor','white');
    exportgraphics(fig,pdfFile,'ContentType','vector', ...
        'BackgroundColor','white');

    fprintf('  Publication figure saved: %s (%g PPI) | %s (vector PDF)\n', ...
        pngFile,ppi,pdfFile);
end
