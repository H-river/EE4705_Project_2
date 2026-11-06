function zipFile = make_submission(reportPdf, outDir, includeResults)
% MAKE_SUBMISSION  Build Group9_Report_Code.zip for the final submission.
%
%   make_submission('path/to/Group9_Report.pdf')               % zip lands in the repo root
%   make_submission('path/to/Group9_Report.pdf', outDir)
%   make_submission('path/to/Group9_Report.pdf', outDir, false) % code only, no results
%
% The zip contains one folder, Group9_Report_Code/, with
%   * every file tracked by git (code, README, original_code/) -- so untracked course PDFs
%     are left out automatically;
%   * minus the team-internal files: TODO.md, tools/, .gitignore, .gitattributes;
%   * plus the report PDF (if given);
%   * plus (includeResults, default true) the figures, tables and videos of every
%     <folder>/results/ (*.png, *.csv, *.mp4 -- not the .mat / .avi files), so the
%     graders can see the results without running anything. Run all task scripts and
%     Videos/make_videos.m on the final code first: the function refuses to pack a
%     results folder whose newest file is older than the newest commit of a .m file.
% It refuses to run if any TODO[ tag is left in the code, and warns about uncommitted
% changes (the zip takes the working-tree version of tracked files).
%
% Needs git on the system path. Not part of the submission itself (tools/ is excluded).

root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 3 || isempty(includeResults), includeResults = true; end
if nargin < 2 || isempty(outDir), outDir = root; end
if nargin < 1, reportPdf = ''; end
name = 'Group9_Report_Code';

[st, out] = system(sprintf('git -C "%s" ls-files', root));
if st ~= 0, error('make_submission: git ls-files failed:\n%s', out); end
files = splitlines(strtrim(out));
files = files(~cellfun(@isempty, files));
skip = startsWith(files, 'tools/') | ismember(files, {'TODO.md', '.gitignore', '.gitattributes'});
files = files(~skip);

[~, dirty] = system(sprintf('git -C "%s" status --porcelain --untracked-files=no', root));
if ~isempty(strtrim(dirty))
    warning('make_submission: uncommitted changes in tracked files are included:\n%s', dirty);
end

% No leftover task-tracking tags in the submitted code
isM = endsWith(files, '.m');
for k = find(isM)'
    if contains(fileread(fullfile(root, files{k})), 'TODO[')
        error('make_submission: %s still contains a TODO[ tag.', files{k});
    end
end

% Result files (figures / tables / videos), relative paths like Task4_Scenarios/results/x.png
resFiles = {};
if includeResults
    [~, tCommit] = system(sprintf('git -C "%s" log -1 --format=%%ct -- "*.m"', root));
    tCommit = str2double(strtrim(tCommit));                       % newest code commit, unix time
    d = [dir(fullfile(root, 'Task*')); dir(fullfile(root, 'Bonus*'))];
    names = [{d([d.isdir]).name}, {'Videos'}];                    % folders with a results/
    folders = struct('name', names);
    for i = 1:numel(folders)
        rdir = fullfile(root, folders(i).name, 'results');
        if ~isfolder(rdir)
            warning('make_submission: %s has no results/ -- run its script first.', folders(i).name);
            continue
        end
        d = [dir(fullfile(rdir, '*.png')); dir(fullfile(rdir, '*.csv')); dir(fullfile(rdir, '*.mp4'))];
        if isempty(d), continue; end
        tNewest = posixtime(datetime(max([d.datenum]), 'ConvertFrom', 'datenum', 'TimeZone', 'local'));
        if tNewest < tCommit
            error(['make_submission: %s/results is older than the newest commit -- re-run the ' ...
                   'script on the final code (or pass includeResults = false).'], folders(i).name);
        end
        for j = 1:numel(d)
            resFiles{end + 1} = [folders(i).name '/results/' d(j).name]; %#ok<AGROW>
        end
    end
end

stage = fullfile(tempname, name);
for k = 1:numel(files)
    dst = fullfile(stage, files{k});
    if ~exist(fileparts(dst), 'dir'), mkdir(fileparts(dst)); end
    copyfile(fullfile(root, files{k}), dst);
end
for k = 1:numel(resFiles)
    dst = fullfile(stage, resFiles{k});
    if ~exist(fileparts(dst), 'dir'), mkdir(fileparts(dst)); end
    copyfile(fullfile(root, resFiles{k}), dst);
end
if ~isempty(reportPdf)
    if ~isfile(reportPdf), error('make_submission: report "%s" not found.', reportPdf); end
    copyfile(reportPdf, fullfile(stage, [name(1:end-5) '.pdf']));   % Group9_Report.pdf
else
    warning('make_submission: no report PDF given -- the zip contains the code only.');
end

zipFile = fullfile(outDir, [name '.zip']);
zip(zipFile, name, fileparts(stage));
rmdir(fileparts(stage), 's');
z = dir(zipFile);
fprintf('make_submission: %s (%d code files, %d result files%s, %.1f MB)\n', zipFile, ...
    numel(files), numel(resFiles), repmat(' + report', 1, ~isempty(reportPdf)), z.bytes / 2^20);
end
