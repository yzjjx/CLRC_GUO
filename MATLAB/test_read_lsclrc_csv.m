function tests = test_read_lsclrc_csv
% 在 MATLAB/ 中运行 runtests('test_read_lsclrc_csv.m')，无需生成图。
tests = functiontests(localfunctions);
end

function setup(testCase)
testCase.TestData.path = [tempname, '.csv'];
end

function teardown(testCase)
if isfile(testCase.TestData.path)
    delete(testCase.TestData.path);
end
end

function testCompleteCSV(testCase)
write_csv(testCase, sprintf('t,q1,diagnostic\n0,1,0\n1,2,3\n'));
data = read_lsclrc_csv(testCase.TestData.path, {'t', 'q1'}, {'diagnostic'});
verifyEqual(testCase, data.t, [0; 1]);
verifyEqual(testCase, data.q1, [1; 2]);
end

function testTruncatedTail(testCase)
contents = sprintf('t,q1,diagnostic\n0,1,0\n1,2,3\n2,');
write_csv(testCase, contents);
verifyWarning(testCase, @() read_lsclrc_csv( ...
    testCase.TestData.path, {'t', 'q1'}, {'diagnostic'}), 'LSCLRC:IncompleteTail');
previous = warning('off', 'LSCLRC:IncompleteTail');
restore_warning = onCleanup(@() warning(previous)); %#ok<NASGU>
data = read_lsclrc_csv(testCase.TestData.path, {'t', 'q1'}, {'diagnostic'});
verifyEqual(testCase, height(data), 2);
verifyEqual(testCase, data.t(end), 1);
verifyEqual(testCase, fileread(testCase.TestData.path), contents);
end

function testMissingOptionalTail(testCase)
write_csv(testCase, sprintf('t,q1,diagnostic\n0,1,0\n1,2,3\n2,4,\n'));
verifyWarning(testCase, @() read_lsclrc_csv( ...
    testCase.TestData.path, {'t', 'q1'}, {'diagnostic'}), 'LSCLRC:IncompleteTail');
end

function testInteriorNaN(testCase)
write_csv(testCase, sprintf('t,q1\n0,1\n1,NaN\n2,3\n'));
verifyError(testCase, @() read_lsclrc_csv( ...
    testCase.TestData.path, {'t', 'q1'}), 'LSCLRC:InvalidData');
end

function testInfiniteTail(testCase)
write_csv(testCase, sprintf('t,q1\n0,1\n1,2\n2,Inf\n'));
verifyError(testCase, @() read_lsclrc_csv( ...
    testCase.TestData.path, {'t', 'q1'}), 'LSCLRC:InvalidData');
end

function testMissingRequiredColumn(testCase)
write_csv(testCase, sprintf('t\n0\n1\n'));
verifyError(testCase, @() read_lsclrc_csv( ...
    testCase.TestData.path, {'t', 'q1'}), 'LSCLRC:MissingColumns');
end

function testNonIncreasingTime(testCase)
write_csv(testCase, sprintf('t,q1\n0,1\n0,2\n'));
verifyError(testCase, @() read_lsclrc_csv( ...
    testCase.TestData.path, {'t', 'q1'}), 'LSCLRC:InvalidTime');
end

function write_csv(testCase, contents)
fid = fopen(testCase.TestData.path, 'w');
assert(fid ~= -1, 'Cannot create temporary CSV.');
close_file = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s', contents);
end
