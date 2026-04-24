const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');

const SIMULATOR_DIR = __dirname;

function discoverTestFiles() {
  return fs.readdirSync(SIMULATOR_DIR)
    .filter(f => f.endsWith('_test.js'))
    .map(f => path.resolve(SIMULATOR_DIR, f));
}

function sourceFileForTest(testFile) {
  const base = path.basename(testFile, '_test.js') + '.js';
  const candidate = path.resolve(SIMULATOR_DIR, base);
  return fs.existsSync(candidate) ? candidate : null;
}

const runningTests = new Set();
const pendingTests = new Set();
const debounceTimers = {};

function runTest(testFile) {
  const label = path.basename(testFile);

  if (runningTests.has(testFile)) {
    pendingTests.add(testFile);
    return;
  }

  runningTests.add(testFile);
  pendingTests.delete(testFile);

  const timestamp = new Date().toISOString();
  console.log(`\n[${timestamp}] Running ${label}...`);

  const child = spawn('node', [testFile], {
    stdio: 'inherit',
    cwd: path.resolve(SIMULATOR_DIR, '..'),
  });

  child.on('close', (code) => {
    const ts = new Date().toISOString();
    if (code === 0) {
      console.log(`[${ts}] ${label}: passed.`);
    } else {
      console.log(`[${ts}] ${label}: FAILED (exit code ${code}).`);
    }
    runningTests.delete(testFile);
    if (pendingTests.has(testFile)) {
      runTest(testFile);
    }
  });
}

function scheduleTest(testFile, changedFile) {
  clearTimeout(debounceTimers[testFile]);
  debounceTimers[testFile] = setTimeout(() => {
    console.log(`\nChange detected in: ${path.basename(changedFile)} — scheduling ${path.basename(testFile)}`);
    runTest(testFile);
  }, 200);
}

function watchFile(filePath, testFile) {
  if (!fs.existsSync(filePath)) {
    console.warn(`Warning: watch target does not exist: ${filePath}`);
    return;
  }
  fs.watch(filePath, () => scheduleTest(testFile, filePath));
  console.log(`Watching: ${filePath}`);
}

const testFiles = discoverTestFiles();

if (testFiles.length === 0) {
  console.log('No *_test.js files found in simulator/. Nothing to watch.');
  process.exit(0);
}

for (const testFile of testFiles) {
  const sourceFile = sourceFileForTest(testFile);
  if (sourceFile) {
    watchFile(sourceFile, testFile);
  }
  watchFile(testFile, testFile);
}

console.log(`\nSimulator test watcher started (${testFiles.length} test file(s) found). Running all tests now...\n`);
for (const testFile of testFiles) {
  runTest(testFile);
}
