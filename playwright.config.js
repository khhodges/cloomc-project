'use strict';

const { defineConfig, devices } = require('@playwright/test');

module.exports = defineConfig({
    testDir: './tests/e2e',
    timeout: 15000,
    expect: {
        timeout: 5000,
    },
    use: {
        baseURL: 'http://localhost:5000',
        headless: true,
    },
    projects: [
        {
            name: 'chromium',
            use: { ...devices['Desktop Chrome'] },
        },
    ],
});
