'use strict';

// lump_warning.spec.js — Playwright E2E test for the "no LUMP" warning toast.
//
// Verifies the full browser-side plumbing:
//   double-click on an abstraction row
//     → _goToLumpByAbstractionName()
//     → _showFpgaToast() renders the toast in the real DOM
//     → toast auto-dismisses after ~2 s
//
// The /api/lumps/list route is intercepted to return an empty list so the test
// is deterministic regardless of which LUMPs happen to be compiled locally.

const { test, expect } = require('@playwright/test');

test.describe('no-LUMP warning toast', () => {

    test.beforeEach(async ({ page }) => {
        // Intercept the lumps list API so the warm/cold cache both resolve to
        // "no lumps exist", guaranteeing the warning path is taken for any
        // abstraction the test double-clicks.
        await page.route('**/api/lumps/list', async route => {
            await route.fulfill({
                status: 200,
                contentType: 'application/json',
                body: JSON.stringify([]),
            });
        });

        await page.goto('/');
        // Wait for the page JS to finish loading.
        await page.waitForLoadState('networkidle');
    });

    test('shows "No compiled LUMP found" toast after double-clicking an abstraction with no LUMP', async ({ page }) => {
        // Navigate to the Abstractions view via the hamburger menu button.
        const absBtn = page.locator('#hamItem-abstractions');
        await absBtn.waitFor({ state: 'visible' });
        await absBtn.click();

        // Wait for at least one abstraction row to render.
        const firstRow = page.locator('.abs-item').first();
        await firstRow.waitFor({ state: 'visible' });

        // Double-click fires the ondblclick handler which calls
        // _goToLumpByAbstractionName(). The route intercept above ensures the
        // lumps list is empty, so the warning toast should appear.
        await firstRow.dblclick();

        // ── Assert 1: toast is visible with the expected text ─────────────────
        const toast = page.locator('#fpgaToastEl');
        await expect(toast).toBeVisible();

        const body = toast.locator('.fpga-toast-body');
        await expect(body).toContainText('No compiled LUMP found');

        // ── Assert 2: toast carries the warning CSS class ─────────────────────
        await expect(toast).toHaveClass(/fpga-toast-warn/);

        // ── Assert 3: toast auto-dismisses within 3 s ────────────────────────
        // _showFpgaToast schedules a fade at 2000 ms and element removal
        // 400 ms later, so the element should be gone well within 3 s.
        await expect(toast).not.toBeVisible({ timeout: 3000 });
    });

});
