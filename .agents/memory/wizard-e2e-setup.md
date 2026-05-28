---
name: Startup Wizard E2E setup
description: Where the wizard panel lives and how to make it truly visible in Playwright tests.
---

## Wizard DOM location

`#swPanel` is inside `#builder > #ti60ConnectPanel`. Both `#builder` (a `.view`
div hidden when another view is active) and `#ti60ConnectPanel` start with
`style="display:none;"`. Any Playwright test that touches wizard elements must
first navigate there:

```javascript
await page.evaluate(() => {
    switchView('builder');
    switchBuilderViewTab('ti60-connect');
});
```

`switchView` is in `app-shell.js` (sync); `switchBuilderViewTab` in
`app-run.js` (sync, line ~9753). Both are global and available after
`networkidle`.

## display:'' vs display:'block'

CSS has `.sw-body-step { display: none; }`. Setting
`element.style.display = ''` removes the inline property and falls back to
that CSS rule — the element stays hidden. Use `'block'` explicitly when you
want the element to be truly visible (this was the bug in `_renderProgress()`).

**Why:** Playwright's `toBeVisible()` fails when computed display is `none`,
even if the element has a non-empty inline style.

**How to apply:** Whenever setting `style.display` to "show" an element that a
CSS class hides, use `'block'` (or the appropriate block type), not `''`.

## async vs defer for init()

The wizard IIFE registers `document.addEventListener('DOMContentLoaded', init)`.
With `<script async>`, the script may execute after DOMContentLoaded has already
fired, so `init()` never runs. Use `<script defer>` so the script executes
before DOMContentLoaded, guaranteeing `init()` is called.
