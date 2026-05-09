/*
 * app-source-library.js — Source Library panel for the Abstractions view
 *
 * Displays all example scaffolds in one scrollable, searchable panel,
 * grouped by language. Each card shows the abstraction name, language badge,
 * method count, and full source in a read-only code block. A "Load" button
 * pushes the source into the editor exactly as the example tab buttons do.
 *
 * Data sources:
 *   window._cloomcExampleSources  — inline CLOOMC/Haskell/Symbolic/English/Lambda examples
 *   window._cloomcFileExamples    — url map for file-based CLOOMC examples
 *   window._cloomcFileLanguages   — lang map for file-based CLOOMC examples
 *   window._asmExampleSources     — inline Assembly examples
 *
 * These globals are populated on first call to loadCLOOMCExample() /
 * loadExample() respectively (patched in app-compile.js / app-run.js).
 */

'use strict';

/* ── Registry ────────────────────────────────────────────────────────────── */

const _SL_REGISTRY = [
    /* ── CLOOMC++ (JavaScript front-end) ─────────────────── */
    { key: 'integer_ops',      name: 'IntegerOps',           lang: 'javascript', loader: 'cloomc' },
    { key: 'packed_string',    name: 'PackedString',          lang: 'javascript', loader: 'cloomc' },
    { key: 'memory',           name: 'Memory',                lang: 'javascript', loader: 'cloomc' },
    { key: 'heap',             name: 'Heap',                  lang: 'javascript', loader: 'cloomc' },
    { key: 'mint',             name: 'Mint',                  lang: 'javascript', loader: 'cloomc' },
    { key: 'sliderule',        name: 'SlideRule',             lang: 'javascript', loader: 'cloomc' },
    { key: 'contact',          name: 'Contact',               lang: 'javascript', loader: 'cloomc' },
    { key: 'contact_stage2',   name: 'ContactStage2',         lang: 'javascript', loader: 'cloomc' },
    { key: 'contact_call',     name: 'ContactCall',           lang: 'javascript', loader: 'cloomc' },
    { key: 'stack_overflow',   name: 'StackOverflow',         lang: 'javascript', loader: 'cloomc' },
    { key: 'recall_demo',      name: 'Feedback',              lang: 'javascript', loader: 'cloomc' },
    { key: 'billing',          name: 'BudgetTracker',         lang: 'javascript', loader: 'cloomc' },
    { key: 'turing_memory',    name: 'TuringMemory',          lang: 'javascript', loader: 'cloomc' },
    { key: 'church_memory',    name: 'ChurchMemory',          lang: 'javascript', loader: 'cloomc' },
    { key: 'physical_pool',    name: 'DMABuffer',             lang: 'javascript', loader: 'cloomc' },

    /* ── Assembly ─────────────────────────────────────────── */
    { key: 'capability_test',  name: 'CapabilityTest',        lang: 'assembly',   loader: 'assembly' },
    { key: 'system_patterns',  name: 'SystemPatterns',        lang: 'assembly',   loader: 'assembly' },
    { key: 'compute_demo',     name: 'ComputeDemo',           lang: 'assembly',   loader: 'assembly' },
    { key: 'led_control',      name: 'LedControl',            lang: 'assembly',   loader: 'assembly' },
    { key: 'salvation',        name: 'Salvation',             lang: 'assembly',   loader: 'assembly' },
    { key: 'perm_attack',      name: 'PermAttack',            lang: 'assembly',   loader: 'assembly' },
    { key: 'bind_attack',      name: 'BindAttack',            lang: 'assembly',   loader: 'assembly' },
    { key: 'ada_note_g',       name: 'NoteG (Assembly)',      lang: 'assembly',   loader: 'assembly' },

    /* ── Haskell ──────────────────────────────────────────── */
    { key: 'church_math',      name: 'ChurchMath',            lang: 'haskell',    loader: 'cloomc' },
    { key: 'church_pair',      name: 'ChurchPair',            lang: 'haskell',    loader: 'cloomc' },
    { key: 'church_case',      name: 'ChurchCase',            lang: 'haskell',    loader: 'cloomc' },
    { key: 'sliderule_hs',     name: 'SlideRule (Haskell)',   lang: 'haskell',    loader: 'cloomc' },

    /* ── Symbolic Math ────────────────────────────────────── */
    { key: 'ada_note_g',             name: 'NoteG (Symbolic)',       lang: 'symbolic',   loader: 'cloomc' },
    { key: 'ada_note_g_published_bug', name: 'NoteG (Published Bug)', lang: 'symbolic',   loader: 'cloomc' },
    { key: 'bernoulli_numbers',      name: 'BernoulliNumbers',       lang: 'symbolic',   loader: 'cloomc' },

    /* ── English ──────────────────────────────────────────── */
    { key: 'english_integer_ops',    name: 'IntegerOps (English)',   lang: 'english',    loader: 'cloomc' },
    { key: 'english_packed_string',  name: 'StringOps (English)',    lang: 'english',    loader: 'cloomc' },
    { key: 'english_loops',          name: 'EnglishLoops',           lang: 'english',    loader: 'cloomc' },
    { key: 'english_contact',        name: 'Contact (English)',      lang: 'english',    loader: 'cloomc' },
    { key: 'english_contact_stage2', name: 'ContactStage2 (English)',lang: 'english',    loader: 'cloomc' },

    /* ── Lambda Calculus ──────────────────────────────────── */
    { key: 'lambda_church_numerals', name: 'ChurchNumerals',         lang: 'lambda',     loader: 'cloomc' },
    { key: 'lambda_church_encoding', name: 'ChurchEncoding',         lang: 'lambda',     loader: 'cloomc' },
    { key: 'lambda_fixed_point',     name: 'FixedPoint',             lang: 'lambda',     loader: 'cloomc' },
    { key: 'lambda_sliderule',       name: 'LambdaSlideRule',        lang: 'lambda',     loader: 'cloomc' },
    { key: 'lambda_rational',        name: 'RationalArithmetic',     lang: 'lambda',     loader: 'cloomc' },
];

const _SL_LANG_ORDER = ['javascript', 'assembly', 'haskell', 'symbolic', 'english', 'lambda'];

const _SL_LANG_LABELS = {
    javascript: 'CLOOMC++ (JavaScript)',
    assembly:   'Assembly',
    haskell:    'Haskell',
    symbolic:   'Symbolic Math (Ada)',
    english:    'English',
    lambda:     'Lambda Calculus',
};

const _SL_LANG_BADGE_CLASS = {
    javascript: 'sl-badge-js',
    assembly:   'sl-badge-asm',
    haskell:    'sl-badge-hs',
    symbolic:   'sl-badge-sym',
    english:    'sl-badge-en',
    lambda:     'sl-badge-lc',
};

/* Cached fetched source text keyed by URL. */
const _SL_FETCH_CACHE = {};

/* Whether the panel has been rendered at least once. */
let _slRendered = false;

/* ── Sub-tab switching ───────────────────────────────────────────────────── */

function switchAbsSubtab(tab) {
    ['catalog', 'sources'].forEach(t => {
        const btn = document.getElementById('absSubtab-' + t);
        const panel = document.getElementById('absSubpanel-' + t);
        const active = t === tab;
        if (btn)   btn.classList.toggle('abs-subtab-active', active);
        if (panel) panel.style.display = active ? '' : 'none';
    });
    if (tab === 'sources') _openSourceLibrary();
}

/* ── Open / render ───────────────────────────────────────────────────────── */

function _openSourceLibrary() {
    const panel = document.getElementById('absSubpanel-sources');
    if (!panel) return;
    if (_slRendered) return;
    _renderSourceLibrary();
}

async function _renderSourceLibrary() {
    const container = document.getElementById('slContent');
    if (!container) return;

    container.innerHTML = '<div class="sl-loading">Loading sources\u2026</div>';

    /* The window globals (_cloomcExampleSources, _asmExampleSources, etc.) are
       set unconditionally at module-load time in app-compile.js / app-run.js.
       No side-effectful bootstrap calls are needed here. If they are somehow
       absent (e.g. script load order wrong), we fall back to empty dicts and
       emit a console warning so the issue is visible in dev tools. */
    if (!window._cloomcExampleSources) {
        console.warn('[SourceLibrary] window._cloomcExampleSources not found — check app-compile.js load order');
    }
    if (!window._asmExampleSources) {
        console.warn('[SourceLibrary] window._asmExampleSources not found — check app-run.js load order');
    }

    /* Collect all file-based URLs that need fetching. */
    const fileExamples  = window._cloomcFileExamples || {};
    const fileLanguages = window._cloomcFileLanguages || {};

    const urlsNeeded = new Set();
    for (const entry of _SL_REGISTRY) {
        if (entry.loader === 'cloomc' && fileExamples[entry.key] && !_SL_FETCH_CACHE[fileExamples[entry.key]]) {
            urlsNeeded.add(fileExamples[entry.key]);
        }
    }

    if (urlsNeeded.size > 0) {
        await Promise.allSettled([...urlsNeeded].map(url =>
            fetch(url)
                .then(r => r.ok ? r.text() : Promise.reject('not found'))
                .then(t  => { _SL_FETCH_CACHE[url] = t; })
                .catch(() => { _SL_FETCH_CACHE[url] = ''; })
        ));
    }

    /* Parity check: warn in console if any registry key is absent from the
       actual example dictionaries — catches drift when new examples are added
       to app-compile.js / app-run.js but not to _SL_REGISTRY. */
    _checkRegistryParity();

    _slRendered = true;
    _drawSourceLibrary(container);
}

function _checkRegistryParity() {
    const cloomcSrc    = window._cloomcExampleSources || {};
    const asmSrc       = window._asmExampleSources    || {};
    const fileExamples = window._cloomcFileExamples   || {};

    /* Keys that are present in the real example sets but absent from our registry. */
    const registryKeys = new Set(_SL_REGISTRY.map(e => e.key));

    const missingFromRegistry = [];
    for (const k of Object.keys(cloomcSrc)) {
        if (!registryKeys.has(k)) missingFromRegistry.push('cloomc:' + k);
    }
    for (const k of Object.keys(asmSrc)) {
        if (!registryKeys.has(k)) missingFromRegistry.push('asm:' + k);
    }
    for (const k of Object.keys(fileExamples)) {
        if (!registryKeys.has(k)) missingFromRegistry.push('file:' + k);
    }

    /* Keys in our registry that are absent from all real example sets. */
    const orphanKeys = _SL_REGISTRY
        .filter(e => !cloomcSrc[e.key] && !asmSrc[e.key] && !fileExamples[e.key])
        .map(e => e.loader + ':' + e.key);

    if (missingFromRegistry.length) {
        console.warn('[SourceLibrary] Examples in compiler not in Source Library registry ' +
            '(add to _SL_REGISTRY in app-source-library.js):', missingFromRegistry);
    }
    if (orphanKeys.length) {
        console.warn('[SourceLibrary] Registry keys with no matching example source ' +
            '(stale entry or wrong key):', orphanKeys);
    }
}

/* ── Draw ────────────────────────────────────────────────────────────────── */

function _drawSourceLibrary(container) {
    const cloomcSrc = window._cloomcExampleSources || {};
    const asmSrc    = window._asmExampleSources    || {};
    const fileExamples = window._cloomcFileExamples || {};

    /* Group entries by lang in defined order. */
    const groups = {};
    for (const lang of _SL_LANG_ORDER) groups[lang] = [];
    for (const entry of _SL_REGISTRY) {
        if (!groups[entry.lang]) groups[entry.lang] = [];
        groups[entry.lang].push(entry);
    }

    let html = '';

    for (const lang of _SL_LANG_ORDER) {
        const entries = groups[lang];
        if (!entries || entries.length === 0) continue;

        html += `<div class="sl-group" data-lang="${_esc(lang)}">`;
        html += `<div class="sl-group-header">${_esc(_SL_LANG_LABELS[lang] || lang)}</div>`;

        for (const entry of entries) {
            const src = _getEntrySource(entry, cloomcSrc, asmSrc, fileExamples);
            const methodCount = _countMethods(src, lang);
            const badgeClass  = _SL_LANG_BADGE_CLASS[lang] || '';
            const methodLabel = methodCount === 1 ? '1 method' : `${methodCount} methods`;
            const cardId      = `sl-card-${lang}-${entry.key}`;

            html += `
<div class="sl-card" id="${_esc(cardId)}"
     data-key="${_esc(entry.key)}"
     data-lang="${_esc(lang)}"
     data-name="${_esc(entry.name.toLowerCase())}"
     data-src="${_esc((src || '').toLowerCase())}">
  <div class="sl-card-header">
    <span class="sl-card-name">${_esc(entry.name)}</span>
    <span class="sl-badge ${_esc(badgeClass)}">${_esc(_slLangShort(lang))}</span>
    <span class="sl-method-count">${_esc(methodLabel)}</span>
    <div class="sl-card-actions">
      <button class="sl-load-btn" onclick="slLoadEntry('${entry.key}','${entry.loader}','${lang}')"
              title="Load into editor">Load \u2192 Editor</button>
      <button class="sl-toggle-btn" onclick="slToggleCard('${cardId}')"
              title="Show/hide source">Source \u25BE</button>
    </div>
  </div>
  <pre class="sl-source-pre" style="display:none">${_esc(src || '(source not available)')}</pre>
</div>`;
        }

        html += '</div>';
    }

    container.innerHTML = html;
    _applySourceLibraryFilter();
}

/* ── Helpers ─────────────────────────────────────────────────────────────── */

function _getEntrySource(entry, cloomcSrc, asmSrc, fileExamples) {
    if (entry.loader === 'assembly') {
        return (asmSrc[entry.key] || '').replace(
            /\` \+ '.*?'\.slice\(.*?\)$/s, ''
        );
    }
    /* File-based CLOOMC example */
    if (fileExamples[entry.key]) {
        return _SL_FETCH_CACHE[fileExamples[entry.key]] || '';
    }
    /* Inline CLOOMC example */
    return cloomcSrc[entry.key] || '';
}

function _countMethods(src, lang) {
    if (!src) return 0;
    if (lang === 'assembly') {
        /* Count numbered entries in the header comment block:
           ";   1. methodName" */
        return (src.match(/^;\s+\d+\./mg) || []).length;
    }
    /* For all high-level front-ends: count "method <name>" lines. */
    return (src.match(/\bmethod\s+\w+/g) || []).length;
}

function _slLangShort(lang) {
    return { javascript: 'JS', assembly: 'ASM', haskell: 'HS', symbolic: 'ADA', english: 'EN', lambda: 'λ' }[lang] || lang;
}

function _esc(str) {
    if (typeof str !== 'string') str = String(str || '');
    return str.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

/* ── Public interaction functions ────────────────────────────────────────── */

function slLoadEntry(key, loader, lang) {
    /* Switch to editor view first. */
    if (typeof switchView === 'function') switchView('editor');

    if (loader === 'assembly') {
        /* Switch language selector to assembly, then load. */
        const sel = document.getElementById('langSelector');
        if (sel) sel.value = 'assembly';
        if (typeof onLangChange === 'function') onLangChange(true);
        if (typeof loadExample === 'function') loadExample(key);
    } else {
        /* Always set the language selector and trigger onLangChange so the
           editor toolbar, tab-strip, and compiler mode all update consistently
           — even if the selector already shows the target language (e.g. the
           user clicked two Haskell cards in a row). */
        const sel = document.getElementById('langSelector');
        if (sel) sel.value = lang;
        if (typeof onLangChange === 'function') onLangChange(true);
        if (typeof loadCLOOMCExample === 'function') loadCLOOMCExample(key);
    }
}

function slToggleCard(cardId) {
    const card = document.getElementById(cardId);
    if (!card) return;
    const pre = card.querySelector('.sl-source-pre');
    const btn = card.querySelector('.sl-toggle-btn');
    if (!pre) return;
    const visible = pre.style.display !== 'none';
    pre.style.display = visible ? 'none' : 'block';
    if (btn) btn.textContent = visible ? 'Source \u25BE' : 'Source \u25B4';
}

/* ── Search / filter ─────────────────────────────────────────────────────── */

function filterSourceLibrary() {
    _applySourceLibraryFilter();
}

function _applySourceLibraryFilter() {
    const q = ((document.getElementById('slSearch') || {}).value || '').toLowerCase().trim();
    const cards = document.querySelectorAll('#slContent .sl-card');
    let visiblePerGroup = {};

    cards.forEach(card => {
        const name = (card.dataset.name || '');
        const src  = (card.dataset.src  || '');
        const lang = (card.dataset.lang  || '');
        const match = !q || name.includes(q) || src.includes(q);
        card.style.display = match ? '' : 'none';
        if (match) visiblePerGroup[lang] = (visiblePerGroup[lang] || 0) + 1;
    });

    /* Show/hide group headers based on whether any card is visible. */
    document.querySelectorAll('#slContent .sl-group').forEach(group => {
        const lang = group.dataset.lang || '';
        group.style.display = (visiblePerGroup[lang] > 0) ? '' : 'none';
    });
}
