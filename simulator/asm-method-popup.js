// asm-method-popup.js
// Method-name completion popup for the assembly editor.
//
// Fires automatically when the cursor lands after  AbstrName.suffix  where
// AbstrName is a key in METHOD_REGISTER_CONVENTIONS (defined in
// app-absdetail.js).  A compact dropdown lists matching method names with a
// short descriptor; clicking one (or pressing Enter) replaces the suffix
// with the chosen name.
//
// Public API:  window.AsmMethodPopup  (attach, hide)

(function () {
    'use strict';

    // ── Pattern that triggers the popup ──────────────────────────────────────
    // Matches  <word>.<suffix>  at the very end of text-before-cursor.
    // suffix may be empty (just typed the dot) or a partial method name.
    var TRIGGER_RE = /\b([A-Z][A-Za-z0-9]*)\.([A-Za-z0-9]*)$/;

    // ── State ─────────────────────────────────────────────────────────────────
    var popupEl      = null;   // the popup DOM node
    var listEl       = null;   // <ul> inside the popup
    var activeEditor = null;   // textarea currently being served
    var items        = [];     // { name, index, short } objects for current abstraction
    var selectedIdx  = -1;     // which item is highlighted
    var dotAbsPos    = -1;     // absolute position of the '.' in textarea.value
    var filterLen    = 0;      // length of what the user typed after the '.'

    // ── Helpers ───────────────────────────────────────────────────────────────

    function getConventions() {
        return (typeof METHOD_REGISTER_CONVENTIONS !== 'undefined')
            ? METHOD_REGISTER_CONVENTIONS : null;
    }

    // Build a short one-line descriptor for display.
    function shortDesc(conv) {
        var parts = [];
        if (conv.input  && conv.input  !== 'none') parts.push('\u2190 ' + conv.input);
        if (conv.output && conv.output !== 'none') parts.push('\u2192 ' + conv.output);
        return parts.join('  ');
    }

    // Mirror of the caret-position approach in asm-instruction-picker.js.
    function caretPixelPos(textarea) {
        var cs = window.getComputedStyle(textarea);
        var div = document.createElement('div');
        ['fontFamily','fontSize','fontWeight','lineHeight','letterSpacing',
         'padding','paddingTop','paddingRight','paddingBottom','paddingLeft',
         'border','borderTop','borderRight','borderBottom','borderLeft',
         'boxSizing','whiteSpace','wordWrap','overflowWrap','tabSize']
            .forEach(function (p) { div.style[p] = cs[p]; });
        div.style.position = 'absolute';
        div.style.visibility = 'hidden';
        div.style.whiteSpace = 'pre-wrap';
        div.style.wordWrap = 'break-word';
        div.style.width = textarea.clientWidth + 'px';
        div.style.height = 'auto';
        div.style.top = '-9999px';
        div.style.left = '-9999px';
        div.style.overflow = 'hidden';

        var textBefore = textarea.value.substring(0, textarea.selectionStart);
        div.textContent = textBefore;

        var span = document.createElement('span');
        span.textContent = '\u200b';
        div.appendChild(span);
        document.body.appendChild(div);

        var taRect = textarea.getBoundingClientRect();
        var lineH  = parseFloat(cs.lineHeight) || parseFloat(cs.fontSize) * 1.4 || 16;
        var x = taRect.left + span.offsetLeft - textarea.scrollLeft;
        var y = taRect.top  + span.offsetTop  - textarea.scrollTop + lineH + 4;

        document.body.removeChild(div);
        return { x: x, y: y };
    }

    // ── DOM ───────────────────────────────────────────────────────────────────

    function ensurePopup() {
        if (popupEl) return popupEl;
        popupEl = document.createElement('div');
        popupEl.id = 'asmMethodPopup';
        popupEl.className = 'asm-method-popup';
        popupEl.setAttribute('role', 'listbox');
        popupEl.setAttribute('aria-label', 'Method picker');
        popupEl.style.display = 'none';

        var hdr = document.createElement('div');
        hdr.className = 'asm-method-popup-header';
        hdr.textContent = 'Methods \u00b7 \u2191\u2193 navigate \u00b7 Enter confirm \u00b7 Esc dismiss';
        popupEl.appendChild(hdr);

        listEl = document.createElement('ul');
        listEl.className = 'asm-method-popup-list';
        popupEl.appendChild(listEl);

        document.body.appendChild(popupEl);
        return popupEl;
    }

    function renderList(filteredItems) {
        listEl.innerHTML = '';
        filteredItems.forEach(function (item, i) {
            var li = document.createElement('li');
            li.className = 'asm-method-popup-item';
            li.setAttribute('role', 'option');
            li.dataset.idx = i;

            var nameSpan = document.createElement('span');
            nameSpan.className = 'asm-method-popup-name';
            nameSpan.textContent = item.name;

            var idxSpan = document.createElement('span');
            idxSpan.className = 'asm-method-popup-index';
            idxSpan.textContent = '#' + item.index;

            var descSpan = document.createElement('span');
            descSpan.className = 'asm-method-popup-desc';
            descSpan.textContent = item.short;

            li.appendChild(nameSpan);
            li.appendChild(idxSpan);
            li.appendChild(descSpan);

            li.addEventListener('mousedown', function (e) {
                e.preventDefault();   // don't blur the textarea
                confirmItem(i);
            });
            li.addEventListener('mousemove', function () { highlight(i); });

            listEl.appendChild(li);
        });
        items = filteredItems;
        selectedIdx = filteredItems.length > 0 ? 0 : -1;
        applyHighlight();
    }

    function highlight(i) {
        selectedIdx = i;
        applyHighlight();
    }

    function applyHighlight() {
        if (!listEl) return;
        var lis = listEl.querySelectorAll('.asm-method-popup-item');
        lis.forEach(function (li, i) {
            li.classList.toggle('asm-method-popup-item--active', i === selectedIdx);
        });
        if (selectedIdx >= 0 && lis[selectedIdx]) {
            lis[selectedIdx].scrollIntoView({ block: 'nearest' });
        }
    }

    function positionPopup(textarea) {
        var pos = caretPixelPos(textarea);
        var popup = ensurePopup();
        var pw = 420;
        var ph = 220;
        var vw = window.innerWidth;
        var vh = window.innerHeight;

        var left = pos.x;
        var top  = pos.y;
        if (left + pw > vw) left = vw - pw - 8;
        if (left < 4) left = 4;
        if (top + ph > vh) top = pos.y - ph - 22;
        if (top < 4) top = 4;

        popup.style.left = left + 'px';
        popup.style.top  = top  + 'px';
    }

    // ── Show / hide ───────────────────────────────────────────────────────────

    function show(textarea, absName, filter, dotPos) {
        var conv = getConventions();
        if (!conv || !conv[absName]) { hide(); return; }

        var allMethods = Object.keys(conv[absName]).map(function (mName) {
            var c = conv[absName][mName];
            return { name: mName, index: c.index, short: shortDesc(c) };
        }).sort(function (a, b) { return a.index - b.index; });

        var lower = filter.toLowerCase();
        var filtered = allMethods.filter(function (m) {
            return m.name.toLowerCase().indexOf(lower) === 0;
        });

        if (filtered.length === 0) { hide(); return; }

        dotAbsPos  = dotPos;
        filterLen  = filter.length;
        activeEditor = textarea;

        var popup = ensurePopup();
        renderList(filtered);
        popup.style.display = 'flex';
        positionPopup(textarea);
    }

    function hide() {
        if (popupEl) popupEl.style.display = 'none';
        selectedIdx  = -1;
        dotAbsPos    = -1;
        filterLen    = 0;
        activeEditor = null;
    }

    function isVisible() {
        return !!(popupEl && popupEl.style.display !== 'none');
    }

    // ── Confirm a selection ───────────────────────────────────────────────────

    function confirmItem(idx) {
        if (!activeEditor || idx < 0 || idx >= items.length) { hide(); return; }
        var methodName = items[idx].name;
        var editor = activeEditor;

        // Replace the characters the user typed after the dot with the method name.
        var val = editor.value;
        var replaceStart = dotAbsPos + 1;          // character right after '.'
        var replaceEnd   = replaceStart + filterLen;
        editor.value = val.substring(0, replaceStart) + methodName + val.substring(replaceEnd);

        var newPos = replaceStart + methodName.length;
        editor.selectionStart = newPos;
        editor.selectionEnd   = newPos;
        editor.focus();

        hide();

        if (typeof updateLineNumbers === 'function') updateLineNumbers();
        if (typeof markUserTabDirty === 'function') markUserTabDirty();
    }

    // ── Attach to a textarea ──────────────────────────────────────────────────

    function attachToEditor(textarea) {
        if (!textarea || textarea._asmMethodPopupAttached) return;
        textarea._asmMethodPopupAttached = true;

        // On every keystroke update, re-evaluate trigger pattern.
        textarea.addEventListener('input', function () {
            var pos      = textarea.selectionStart;
            var before   = textarea.value.substring(0, pos);
            var m        = TRIGGER_RE.exec(before);
            if (!m) { hide(); return; }

            var absName  = m[1];
            var filter   = m[2];
            var dotPos   = pos - filter.length - 1;   // absolute index of '.'

            var conv = getConventions();
            if (!conv || !conv[absName]) { hide(); return; }

            show(textarea, absName, filter, dotPos);
        });

        // Keyboard navigation while popup is open.
        textarea.addEventListener('keydown', function (e) {
            if (!isVisible()) return;

            if (e.key === 'Escape') {
                e.preventDefault();
                hide();
                return;
            }
            if (e.key === 'ArrowDown') {
                e.preventDefault();
                var next = selectedIdx < 0 ? 0 : Math.min(selectedIdx + 1, items.length - 1);
                highlight(next);
                return;
            }
            if (e.key === 'ArrowUp') {
                e.preventDefault();
                var prev = selectedIdx < 0 ? items.length - 1 : Math.max(selectedIdx - 1, 0);
                highlight(prev);
                return;
            }
            if ((e.key === 'Enter' || e.key === 'Tab') && selectedIdx >= 0) {
                e.preventDefault();
                confirmItem(selectedIdx);
                return;
            }
        });

        // Dismiss on outside click.
        document.addEventListener('mousedown', function (e) {
            if (!isVisible()) return;
            if (popupEl && !popupEl.contains(e.target) && e.target !== textarea) {
                hide();
            }
        }, true);

        // Dismiss when the editor loses focus (unless click went into popup).
        textarea.addEventListener('blur', function () {
            setTimeout(function () {
                if (popupEl && document.activeElement && popupEl.contains(document.activeElement)) return;
                hide();
            }, 80);
        });
    }

    // ── Auto-attach ───────────────────────────────────────────────────────────

    function autoAttach() {
        attachToEditor(document.getElementById('asmEditor'));
        attachToEditor(document.getElementById('codeEditor'));
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', autoAttach);
    } else {
        autoAttach();
    }

    window.AsmMethodPopup = { attach: attachToEditor, hide: hide };

}());
