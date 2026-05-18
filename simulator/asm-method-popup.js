// asm-method-popup.js
// Method-name completion popup for the assembly editor.
//
// Fires automatically when the cursor lands after  AbstrName.suffix  where
// AbstrName is a key in METHOD_REGISTER_CONVENTIONS (defined in
// app-absdetail.js).  A compact dropdown lists matching method names with a
// short descriptor; clicking one (or pressing Enter) replaces the suffix
// with the chosen name.
//
// Additionally shows an inline parameter-hint bar (VS Code style) when the
// cursor is on a fully-typed dot-notation token like  CALL Navana.Init  and
// the method exists in API_DATA.  The hint shows the full signature and the
// required permission.
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
    var items        = [];     // { name, index, short, sig, perms } objects for current abstraction
    var selectedIdx  = -1;     // which item is highlighted
    var dotAbsPos    = -1;     // absolute position of the '.' in textarea.value
    var filterLen    = 0;      // length of what the user typed after the '.'
    var _currentAbsName = '';  // abstraction name currently being served

    // ── API_DATA lookup helper ────────────────────────────────────────────────
    // Returns the API_DATA method object for the given abstraction + method names,
    // or null if not found.  Works whether API_DATA is loaded or not.

    function apiMethodData(absName, methodName) {
        if (typeof API_DATA === 'undefined') return null;
        var nameLower = methodName.toLowerCase();
        for (var i = 0; i < API_DATA.length; i++) {
            if (API_DATA[i].name === absName) {
                var meths = API_DATA[i].methods;
                for (var j = 0; j < meths.length; j++) {
                    if (meths[j].name.toLowerCase() === nameLower) return meths[j];
                }
                return null;
            }
        }
        return null;
    }

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

    // ── Inline signature hint ─────────────────────────────────────────────────

    var sigHintEl = null;
    var sigHintTimer = null;

    function ensureSigHint() {
        if (sigHintEl) return sigHintEl;
        sigHintEl = document.createElement('div');
        sigHintEl.id = 'asmSigHint';
        sigHintEl.className = 'asm-sig-hint';
        sigHintEl.style.display = 'none';
        document.body.appendChild(sigHintEl);
        return sigHintEl;
    }

    function showSigHint(absName, methodName, textarea) {
        var mData = apiMethodData(absName, methodName);
        if (!mData) { hideSigHint(); return; }

        var el = ensureSigHint();

        var sigText = mData.signature || (methodName + '()');
        var perms   = mData.perms || '';
        var desc    = mData.description || '';
        var impl    = mData.implemented;
        var implClass = impl ? 'asm-sig-hint-badge--impl' : 'asm-sig-hint-badge--planned';
        var implLabel = impl ? 'Implemented' : 'Planned';

        el.innerHTML =
            '<span class="asm-sig-hint-abs">' + _esc(absName) + '</span>'
          + '<span class="asm-sig-hint-dot">.</span>'
          + '<span class="asm-sig-hint-method">' + _esc(methodName) + '</span>'
          + '<span class="asm-sig-hint-sig">' + _esc(sigText) + '</span>'
          + (perms ? '<span class="asm-sig-hint-perm-label">Permission:</span>'
                   + '<span class="asm-sig-hint-perm">' + _esc(perms) + '</span>' : '')
          + (desc ? '<span class="asm-sig-hint-desc">' + _esc(desc) + '</span>' : '')
          + '<span class="asm-sig-hint-badge ' + implClass + '">' + implLabel + '</span>'
          + '<span class="asm-sig-hint-dismiss" title="Dismiss (Esc)">\u00d7</span>';

        el.querySelector('.asm-sig-hint-dismiss').addEventListener('mousedown', function (e) {
            e.preventDefault();
            hideSigHint();
        });

        var pos = caretPixelPos(textarea);
        var ew  = el.offsetWidth  || 480;
        var eh  = el.offsetHeight || 28;
        var vw  = window.innerWidth;
        var vh  = window.innerHeight;

        var left = pos.x - 4;
        var top  = pos.y + 2;

        if (left + ew > vw - 8) left = vw - ew - 8;
        if (left < 4) left = 4;
        if (top + eh > vh - 8) top = pos.y - eh - 24;
        if (top < 4) top = 4;

        el.style.left    = left + 'px';
        el.style.top     = top  + 'px';
        el.style.display = 'flex';
    }

    function hideSigHint() {
        clearTimeout(sigHintTimer);
        if (sigHintEl) sigHintEl.style.display = 'none';
    }

    function _esc(s) {
        if (!s) return '';
        return String(s)
            .replace(/&/g, '&amp;')
            .replace(/</g, '&lt;')
            .replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;');
    }

    // Check whether the cursor is sitting on or just after a complete Abs.Method
    // token (i.e. the user has finished typing the method name and has not moved on).
    // Returns { absName, methodName } or null.
    function detectCompleteToken(textarea) {
        var pos    = textarea.selectionStart;
        var text   = textarea.value;
        // Look at a window around the cursor position on the current line
        var lineStart = text.lastIndexOf('\n', pos - 1) + 1;
        var lineEnd   = text.indexOf('\n', pos);
        if (lineEnd === -1) lineEnd = text.length;
        var line   = text.slice(lineStart, lineEnd);
        var col    = pos - lineStart;

        // Scan the line for all Abs.Method occurrences, pick the one that contains col
        var re = /\b([A-Z][A-Za-z0-9]*)\.([A-Za-z][A-Za-z0-9]+)\b/g;
        var m;
        while ((m = re.exec(line)) !== null) {
            var start = m.index;
            var end   = m.index + m[0].length;
            if (col >= start && col <= end) {
                return { absName: m[1], methodName: m[2] };
            }
        }
        return null;
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

            li.appendChild(nameSpan);
            li.appendChild(idxSpan);

            // Signature from API_DATA (if available)
            if (item.sig) {
                var sigSpan = document.createElement('span');
                sigSpan.className = 'asm-method-popup-sig';
                sigSpan.textContent = item.sig;
                li.appendChild(sigSpan);
            }

            // Permissions badge from API_DATA (if available)
            if (item.perms) {
                var permSpan = document.createElement('span');
                permSpan.className = 'asm-method-popup-perms';
                permSpan.textContent = item.perms;
                li.appendChild(permSpan);
            }

            // Fall back to old short desc if no signature/perms
            if (!item.sig && item.short) {
                var descSpan = document.createElement('span');
                descSpan.className = 'asm-method-popup-desc';
                descSpan.textContent = item.short;
                li.appendChild(descSpan);
            }

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
            // Enrich with API_DATA if available
            var mApi  = apiMethodData(absName, mName);
            return {
                name:  mName,
                index: c.index,
                short: shortDesc(c),
                sig:   mApi ? mApi.signature   : null,
                perms: mApi ? mApi.perms        : null
            };
        }).sort(function (a, b) { return a.index - b.index; });

        var lower = filter.toLowerCase();
        var filtered = allMethods.filter(function (m) {
            return m.name.toLowerCase().indexOf(lower) === 0;
        });

        if (filtered.length === 0) { hide(); return; }

        dotAbsPos      = dotPos;
        filterLen      = filter.length;
        activeEditor   = textarea;
        _currentAbsName = absName;

        var popup = ensurePopup();
        renderList(filtered);
        popup.style.display = 'flex';
        positionPopup(textarea);

        // If there is exactly one match and the filter already equals it exactly,
        // the user has already finished typing; show the sig hint immediately.
        if (filtered.length === 1 && filter.length > 0
                && filter.toLowerCase() === filtered[0].name.toLowerCase()) {
            showSigHint(absName, filtered[0].name, textarea);
        } else {
            hideSigHint();
        }
    }

    function hide() {
        if (popupEl) popupEl.style.display = 'none';
        selectedIdx   = -1;
        dotAbsPos     = -1;
        filterLen     = 0;
        activeEditor  = null;
        _currentAbsName = '';
    }

    function isVisible() {
        return !!(popupEl && popupEl.style.display !== 'none');
    }

    // ── Confirm a selection ───────────────────────────────────────────────────

    function confirmItem(idx) {
        if (!activeEditor || idx < 0 || idx >= items.length) { hide(); return; }
        var item   = items[idx];
        var absName = _currentAbsName;
        var editor = activeEditor;

        // Replace the characters the user typed after the dot with the method name.
        var val = editor.value;
        var replaceStart = dotAbsPos + 1;          // character right after '.'
        var replaceEnd   = replaceStart + filterLen;
        editor.value = val.substring(0, replaceStart) + item.name + val.substring(replaceEnd);

        var newPos = replaceStart + item.name.length;
        editor.selectionStart = newPos;
        editor.selectionEnd   = newPos;
        editor.focus();

        hide();

        if (typeof updateLineNumbers === 'function') updateLineNumbers();
        if (typeof markUserTabDirty === 'function') markUserTabDirty();

        // Show the parameter hint for the confirmed method
        showSigHint(absName, item.name, editor);
    }

    // ── Attach to a textarea ──────────────────────────────────────────────────

    function attachToEditor(textarea) {
        if (!textarea || textarea._asmMethodPopupAttached) return;
        textarea._asmMethodPopupAttached = true;

        // On every keystroke update, re-evaluate trigger pattern.
        textarea.addEventListener('input', function () {
            var pos    = textarea.selectionStart;
            var before = textarea.value.substring(0, pos);
            var m      = TRIGGER_RE.exec(before);
            if (!m) {
                hide();
                // Even if no dropdown trigger, check for a complete Abs.Method token
                // and show the signature hint inline.
                clearTimeout(sigHintTimer);
                sigHintTimer = setTimeout(function () {
                    var tok = detectCompleteToken(textarea);
                    if (tok) {
                        showSigHint(tok.absName, tok.methodName, textarea);
                    } else {
                        hideSigHint();
                    }
                }, 200);
                return;
            }

            var absName = m[1];
            var filter  = m[2];
            var dotPos  = pos - filter.length - 1;   // absolute index of '.'

            var conv = getConventions();
            if (!conv || !conv[absName]) { hide(); return; }

            // Hide sig hint while dropdown is open (avoid double-display)
            hideSigHint();
            show(textarea, absName, filter, dotPos);
        });

        // Keyboard navigation while popup is open.
        textarea.addEventListener('keydown', function (e) {
            if (e.key === 'Escape') {
                if (isVisible()) {
                    e.preventDefault();
                    hide();
                    return;
                }
                if (sigHintEl && sigHintEl.style.display !== 'none') {
                    e.preventDefault();
                    hideSigHint();
                    return;
                }
            }
            if (!isVisible()) return;

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

        // Also show sig hint when the cursor moves onto a complete token via
        // keyup (arrow keys, clicking, etc.) — with a short delay.
        textarea.addEventListener('keyup', function (e) {
            if (isVisible()) return;
            clearTimeout(sigHintTimer);
            sigHintTimer = setTimeout(function () {
                var tok = detectCompleteToken(textarea);
                if (tok) {
                    showSigHint(tok.absName, tok.methodName, textarea);
                } else {
                    hideSigHint();
                }
            }, 300);
        });

        textarea.addEventListener('click', function () {
            if (isVisible()) return;
            clearTimeout(sigHintTimer);
            sigHintTimer = setTimeout(function () {
                var tok = detectCompleteToken(textarea);
                if (tok) {
                    showSigHint(tok.absName, tok.methodName, textarea);
                } else {
                    hideSigHint();
                }
            }, 150);
        });

        // Dismiss on outside click.
        document.addEventListener('mousedown', function (e) {
            if (!isVisible()) return;
            if (popupEl && !popupEl.contains(e.target) && e.target !== textarea) {
                hide();
            }
        }, true);

        // Dismiss when the editor loses focus (unless click went into popup).
        // The signature hint is always hidden on blur; it re-appears when the
        // editor regains focus and input/keyup fires again.
        textarea.addEventListener('blur', function () {
            setTimeout(function () {
                if (popupEl && document.activeElement && popupEl.contains(document.activeElement)) return;
                hide();
                hideSigHint();
            }, 80);
        });

        // Dismiss sig hint on scroll
        textarea.addEventListener('scroll', function () { hideSigHint(); });
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
