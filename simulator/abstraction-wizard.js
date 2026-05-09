// abstraction-wizard.js
// 4-step "New Abstraction" wizard for the assembly editor toolbar.
// Steps: Name → Methods → Dependencies → Docs → Generate CLOOMC++ scaffold.
//
// Depends on globals set up by other simulator scripts:
//   sim           — ChurchMachineSim instance (app-shell.js)
//   _petNameCRMap — slot-indexed pet name map  (app-run.js)
//
// Public API: window.AbstractionWizard = { open, close, isOpen }

(function () {
    'use strict';

    var DEFAULT_SHORTCUT = { code: 'KeyN', ctrl: true, shift: true };

    var overlayEl = null;
    var step = 1;
    var TOTAL_STEPS = 4;

    // ── Wizard state ─────────────────────────────────────────────────────────
    var state = {
        name: '',
        methods: [],
        deps: [],
        doc: { description: '', author: '', version: '1.0', purpose: '' }
    };

    function resetState() {
        step = 1;
        state.name = '';
        state.methods = [{ name: 'Run', params: '', doc: '' }];
        state.deps = [];
        state.doc = { description: '', author: '', version: '1.0', purpose: '' };
    }

    // ── NS dependency candidates ──────────────────────────────────────────────
    function getDependencyCandidates() {
        var candidates = [];
        try {
            var s = window.sim;
            if (!s) return FALLBACK_DEPS;
            var labels = s.nsLabels || {};
            var seen = {};
            Object.keys(labels).forEach(function (k) {
                var slot = parseInt(k);
                var label = labels[k];
                if (!label || label === '(free)' || seen[label]) return;
                seen[label] = true;
                candidates.push({ slot: slot, label: label });
            });
            if (candidates.length === 0) return FALLBACK_DEPS;
            return candidates.sort(function (a, b) { return a.slot - b.slot; });
        } catch (_) { return FALLBACK_DEPS; }
    }

    var FALLBACK_DEPS = [
        { slot: 0, label: 'Boot.NS' },
        { slot: 4, label: 'Salvation' },
        { slot: 5, label: 'Navana' },
        { slot: 6, label: 'Mint' },
        { slot: 7, label: 'Memory' },
        { slot: 8, label: 'Scheduler' },
        { slot: 12, label: 'LED' },
        { slot: 16, label: 'SlideRule' },
        { slot: 17, label: 'Abacus' },
        { slot: 18, label: 'Constants' }
    ];

    // ── Overlay helpers ───────────────────────────────────────────────────────
    function open() {
        if (overlayEl) return;
        resetState();
        _hideOthers();
        overlayEl = document.createElement('div');
        overlayEl.className = 'abswiz-overlay';
        overlayEl.addEventListener('mousedown', function (e) {
            if (e.target === overlayEl) close();
        });
        document.body.appendChild(overlayEl);
        renderStep();
        document.addEventListener('keydown', _onKeyDown, true);
    }

    function close() {
        if (!overlayEl) return;
        overlayEl.remove();
        overlayEl = null;
        document.removeEventListener('keydown', _onKeyDown, true);
    }

    function isOpen() { return !!overlayEl; }

    function _hideOthers() {
        if (window.AsmInstructionPicker && window.AsmInstructionPicker.hide)
            window.AsmInstructionPicker.hide();
        if (window.CListViewer && window.CListViewer.hide)
            window.CListViewer.hide();
    }

    function _onKeyDown(e) {
        if (e.key === 'Escape') { close(); e.stopPropagation(); }
    }

    // ── Step rendering ────────────────────────────────────────────────────────
    function renderStep() {
        if (!overlayEl) return;
        overlayEl.innerHTML = '';
        var dialog = document.createElement('div');
        dialog.className = 'abswiz-dialog';
        overlayEl.appendChild(dialog);

        _buildHeader(dialog);
        _buildStepIndicator(dialog);

        var body = document.createElement('div');
        body.className = 'abswiz-body';
        dialog.appendChild(body);

        if (step === 1) _renderStepName(body);
        else if (step === 2) _renderStepMethods(body);
        else if (step === 3) _renderStepDeps(body);
        else if (step === 4) _renderStepDocs(body);

        _buildFooter(dialog);
    }

    function _buildHeader(parent) {
        var hdr = document.createElement('div');
        hdr.className = 'abswiz-header';
        hdr.innerHTML = '<span class="abswiz-title">&#x2295; New Abstraction</span>';
        var closeBtn = document.createElement('button');
        closeBtn.className = 'abswiz-close';
        closeBtn.innerHTML = '&times;';
        closeBtn.title = 'Close wizard';
        closeBtn.addEventListener('click', close);
        hdr.appendChild(closeBtn);
        parent.appendChild(hdr);
    }

    function _buildStepIndicator(parent) {
        var labels = ['Name', 'Methods', 'Dependencies', 'Docs'];
        var wrap = document.createElement('div');
        wrap.className = 'abswiz-steps';
        labels.forEach(function (label, i) {
            var dot = document.createElement('div');
            dot.className = 'abswiz-step-dot' +
                (i + 1 === step ? ' abswiz-step-dot--active' : '') +
                (i + 1 < step ? ' abswiz-step-dot--done' : '');
            dot.textContent = i + 1 < step ? '✓' : String(i + 1);
            var lbl = document.createElement('div');
            lbl.className = 'abswiz-step-label' + (i + 1 === step ? ' abswiz-step-label--active' : '');
            lbl.textContent = label;
            var cell = document.createElement('div');
            cell.className = 'abswiz-step-cell';
            cell.appendChild(dot);
            cell.appendChild(lbl);
            wrap.appendChild(cell);
            if (i < labels.length - 1) {
                var line = document.createElement('div');
                line.className = 'abswiz-step-line' + (i + 1 < step ? ' abswiz-step-line--done' : '');
                wrap.appendChild(line);
            }
        });
        parent.appendChild(wrap);
    }

    function _buildFooter(parent) {
        var footer = document.createElement('div');
        footer.className = 'abswiz-footer';

        if (step > 1) {
            var back = document.createElement('button');
            back.className = 'abswiz-btn abswiz-btn--secondary';
            back.textContent = '← Back';
            back.addEventListener('click', function () { _saveStep(); step--; renderStep(); });
            footer.appendChild(back);
        }

        var spacer = document.createElement('span');
        spacer.style.flex = '1';
        footer.appendChild(spacer);

        if (step < TOTAL_STEPS) {
            var next = document.createElement('button');
            next.className = 'abswiz-btn abswiz-btn--primary';
            next.textContent = 'Next →';
            next.addEventListener('click', function () {
                if (!_validateStep()) return;
                _saveStep(); step++; renderStep();
            });
            footer.appendChild(next);
        } else {
            var gen = document.createElement('button');
            gen.className = 'abswiz-btn abswiz-btn--generate';
            gen.innerHTML = '&#x2699; Generate Scaffold';
            gen.addEventListener('click', function () {
                _saveStep();
                _generate();
                close();
            });
            footer.appendChild(gen);
        }

        parent.appendChild(footer);
    }

    // ── Step 1: Name ──────────────────────────────────────────────────────────
    function _renderStepName(body) {
        body.innerHTML =
            '<p class="abswiz-hint">Give your abstraction a descriptive PascalCase name. ' +
            'This becomes the CLOOMC++ <code>abstraction</code> keyword and the lump label.</p>';
        var field = document.createElement('div');
        field.className = 'abswiz-field';
        var lbl = document.createElement('label');
        lbl.className = 'abswiz-label';
        lbl.textContent = 'Abstraction name';
        var inp = document.createElement('input');
        inp.className = 'abswiz-input';
        inp.id = 'wiz-name';
        inp.type = 'text';
        inp.placeholder = 'e.g. MyCounter, StringUtils, NetworkStack';
        inp.value = state.name;
        inp.maxLength = 64;
        inp.addEventListener('keydown', function (e) {
            if (e.key === 'Enter') { if (!_validateStep()) return; _saveStep(); step++; renderStep(); }
        });
        field.appendChild(lbl);
        field.appendChild(inp);
        body.appendChild(field);
        setTimeout(function () { inp.focus(); inp.select(); }, 30);
    }

    // ── Step 2: Methods ───────────────────────────────────────────────────────
    function _renderStepMethods(body) {
        body.innerHTML =
            '<p class="abswiz-hint">Add the methods your abstraction exposes. ' +
            'Each method becomes a CLOOMC++ <code>method</code> definition. ' +
            'Params are comma-separated (e.g. <code>a, b</code>).</p>';

        var table = document.createElement('div');
        table.className = 'abswiz-method-table';
        table.id = 'wiz-method-table';

        function addRow(m) {
            var row = document.createElement('div');
            row.className = 'abswiz-method-row';

            var nameInp = document.createElement('input');
            nameInp.className = 'abswiz-input abswiz-input--method-name';
            nameInp.placeholder = 'MethodName';
            nameInp.value = m ? m.name : '';
            nameInp.maxLength = 48;

            var paramsInp = document.createElement('input');
            paramsInp.className = 'abswiz-input abswiz-input--method-params';
            paramsInp.placeholder = 'params (a, b, …)';
            paramsInp.value = m ? m.params : '';
            paramsInp.maxLength = 80;

            var docInp = document.createElement('input');
            docInp.className = 'abswiz-input abswiz-input--method-doc';
            docInp.placeholder = 'one-line description';
            docInp.value = m ? m.doc : '';
            docInp.maxLength = 120;

            var delBtn = document.createElement('button');
            delBtn.className = 'abswiz-method-del';
            delBtn.innerHTML = '&times;';
            delBtn.title = 'Remove this method';
            delBtn.addEventListener('click', function () {
                if (table.querySelectorAll('.abswiz-method-row').length > 1)
                    row.remove();
            });

            row.appendChild(nameInp);
            row.appendChild(paramsInp);
            row.appendChild(docInp);
            row.appendChild(delBtn);
            table.appendChild(row);
        }

        if (state.methods.length === 0)
            state.methods = [{ name: 'Run', params: '', doc: '' }];
        state.methods.forEach(function (m) { addRow(m); });
        body.appendChild(table);

        var addBtn = document.createElement('button');
        addBtn.className = 'abswiz-btn abswiz-btn--add-method';
        addBtn.innerHTML = '&#x2295; Add method';
        addBtn.addEventListener('click', function () { addRow(null); });
        body.appendChild(addBtn);

        var colHdr = document.createElement('div');
        colHdr.className = 'abswiz-method-col-hdr';
        colHdr.innerHTML =
            '<span>Name</span><span>Parameters</span><span>Description</span><span></span>';
        table.insertBefore(colHdr, table.firstChild);
    }

    // ── Step 3: Dependencies ──────────────────────────────────────────────────
    function _renderStepDeps(body) {
        body.innerHTML =
            '<p class="abswiz-hint">Select the abstractions your code will call (C-List entries). ' +
            'Each selected dependency becomes a <code>capabilities { }</code> entry and a ' +
            'C-List slot <code>CR0</code>–<code>CR17</code>. You can skip this if the ' +
            'abstraction needs no external capabilities.</p>';

        var candidates = getDependencyCandidates();
        var grid = document.createElement('div');
        grid.className = 'abswiz-deps-grid';

        candidates.forEach(function (c) {
            var isChecked = state.deps.some(function (d) { return d.slot === c.slot; });
            var item = document.createElement('label');
            item.className = 'abswiz-dep-item' + (isChecked ? ' abswiz-dep-item--checked' : '');

            var cb = document.createElement('input');
            cb.type = 'checkbox';
            cb.value = String(c.slot);
            cb.checked = isChecked;
            cb.addEventListener('change', function () {
                item.classList.toggle('abswiz-dep-item--checked', cb.checked);
            });

            var slotBadge = document.createElement('span');
            slotBadge.className = 'abswiz-dep-slot';
            slotBadge.textContent = 'NS' + c.slot;

            var nameLbl = document.createElement('span');
            nameLbl.className = 'abswiz-dep-name';
            nameLbl.textContent = c.label;

            item.appendChild(cb);
            item.appendChild(slotBadge);
            item.appendChild(nameLbl);
            grid.appendChild(item);
        });

        body.appendChild(grid);
    }

    // ── Step 4: Docs ──────────────────────────────────────────────────────────
    function _renderStepDocs(body) {
        body.innerHTML =
            '<p class="abswiz-hint">These fields are written as comments at the top of the ' +
            'generated scaffold. They help document intent and ownership. All fields are optional.</p>';

        [
            { id: 'wiz-doc-desc', label: 'Description', key: 'description',
              placeholder: 'What does this abstraction do?', type: 'textarea' },
            { id: 'wiz-doc-author', label: 'Author', key: 'author',
              placeholder: 'Your name', type: 'text' },
            { id: 'wiz-doc-version', label: 'Version', key: 'version',
              placeholder: '1.0', type: 'text' },
            { id: 'wiz-doc-purpose', label: 'Purpose / Use case', key: 'purpose',
              placeholder: 'Why does this abstraction exist?', type: 'textarea' }
        ].forEach(function (f) {
            var field = document.createElement('div');
            field.className = 'abswiz-field';
            var lbl = document.createElement('label');
            lbl.className = 'abswiz-label';
            lbl.textContent = f.label;
            lbl.setAttribute('for', f.id);
            var inp;
            if (f.type === 'textarea') {
                inp = document.createElement('textarea');
                inp.className = 'abswiz-input abswiz-textarea';
                inp.rows = 2;
            } else {
                inp = document.createElement('input');
                inp.className = 'abswiz-input';
                inp.type = 'text';
            }
            inp.id = f.id;
            inp.placeholder = f.placeholder;
            inp.value = state.doc[f.key] || '';
            inp.maxLength = 300;
            field.appendChild(lbl);
            field.appendChild(inp);
            body.appendChild(field);
        });
    }

    // ── Save current step into state ─────────────────────────────────────────
    function _saveStep() {
        if (!overlayEl) return;
        if (step === 1) {
            var inp = overlayEl.querySelector('#wiz-name');
            if (inp) state.name = inp.value.trim();
        } else if (step === 2) {
            var rows = overlayEl.querySelectorAll('.abswiz-method-row');
            state.methods = [];
            rows.forEach(function (row) {
                var inputs = row.querySelectorAll('input');
                if (inputs.length < 3) return;
                var name = inputs[0].value.trim();
                if (!name) return;
                state.methods.push({
                    name: name,
                    params: inputs[1].value.trim(),
                    doc: inputs[2].value.trim()
                });
            });
            if (state.methods.length === 0)
                state.methods = [{ name: 'Run', params: '', doc: '' }];
        } else if (step === 3) {
            var cbs = overlayEl.querySelectorAll('.abswiz-dep-item input[type=checkbox]:checked');
            var candidates = getDependencyCandidates();
            state.deps = [];
            cbs.forEach(function (cb) {
                var slot = parseInt(cb.value);
                var found = candidates.find(function (c) { return c.slot === slot; });
                if (found) state.deps.push(found);
            });
        } else if (step === 4) {
            ['description', 'author', 'version', 'purpose'].forEach(function (k, i) {
                var ids = ['wiz-doc-desc', 'wiz-doc-author', 'wiz-doc-version', 'wiz-doc-purpose'];
                var el = overlayEl.querySelector('#' + ids[i]);
                if (el) state.doc[k] = el.value.trim();
            });
        }
    }

    // ── Validate current step ─────────────────────────────────────────────────
    function _validateStep() {
        if (step === 1) {
            var inp = overlayEl && overlayEl.querySelector('#wiz-name');
            var val = inp ? inp.value.trim() : '';
            if (!val) {
                if (inp) { inp.classList.add('abswiz-input--error'); inp.focus(); }
                return false;
            }
            if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(val)) {
                if (inp) { inp.classList.add('abswiz-input--error'); inp.focus(); }
                _showError('Name must start with a letter and contain only letters, digits, or underscores.');
                return false;
            }
            if (inp) inp.classList.remove('abswiz-input--error');
        }
        return true;
    }

    function _showError(msg) {
        if (!overlayEl) return;
        var existing = overlayEl.querySelector('.abswiz-error');
        if (existing) existing.remove();
        var err = document.createElement('div');
        err.className = 'abswiz-error';
        err.textContent = msg;
        var body = overlayEl.querySelector('.abswiz-body');
        if (body) body.insertBefore(err, body.firstChild);
    }

    // ── Code generation ───────────────────────────────────────────────────────
    function _generate() {
        var today = new Date().toISOString().slice(0, 10);
        var name = state.name || 'MyAbstraction';
        var methods = state.methods.length ? state.methods
            : [{ name: 'Run', params: '', doc: '' }];
        var deps = state.deps;
        var doc = state.doc;

        var LINE = '; ' + '='.repeat(61);
        var lines = [];

        lines.push(LINE);
        lines.push('; Abstraction : ' + name);
        if (doc.description) lines.push('; Description : ' + doc.description);
        if (doc.purpose)     lines.push('; Purpose     : ' + doc.purpose);
        if (doc.author)      lines.push('; Author      : ' + doc.author);
        lines.push('; Version     : ' + (doc.version || '1.0'));
        lines.push('; Created     : ' + today);
        lines.push(';');

        if (deps.length) {
            lines.push('; Dependencies (C-List):');
            deps.forEach(function (d, i) {
                lines.push(';   CR' + i + ' \u2014 ' + d.label + ' (NS' + d.slot + ', E)');
            });
        } else {
            lines.push('; Dependencies : none');
        }
        lines.push(';');
        lines.push('; Methods:');
        methods.forEach(function (m, i) {
            var sig = m.name + '(' + (m.params || '') + ')';
            var desc = m.doc ? ' \u2014 ' + m.doc : '';
            lines.push(';   ' + (i + 1) + '. ' + sig + desc);
        });
        lines.push(LINE);
        lines.push('');

        lines.push('abstraction ' + name + ' {');

        if (deps.length) {
            lines.push('    capabilities {');
            deps.forEach(function (d, i) {
                var comma = i < deps.length - 1 ? ',' : '';
                lines.push('        ' + d.label + comma);
            });
            lines.push('    }');
        } else {
            lines.push('    capabilities { }');
        }

        methods.forEach(function (m) {
            lines.push('');
            if (m.doc)    lines.push('    ; ' + m.doc);
            var params = m.params || '';
            var paramList = params ? params.split(',').map(function (p) { return p.trim(); }) : [];
            lines.push('    ; Parameters: ' + (paramList.length ? paramList.join(', ') : 'none'));
            lines.push('    ; Returns:    (fill in)');
            var retVal = paramList.length ? paramList[0] : '0';
            lines.push('    method ' + m.name + '(' + params + ') {');
            lines.push('        return(' + retVal + ')');
            lines.push('    }');
        });

        lines.push('}');
        lines.push('');

        var scaffold = lines.join('\n');

        var ed = document.getElementById('asmEditor');
        if (ed) {
            ed.value = scaffold;
            ed.dispatchEvent(new Event('input', { bubbles: true }));
            ed.focus();
        }
    }

    // ── Keyboard shortcut ─────────────────────────────────────────────────────
    document.addEventListener('keydown', function (e) {
        if (overlayEl) return;
        var cfg = (window.AbstractionWizardConfig) || DEFAULT_SHORTCUT;
        var mod = cfg.ctrl ? (e.ctrlKey || e.metaKey) : true;
        var sh  = cfg.shift ? e.shiftKey : true;
        var key = e.code === cfg.code || e.key === 'N';
        if (mod && sh && key) { e.preventDefault(); open(); }
    });

    // ── Wrap AsmInstructionPicker.show and CListViewer.show ──────────────────
    (function () {
        function wrapShow(obj) {
            if (!obj || !obj.show) return;
            var orig = obj.show;
            obj.show = function () {
                close();
                return orig.apply(obj, arguments);
            };
        }
        wrapShow(window.AsmInstructionPicker);
        wrapShow(window.CListViewer);
    })();

    window.AbstractionWizard = { open: open, close: close, isOpen: isOpen };
})();
