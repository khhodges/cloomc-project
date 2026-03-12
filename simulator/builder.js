class NamespaceBuilder {
    static VERSION = '1.1';
    constructor() {
        this.computers = [];
        this.nextComputerId = 1;
        this.nextNamespaceId = 1;
        this.nextAbstractionId = 1;
        this.selectedComputer = null;
        this.selectedNamespace = null;
        this.draggingNode = null;
        this.dragOffset = { x: 0, y: 0 };
        this._dragMoved = false;
        this.error = null;
        this._boundDocMouseUp = this._onDocMouseUp.bind(this);
        this._boundDocMouseMove = this._onDocMouseMove.bind(this);
        this._docListenersAttached = false;
        this._loadState();
    }

    _esc(s) {
        if (typeof s !== 'string') s = String(s);
        return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;').replace(/'/g, '&#39;');
    }

    _loadState() {
        try {
            var saved = localStorage.getItem('church_builder');
            if (saved) {
                var s = JSON.parse(saved);
                this.computers = s.computers || [];
                this.nextComputerId = s.nextComputerId || 1;
                this.nextNamespaceId = s.nextNamespaceId || 1;
                this.nextAbstractionId = s.nextAbstractionId || 1;
            }
        } catch (e) {}
    }

    _saveState() {
        try {
            localStorage.setItem('church_builder', JSON.stringify({
                computers: this.computers,
                nextComputerId: this.nextComputerId,
                nextNamespaceId: this.nextNamespaceId,
                nextAbstractionId: this.nextAbstractionId
            }));
        } catch (e) {}
    }

    addComputer(x, y) {
        var id = this.nextComputerId++;
        this.computers.push({
            id: id, label: 'Computer ' + id, x: x, y: y, namespaces: []
        });
        this._saveState();
        this.render();
    }

    removeComputer(id) {
        this.computers = this.computers.filter(function(c) { return c.id !== id; });
        if (this.selectedComputer && this.selectedComputer.id === id) {
            this.selectedComputer = null;
            this.selectedNamespace = null;
        }
        this._saveState();
        this.render();
    }

    selectComputer(id) {
        this.selectedComputer = this.computers.find(function(c) { return c.id === id; }) || null;
        this.selectedNamespace = null;
        this.error = null;
        this.render();
    }

    addNamespace(computerId, label) {
        var comp = this.computers.find(function(c) { return c.id === computerId; });
        if (!comp) return;
        if (comp.namespaces.length >= 8) {
            this.error = 'Maximum 8 namespaces per computer';
            this.render();
            return;
        }
        var id = this.nextNamespaceId++;
        var nsLabel = label || 'Namespace ' + id;
        comp.namespaces.push({ id: id, label: nsLabel, abstractions: [] });
        this._saveState();
        this.render();
    }

    removeNamespace(computerId, nsId) {
        var comp = this.computers.find(function(c) { return c.id === computerId; });
        if (!comp) return;
        comp.namespaces = comp.namespaces.filter(function(n) { return n.id !== nsId; });
        if (this.selectedNamespace && this.selectedNamespace.id === nsId) {
            this.selectedNamespace = null;
        }
        this._saveState();
        this.render();
    }

    selectNamespace(computerId, nsId) {
        var comp = this.computers.find(function(c) { return c.id === computerId; });
        if (!comp) return;
        this.selectedComputer = comp;
        this.selectedNamespace = comp.namespaces.find(function(n) { return n.id === nsId; }) || null;
        this.error = null;
        this.render();
    }

    addAbstraction(computerId, nsId, json) {
        var comp = this.computers.find(function(c) { return c.id === computerId; });
        if (!comp) return false;
        var ns = comp.namespaces.find(function(n) { return n.id === nsId; });
        if (!ns) return false;
        if (!json.abstraction || typeof json.abstraction !== 'string') {
            this.error = 'Invalid upload.json: "abstraction" must be a non-empty string';
            this.render();
            return false;
        }
        if (!json.methods || !Array.isArray(json.methods) || json.methods.length === 0) {
            this.error = 'Invalid upload.json: "methods" must be a non-empty array';
            this.render();
            return false;
        }
        if (!json.capabilities || !Array.isArray(json.capabilities)) {
            this.error = 'Invalid upload.json: "capabilities" must be an array (use [] if none)';
            this.render();
            return false;
        }
        if (!json.grants || !Array.isArray(json.grants)) {
            this.error = 'Invalid upload.json: "grants" must be an array';
            this.render();
            return false;
        }
        var localNames = ns.abstractions.map(function(a) { return a.name.toUpperCase(); });
        var unresolvedCaps = [];
        for (var ci = 0; ci < json.capabilities.length; ci++) {
            var capName = String(json.capabilities[ci].name || 'unknown').toUpperCase();
            if (localNames.indexOf(capName) < 0) {
                var crossFound = this._findAbstractionLocation(capName, computerId, nsId);
                if (!crossFound) {
                    unresolvedCaps.push(json.capabilities[ci].name || 'unknown');
                }
            }
        }
        var id = this.nextAbstractionId++;
        ns.abstractions.push({
            id: id,
            name: String(json.abstraction),
            methods: json.methods.map(function(m) { return String(m.name || 'unnamed'); }),
            methodCount: json.methods.length,
            capabilities: json.capabilities.map(function(c) {
                return {
                    name: String(c.name || 'unknown'),
                    target: c.target,
                    grants: (c.grants || ['E']).map(function(g) { return String(g); })
                };
            }),
            grants: json.grants.map(function(g) { return String(g); }),
            codeSize: json.methods.reduce(function(sum, m) { return sum + (m.code ? m.code.length : 0); }, 0)
        });
        if (unresolvedCaps.length > 0) {
            this.error = 'Warning: unresolved dependencies: ' + unresolvedCaps.join(', ') + '. Add the required abstractions to resolve.';
        } else {
            this.error = null;
        }
        this._saveState();
        this.render();
        return true;
    }

    removeAbstraction(computerId, nsId, absId) {
        var comp = this.computers.find(function(c) { return c.id === computerId; });
        if (!comp) return;
        var ns = comp.namespaces.find(function(n) { return n.id === nsId; });
        if (!ns) return;
        ns.abstractions = ns.abstractions.filter(function(a) { return a.id !== absId; });
        this._saveState();
        this.render();
    }

    _resolveCapabilities(ns) {
        var names = ns.abstractions.map(function(a) { return a.name.toUpperCase(); });
        var resolved = [];
        for (var i = 0; i < ns.abstractions.length; i++) {
            var abs = ns.abstractions[i];
            for (var j = 0; j < abs.capabilities.length; j++) {
                var cap = abs.capabilities[j];
                var targetIdx = names.indexOf(cap.name.toUpperCase());
                resolved.push({
                    from: abs.name,
                    fromId: abs.id,
                    to: cap.name,
                    toId: targetIdx >= 0 ? ns.abstractions[targetIdx].id : null,
                    resolved: targetIdx >= 0
                });
            }
        }
        return resolved;
    }

    _resolveCrossNamespaceDeps() {
        var crossDeps = [];
        for (var ci = 0; ci < this.computers.length; ci++) {
            var comp = this.computers[ci];
            for (var ni = 0; ni < comp.namespaces.length; ni++) {
                var ns = comp.namespaces[ni];
                var localNames = ns.abstractions.map(function(a) { return a.name.toUpperCase(); });
                for (var ai = 0; ai < ns.abstractions.length; ai++) {
                    var abs = ns.abstractions[ai];
                    for (var ci2 = 0; ci2 < abs.capabilities.length; ci2++) {
                        var cap = abs.capabilities[ci2];
                        if (localNames.indexOf(cap.name.toUpperCase()) >= 0) continue;
                        var found = this._findAbstractionLocation(cap.name, comp.id, ns.id);
                        crossDeps.push({
                            fromCompId: comp.id,
                            fromNsId: ns.id,
                            fromAbsName: abs.name,
                            capName: cap.name,
                            toCompId: found ? found.compId : null,
                            toNsId: found ? found.nsId : null,
                            toNsLabel: found ? found.nsLabel : null,
                            resolved: !!found
                        });
                    }
                }
            }
        }
        return crossDeps;
    }

    _findAbstractionLocation(name, excludeCompId, excludeNsId) {
        var upper = name.toUpperCase();
        for (var ci = 0; ci < this.computers.length; ci++) {
            var comp = this.computers[ci];
            for (var ni = 0; ni < comp.namespaces.length; ni++) {
                var ns = comp.namespaces[ni];
                if (comp.id === excludeCompId && ns.id === excludeNsId) continue;
                for (var ai = 0; ai < ns.abstractions.length; ai++) {
                    if (ns.abstractions[ai].name.toUpperCase() === upper) {
                        return { compId: comp.id, nsId: ns.id, nsLabel: ns.label };
                    }
                }
            }
        }
        return null;
    }

    _findSharedDeps() {
        var crossDeps = this._resolveCrossNamespaceDeps();
        var edgeSet = {};
        var edges = [];
        for (var i = 0; i < crossDeps.length; i++) {
            var dep = crossDeps[i];
            if (!dep.resolved || !dep.toCompId) continue;
            if (dep.fromCompId === dep.toCompId) continue;
            var key = Math.min(dep.fromCompId, dep.toCompId) + '-' + Math.max(dep.fromCompId, dep.toCompId);
            if (!edgeSet[key]) {
                edgeSet[key] = true;
                edges.push({ from: dep.fromCompId, to: dep.toCompId });
            }
        }
        return edges;
    }

    _buildConnectionMap() {
        var connections = [];
        for (var ci = 0; ci < this.computers.length; ci++) {
            var comp = this.computers[ci];
            for (var ni = 0; ni < comp.namespaces.length; ni++) {
                var ns = comp.namespaces[ni];
                var deps = this._resolveCapabilities(ns);
                for (var di = 0; di < deps.length; di++) {
                    var dep = deps[di];
                    connections.push({
                        fromComputer: comp.label,
                        fromNamespace: ns.label,
                        fromAbstraction: dep.from,
                        toAbstraction: dep.to,
                        resolved: dep.resolved,
                        scope: 'local'
                    });
                }
                var crossDeps = this._resolveCrossNamespaceDeps();
                for (var xi = 0; xi < crossDeps.length; xi++) {
                    var xd = crossDeps[xi];
                    if (xd.fromCompId === comp.id && xd.fromNsId === ns.id) {
                        connections.push({
                            fromComputer: comp.label,
                            fromNamespace: ns.label,
                            fromAbstraction: xd.fromAbsName,
                            toAbstraction: xd.capName,
                            toNamespace: xd.toNsLabel || null,
                            resolved: xd.resolved,
                            scope: 'cross-namespace'
                        });
                    }
                }
            }
        }
        return connections;
    }

    _computeLoadOrder(ns) {
        var abstractions = ns.abstractions.slice();
        var ordered = [];
        var placed = {};
        var maxIter = abstractions.length * abstractions.length + 1;
        var iter = 0;
        while (abstractions.length > 0 && iter < maxIter) {
            iter++;
            for (var i = abstractions.length - 1; i >= 0; i--) {
                var abs = abstractions[i];
                var allResolved = true;
                for (var j = 0; j < abs.capabilities.length; j++) {
                    var capName = abs.capabilities[j].name.toUpperCase();
                    if (!placed[capName]) {
                        allResolved = false;
                        break;
                    }
                }
                if (allResolved) {
                    ordered.push(abs.name);
                    placed[abs.name.toUpperCase()] = true;
                    abstractions.splice(i, 1);
                }
            }
        }
        for (var k = 0; k < abstractions.length; k++) {
            ordered.push(abstractions[k].name);
        }
        return ordered;
    }

    exportTopology() {
        var self = this;
        var topology = {
            version: '1.0',
            exported: new Date().toISOString(),
            computers: this.computers.map(function(comp) {
                return {
                    id: comp.id,
                    label: comp.label,
                    position: { x: comp.x, y: comp.y },
                    namespaces: comp.namespaces.map(function(ns) {
                        return {
                            id: ns.id,
                            label: ns.label,
                            loadOrder: self._computeLoadOrder(ns),
                            abstractions: ns.abstractions.map(function(abs) {
                                return {
                                    name: abs.name,
                                    methods: abs.methods,
                                    capabilities: abs.capabilities,
                                    grants: abs.grants,
                                    codeSize: abs.codeSize
                                };
                            })
                        };
                    })
                };
            }),
            connectionMap: this._buildConnectionMap()
        };
        var blob = new Blob([JSON.stringify(topology, null, 2)], { type: 'application/json' });
        var url = URL.createObjectURL(blob);
        var a = document.createElement('a');
        a.href = url;
        a.download = 'topology.json';
        a.click();
        URL.revokeObjectURL(url);
    }

    clearAll() {
        this.computers = [];
        this.nextComputerId = 1;
        this.nextNamespaceId = 1;
        this.nextAbstractionId = 1;
        this.selectedComputer = null;
        this.selectedNamespace = null;
        this.error = null;
        this._saveState();
        this.render();
    }

    _renderSvgCanvas() {
        var W = 800, H = 500;
        var edges = this._findSharedDeps();
        var svg = '<svg class="builder-svg" width="100%" height="100%" viewBox="0 0 ' + W + ' ' + H + '" xmlns="http://www.w3.org/2000/svg">';
        svg += '<defs><marker id="arrowResolved" markerWidth="8" markerHeight="6" refX="8" refY="3" orient="auto"><polygon points="0 0, 8 3, 0 6" fill="#4ade80"/></marker>';
        svg += '<marker id="arrowUnresolved" markerWidth="8" markerHeight="6" refX="8" refY="3" orient="auto"><polygon points="0 0, 8 3, 0 6" fill="#f87171"/></marker>';
        svg += '<marker id="arrowCross" markerWidth="8" markerHeight="6" refX="8" refY="3" orient="auto"><polygon points="0 0, 8 3, 0 6" fill="#fbbf24"/></marker></defs>';
        svg += '<rect width="' + W + '" height="' + H + '" fill="#0a0a1a" rx="6" class="builder-canvas-bg"/>';

        for (var ei = 0; ei < edges.length; ei++) {
            var edge = edges[ei];
            var a = this.computers.find(function(c) { return c.id === edge.from; });
            var b = this.computers.find(function(c) { return c.id === edge.to; });
            if (a && b) {
                svg += '<line x1="' + (a.x + 75) + '" y1="' + (a.y + 30) + '" x2="' + (b.x + 75) + '" y2="' + (b.y + 30) + '" stroke="#3a86ff" stroke-width="1.5" stroke-dasharray="6,3" opacity="0.5"/>';
            }
        }

        for (var ci = 0; ci < this.computers.length; ci++) {
            var comp = this.computers[ci];
            var isSelected = this.selectedComputer && this.selectedComputer.id === comp.id;
            var stroke = isSelected ? '#fbbf24' : '#3a86ff';
            var fill = isSelected ? '#1a1500' : '#111833';
            var sw = isSelected ? 2.5 : 1.5;
            svg += '<g class="builder-node" data-comp-id="' + comp.id + '" style="cursor:pointer;">';
            svg += '<rect x="' + comp.x + '" y="' + comp.y + '" width="150" height="60" rx="6" fill="' + fill + '" stroke="' + stroke + '" stroke-width="' + sw + '"/>';
            svg += '<text x="' + (comp.x + 75) + '" y="' + (comp.y + 22) + '" fill="' + stroke + '" font-size="12" font-weight="bold" text-anchor="middle" font-family="monospace">' + this._esc(comp.label) + '</text>';
            svg += '<text x="' + (comp.x + 75) + '" y="' + (comp.y + 40) + '" fill="#888" font-size="10" text-anchor="middle" font-family="monospace">' + comp.namespaces.length + ' namespace' + (comp.namespaces.length !== 1 ? 's' : '') + '</text>';
            svg += '<text x="' + (comp.x + 140) + '" y="' + (comp.y + 14) + '" fill="#f87171" font-size="11" font-weight="bold" text-anchor="middle" class="builder-remove-node" data-comp-id="' + comp.id + '" style="cursor:pointer;">\u00d7</text>';
            svg += '</g>';
        }

        if (this.computers.length === 0) {
            svg += '<text x="' + (W / 2) + '" y="' + (H / 2 - 10) + '" fill="#555" font-size="14" text-anchor="middle" font-family="sans-serif">Drop or click to add a computer</text>';
            svg += '<text x="' + (W / 2) + '" y="' + (H / 2 + 12) + '" fill="#444" font-size="11" text-anchor="middle" font-family="sans-serif">Drag nodes to reposition \u00b7 Click to select</text>';
        }
        svg += '</svg>';
        return svg;
    }

    _renderComputerPanel() {
        var comp = this.selectedComputer;
        if (!comp) return '<div class="builder-panel-empty">Select a computer on the canvas to see its namespaces</div>';
        var html = '';
        if (this.error) {
            html += '<div class="builder-error">' + this._esc(this.error) + '</div>';
        }
        html += '<div class="builder-panel-header">';
        html += '<input class="builder-label-input" value="' + this._esc(comp.label) + '" onchange="builder.renameComputer(' + comp.id + ', this.value)" />';
        html += '<button class="btn btn-sm builder-add-btn" onclick="builder.addNamespace(' + comp.id + ')">+ Namespace</button>';
        html += '</div>';
        html += '<div class="builder-ns-list" id="builderNsDropTarget" data-comp-id="' + comp.id + '">';

        html += '<div class="builder-ns-drop-zone" id="nsDropZone">';
        html += '<span class="builder-drop-hint">Drop namespace card here or click + Namespace</span>';
        html += '</div>';

        for (var ni = 0; ni < comp.namespaces.length; ni++) {
            var ns = comp.namespaces[ni];
            var isSelected = this.selectedNamespace && this.selectedNamespace.id === ns.id;
            var cls = isSelected ? 'builder-ns-card selected' : 'builder-ns-card';
            html += '<div class="' + cls + '" draggable="true" data-ns-id="' + ns.id + '" onclick="builder.selectNamespace(' + comp.id + ', ' + ns.id + ')">';
            html += '<div class="builder-ns-card-header">';
            html += '<span class="builder-ns-name">' + this._esc(ns.label) + '</span>';
            html += '<span class="builder-ns-count">' + ns.abstractions.length + ' abstraction' + (ns.abstractions.length !== 1 ? 's' : '') + '</span>';
            html += '<button class="builder-remove-btn" onclick="event.stopPropagation(); builder.removeNamespace(' + comp.id + ', ' + ns.id + ')">\u00d7</button>';
            html += '</div></div>';
        }

        var crossDeps = this._resolveCrossNamespaceDeps();
        var compCross = [];
        for (var xi = 0; xi < crossDeps.length; xi++) {
            if (crossDeps[xi].fromCompId === comp.id) compCross.push(crossDeps[xi]);
        }
        if (compCross.length > 0) {
            html += '<div class="builder-cross-deps-section">';
            html += '<div class="builder-cross-deps-title">Cross-namespace dependencies</div>';
            for (var xj = 0; xj < compCross.length; xj++) {
                var xd = compCross[xj];
                var cls2 = xd.resolved ? 'builder-cross-dep resolved' : 'builder-cross-dep unresolved';
                var icon = xd.resolved ? '\u2714' : '\u26a0';
                var target = xd.resolved ? ' \u2192 ' + this._esc(xd.toNsLabel) : ' (unresolved)';
                html += '<div class="' + cls2 + '">' + icon + ' ' + this._esc(xd.fromAbsName) + ' needs ' + this._esc(xd.capName) + target + '</div>';
            }
            html += '</div>';
        }

        html += '</div>';
        return html;
    }

    _renderNamespacePanel() {
        var comp = this.selectedComputer;
        var ns = this.selectedNamespace;
        if (!comp || !ns) return '';
        var html = '<div class="builder-panel-header">';
        html += '<input class="builder-label-input" value="' + this._esc(ns.label) + '" onchange="builder.renameNamespace(' + comp.id + ', ' + ns.id + ', this.value)" />';
        html += '<button class="btn btn-sm builder-back-btn" onclick="builder.selectedNamespace = null; builder.render()">\u2190 Back</button>';
        html += '</div>';

        html += '<div class="builder-drop-zone" id="builderDropZone">';
        html += '<div class="builder-drop-icon">\u2b07</div>';
        html += '<div class="builder-drop-text">Drop .json abstraction file here</div>';
        html += '<div class="builder-drop-sub">or <label class="builder-file-label"><input type="file" accept=".json" onchange="builder.handleFilePick(event, ' + comp.id + ', ' + ns.id + ')" style="display:none;"/>click to browse</label></div>';
        html += '</div>';

        if (this.error) {
            html += '<div class="builder-error">' + this._esc(this.error) + '</div>';
        }

        var deps = this._resolveCapabilities(ns);

        if (ns.abstractions.length > 0) {
            html += '<div class="builder-abs-list" id="builderAbsList">';

            for (var ai = 0; ai < ns.abstractions.length; ai++) {
                var abs = ns.abstractions[ai];
                var absDeps = deps.filter(function(d) { return d.fromId === abs.id; });
                html += '<div class="builder-abs-card" data-abs-id="' + abs.id + '">';
                html += '<div class="builder-abs-header">';
                html += '<span class="builder-abs-name">' + this._esc(abs.name) + '</span>';
                html += '<span class="builder-abs-meta">' + abs.methodCount + ' method' + (abs.methodCount !== 1 ? 's' : '') + ' \u00b7 ' + abs.codeSize + ' words</span>';
                html += '<button class="builder-remove-btn" onclick="builder.removeAbstraction(' + comp.id + ', ' + ns.id + ', ' + abs.id + ')">\u00d7</button>';
                html += '</div>';
                html += '<div class="builder-abs-grants">Grants: ' + this._esc(abs.grants.join(', ')) + '</div>';
                if (abs.methods.length > 0) {
                    html += '<div class="builder-abs-methods">Methods: ' + this._esc(abs.methods.join(', ')) + '</div>';
                }
                if (absDeps.length > 0) {
                    var depsId = 'deps-' + abs.id;
                    html += '<details class="builder-abs-deps-details">';
                    html += '<summary class="builder-deps-label">Requires (' + absDeps.length + ')</summary>';
                    html += '<div class="builder-abs-deps">';
                    for (var di = 0; di < absDeps.length; di++) {
                        var dep = absDeps[di];
                        var dcls = dep.resolved ? 'builder-dep resolved' : 'builder-dep unresolved';
                        var dicon = dep.resolved ? '\u2714' : '\u26a0';
                        html += '<span class="' + dcls + '">' + dicon + ' ' + this._esc(dep.to) + '</span>';
                    }
                    html += '</div>';
                    html += '</details>';
                }
                html += '</div>';
            }
            html += '</div>';
        }
        return html;
    }

    _drawDependencyArrows() {
        var listEl = document.getElementById('builderAbsList');
        if (!listEl) return;
        var comp = this.selectedComputer;
        var ns = this.selectedNamespace;
        if (!comp || !ns || ns.abstractions.length < 2) return;
        var deps = this._resolveCapabilities(ns);
        if (deps.length === 0) return;

        var cards = listEl.querySelectorAll('.builder-abs-card');
        var cardMap = {};
        for (var i = 0; i < cards.length; i++) {
            var absId = parseInt(cards[i].getAttribute('data-abs-id'));
            var rect = cards[i].getBoundingClientRect();
            var listRect = listEl.getBoundingClientRect();
            cardMap[absId] = {
                top: rect.top - listRect.top,
                bottom: rect.bottom - listRect.top,
                left: rect.left - listRect.left,
                right: rect.right - listRect.left,
                midY: (rect.top + rect.bottom) / 2 - listRect.top,
                width: rect.width
            };
        }

        var existingSvg = listEl.querySelector('.builder-dep-arrows');
        if (existingSvg) existingSvg.remove();

        var svgNs = 'http://www.w3.org/2000/svg';
        var svgEl = document.createElementNS(svgNs, 'svg');
        svgEl.setAttribute('class', 'builder-dep-arrows');
        svgEl.setAttribute('style', 'position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none;overflow:visible;');
        svgEl.setAttribute('xmlns', svgNs);

        var defs = document.createElementNS(svgNs, 'defs');
        var markers = [
            { id: 'arrR', color: '#4ade80' },
            { id: 'arrU', color: '#f87171' }
        ];
        for (var mi = 0; mi < markers.length; mi++) {
            var marker = document.createElementNS(svgNs, 'marker');
            marker.setAttribute('id', markers[mi].id);
            marker.setAttribute('markerWidth', '8');
            marker.setAttribute('markerHeight', '6');
            marker.setAttribute('refX', '8');
            marker.setAttribute('refY', '3');
            marker.setAttribute('orient', 'auto');
            var poly = document.createElementNS(svgNs, 'polygon');
            poly.setAttribute('points', '0 0, 8 3, 0 6');
            poly.setAttribute('fill', markers[mi].color);
            marker.appendChild(poly);
            defs.appendChild(marker);
        }
        svgEl.appendChild(defs);

        for (var di = 0; di < deps.length; di++) {
            var dep = deps[di];
            var fromBox = cardMap[dep.fromId];
            var toBox = dep.toId ? cardMap[dep.toId] : null;
            if (!fromBox) continue;

            if (dep.resolved && toBox) {
                var line = document.createElementNS(svgNs, 'line');
                var x1 = fromBox.width - 5;
                var y1 = fromBox.midY;
                var x2 = toBox.width - 5;
                var y2 = toBox.midY;
                var offset = (di % 3) * 6 + 12;
                var path = document.createElementNS(svgNs, 'path');
                var cx = fromBox.width + offset;
                var d = 'M ' + x1 + ' ' + y1 + ' C ' + cx + ' ' + y1 + ', ' + cx + ' ' + y2 + ', ' + x2 + ' ' + y2;
                path.setAttribute('d', d);
                path.setAttribute('stroke', '#4ade80');
                path.setAttribute('stroke-width', '1.5');
                path.setAttribute('fill', 'none');
                path.setAttribute('marker-end', 'url(#arrR)');
                path.setAttribute('opacity', '0.7');
                svgEl.appendChild(path);
            } else {
                var dashLine = document.createElementNS(svgNs, 'line');
                dashLine.setAttribute('x1', fromBox.width - 5);
                dashLine.setAttribute('y1', fromBox.midY);
                dashLine.setAttribute('x2', fromBox.width + 30);
                dashLine.setAttribute('y2', fromBox.midY);
                dashLine.setAttribute('stroke', '#f87171');
                dashLine.setAttribute('stroke-width', '1.5');
                dashLine.setAttribute('stroke-dasharray', '4,3');
                dashLine.setAttribute('marker-end', 'url(#arrU)');
                dashLine.setAttribute('opacity', '0.7');
                svgEl.appendChild(dashLine);

                var label = document.createElementNS(svgNs, 'text');
                label.setAttribute('x', fromBox.width + 34);
                label.setAttribute('y', fromBox.midY + 4);
                label.setAttribute('fill', '#f87171');
                label.setAttribute('font-size', '9');
                label.setAttribute('font-family', 'monospace');
                label.textContent = '? ' + dep.to;
                svgEl.appendChild(label);
            }
        }

        listEl.style.position = 'relative';
        listEl.appendChild(svgEl);
    }

    renameComputer(id, label) {
        var comp = this.computers.find(function(c) { return c.id === id; });
        if (comp) { comp.label = label; this._saveState(); this.render(); }
    }

    renameNamespace(compId, nsId, label) {
        var comp = this.computers.find(function(c) { return c.id === compId; });
        if (!comp) return;
        var ns = comp.namespaces.find(function(n) { return n.id === nsId; });
        if (ns) { ns.label = label; this._saveState(); this.render(); }
    }

    handleFilePick(event, compId, nsId) {
        var file = event.target.files[0];
        if (!file) return;
        this._readJsonFile(file, compId, nsId);
        event.target.value = '';
    }

    _readJsonFile(file, compId, nsId) {
        var self = this;
        var reader = new FileReader();
        reader.onload = function(e) {
            try {
                var json = JSON.parse(e.target.result);
                self.addAbstraction(compId, nsId, json);
            } catch (err) {
                self.error = 'Failed to parse JSON: ' + err.message;
                self.render();
            }
        };
        reader.readAsText(file);
    }

    render() {
        var container = document.getElementById('builderView');
        if (!container) return;

        var html = '<div class="builder-layout">';

        html += '<div class="builder-canvas-area">';
        html += '<div class="builder-toolbar">';
        html += '<span class="builder-title">Universal Cyberspace</span>';
        html += '<div class="builder-toolbar-actions">';
        html += '<div class="builder-palette-card" draggable="true" id="paletteComputer" title="Drag onto canvas to add a computer">\u2395 Computer</div>';
        html += '<div class="builder-palette-card" draggable="true" id="paletteNamespace" title="Drag onto computer panel to add a namespace">\u2630 Namespace</div>';
        html += '<button class="btn btn-sm builder-export-btn" onclick="builder.exportTopology()">Save Topology</button>';
        html += '<button class="btn btn-sm builder-clear-btn" onclick="if(confirm(\'Clear all?\')) builder.clearAll()">Clear All</button>';
        html += '</div>';
        html += '</div>';
        html += '<div class="builder-canvas" id="builderCanvas">';
        html += this._renderSvgCanvas();
        html += '</div>';
        html += '</div>';

        html += '<div class="builder-side-panel">';
        if (this.selectedNamespace) {
            html += this._renderNamespacePanel();
        } else {
            html += this._renderComputerPanel();
        }
        html += '</div>';

        html += '</div>';
        container.innerHTML = html;

        this._attachCanvasEvents();
        this._attachDropZoneEvents();
        this._attachNsDropEvents();
        this._attachPaletteDragEvents();
        this._drawDependencyArrows();
        this._drawCrossNamespaceLines();
    }

    _getSvgCoords(e, svg) {
        var svgRect = svg.getBoundingClientRect();
        var scaleX = 800 / svgRect.width;
        var scaleY = 500 / svgRect.height;
        return {
            x: (e.clientX - svgRect.left) * scaleX,
            y: (e.clientY - svgRect.top) * scaleY
        };
    }

    _updateDraggedNodeSvg() {
        if (!this.draggingNode) return;
        var comp = this.draggingNode;
        var svg = document.querySelector('#builderCanvas .builder-svg');
        if (!svg) return;
        var group = svg.querySelector('.builder-node[data-comp-id="' + comp.id + '"]');
        if (!group) return;
        var rect = group.querySelector('rect');
        var texts = group.querySelectorAll('text');
        if (rect) {
            rect.setAttribute('x', comp.x);
            rect.setAttribute('y', comp.y);
        }
        if (texts[0]) {
            texts[0].setAttribute('x', comp.x + 75);
            texts[0].setAttribute('y', comp.y + 22);
        }
        if (texts[1]) {
            texts[1].setAttribute('x', comp.x + 75);
            texts[1].setAttribute('y', comp.y + 40);
        }
        if (texts[2]) {
            texts[2].setAttribute('x', comp.x + 140);
            texts[2].setAttribute('y', comp.y + 14);
        }

        var edges = this._findSharedDeps();
        var lines = svg.querySelectorAll('line');
        var lineIdx = 0;
        for (var ei = 0; ei < edges.length; ei++) {
            var a = this.computers.find(function(c) { return c.id === edges[ei].from; });
            var b = this.computers.find(function(c) { return c.id === edges[ei].to; });
            if (a && b && lineIdx < lines.length) {
                lines[lineIdx].setAttribute('x1', a.x + 75);
                lines[lineIdx].setAttribute('y1', a.y + 30);
                lines[lineIdx].setAttribute('x2', b.x + 75);
                lines[lineIdx].setAttribute('y2', b.y + 30);
                lineIdx++;
            }
        }
    }

    _onDocMouseMove(e) {
        if (!this.draggingNode) return;
        var svg = document.querySelector('#builderCanvas .builder-svg');
        if (!svg) return;
        var coords = this._getSvgCoords(e, svg);
        this.draggingNode.x = Math.max(0, Math.min(650, coords.x - this.dragOffset.x));
        this.draggingNode.y = Math.max(0, Math.min(440, coords.y - this.dragOffset.y));
        this._dragMoved = true;
        this._updateDraggedNodeSvg();
    }

    _onDocMouseUp() {
        if (this.draggingNode) {
            this._saveState();
            if (this._dragMoved) {
                this.render();
            }
            this.draggingNode = null;
            this._dragMoved = false;
        }
    }

    _attachCanvasEvents() {
        var canvas = document.getElementById('builderCanvas');
        if (!canvas) return;
        var svg = canvas.querySelector('.builder-svg');
        if (!svg) return;
        var self = this;

        if (!this._docListenersAttached) {
            document.addEventListener('mousemove', this._boundDocMouseMove);
            document.addEventListener('mouseup', this._boundDocMouseUp);
            this._docListenersAttached = true;
        }

        svg.querySelectorAll('.builder-remove-node').forEach(function(el) {
            el.addEventListener('click', function(e) {
                e.stopPropagation();
                var id = parseInt(this.getAttribute('data-comp-id'));
                self.removeComputer(id);
            });
        });

        svg.querySelectorAll('.builder-node').forEach(function(el) {
            el.addEventListener('mousedown', function(e) {
                if (e.target.classList.contains('builder-remove-node')) return;
                var id = parseInt(this.getAttribute('data-comp-id'));
                var comp = self.computers.find(function(c) { return c.id === id; });
                if (!comp) return;
                self.draggingNode = comp;
                self._dragMoved = false;
                var coords = self._getSvgCoords(e, svg);
                self.dragOffset = {
                    x: coords.x - comp.x,
                    y: coords.y - comp.y
                };
                e.preventDefault();
            });
        });

        var bgRect = svg.querySelector('.builder-canvas-bg');
        if (bgRect) {
            bgRect.addEventListener('dragover', function(e) {
                e.preventDefault();
                e.dataTransfer.dropEffect = 'copy';
            });
            bgRect.addEventListener('drop', function(e) {
                e.preventDefault();
                var type = e.dataTransfer.getData('text/plain');
                if (type === 'new-computer') {
                    var coords = self._getSvgCoords(e, svg);
                    self.addComputer(Math.max(0, Math.min(650, coords.x - 75)), Math.max(0, Math.min(440, coords.y - 30)));
                }
            });
        }

        svg.addEventListener('dragover', function(e) {
            e.preventDefault();
            e.dataTransfer.dropEffect = 'copy';
        });
        svg.addEventListener('drop', function(e) {
            e.preventDefault();
            var type = e.dataTransfer.getData('text/plain');
            if (type === 'new-computer') {
                var coords = self._getSvgCoords(e, svg);
                self.addComputer(Math.max(0, Math.min(650, coords.x - 75)), Math.max(0, Math.min(440, coords.y - 30)));
            }
        });

        svg.addEventListener('click', function(e) {
            if (self._dragMoved) return;
            if (self.draggingNode) return;
            if (e.target.classList.contains('builder-remove-node')) return;
            var nodeEl = e.target.closest('.builder-node');
            if (nodeEl) {
                var id = parseInt(nodeEl.getAttribute('data-comp-id'));
                self.selectComputer(id);
                return;
            }
            var coords = self._getSvgCoords(e, svg);
            self.addComputer(Math.max(0, Math.min(650, coords.x - 75)), Math.max(0, Math.min(440, coords.y - 30)));
        });
    }

    _attachNsDropEvents() {
        var nsDropZone = document.getElementById('nsDropZone');
        if (!nsDropZone || !this.selectedComputer) return;
        var compId = this.selectedComputer.id;
        var self = this;

        nsDropZone.addEventListener('dragover', function(e) {
            e.preventDefault();
            e.dataTransfer.dropEffect = 'copy';
            nsDropZone.classList.add('drag-over');
        });
        nsDropZone.addEventListener('dragleave', function(e) {
            nsDropZone.classList.remove('drag-over');
        });
        nsDropZone.addEventListener('drop', function(e) {
            e.preventDefault();
            nsDropZone.classList.remove('drag-over');
            var type = e.dataTransfer.getData('text/plain');
            if (type === 'new-namespace' || type === '') {
                self.addNamespace(compId);
            }
        });

        var nsCards = document.querySelectorAll('.builder-ns-card[draggable]');
        nsCards.forEach(function(card) {
            card.addEventListener('dragstart', function(e) {
                e.dataTransfer.setData('text/plain', 'namespace-' + card.getAttribute('data-ns-id'));
                e.dataTransfer.effectAllowed = 'move';
            });
        });
    }

    _attachDropZoneEvents() {
        var dropZone = document.getElementById('builderDropZone');
        if (!dropZone || !this.selectedComputer || !this.selectedNamespace) return;
        var compId = this.selectedComputer.id;
        var nsId = this.selectedNamespace.id;
        var self = this;

        dropZone.addEventListener('dragover', function(e) {
            e.preventDefault();
            e.stopPropagation();
            dropZone.classList.add('drag-over');
        });
        dropZone.addEventListener('dragleave', function(e) {
            e.preventDefault();
            e.stopPropagation();
            dropZone.classList.remove('drag-over');
        });
        dropZone.addEventListener('drop', function(e) {
            e.preventDefault();
            e.stopPropagation();
            dropZone.classList.remove('drag-over');
            var files = e.dataTransfer.files;
            if (files.length > 0) {
                for (var i = 0; i < files.length; i++) {
                    self._readJsonFile(files[i], compId, nsId);
                }
            }
        });
    }

    _attachPaletteDragEvents() {
        var palComp = document.getElementById('paletteComputer');
        var palNs = document.getElementById('paletteNamespace');
        if (palComp) {
            palComp.addEventListener('dragstart', function(e) {
                e.dataTransfer.setData('text/plain', 'new-computer');
                e.dataTransfer.effectAllowed = 'copy';
            });
        }
        if (palNs) {
            palNs.addEventListener('dragstart', function(e) {
                e.dataTransfer.setData('text/plain', 'new-namespace');
                e.dataTransfer.effectAllowed = 'copy';
            });
        }
    }

    _drawCrossNamespaceLines() {
        var comp = this.selectedComputer;
        if (!comp || this.selectedNamespace) return;
        var nsCards = document.querySelectorAll('.builder-ns-card[data-ns-id]');
        if (nsCards.length < 2) return;

        var crossDeps = this._resolveCrossNamespaceDeps();
        var compCross = [];
        for (var xi = 0; xi < crossDeps.length; xi++) {
            if (crossDeps[xi].fromCompId === comp.id && crossDeps[xi].resolved && crossDeps[xi].toCompId === comp.id) {
                compCross.push(crossDeps[xi]);
            }
        }
        if (compCross.length === 0) return;

        var listEl = document.getElementById('builderNsDropTarget');
        if (!listEl) return;
        var listRect = listEl.getBoundingClientRect();

        var cardMap = {};
        for (var i = 0; i < nsCards.length; i++) {
            var nsId = parseInt(nsCards[i].getAttribute('data-ns-id'));
            var rect = nsCards[i].getBoundingClientRect();
            cardMap[nsId] = {
                midY: (rect.top + rect.bottom) / 2 - listRect.top,
                right: rect.right - listRect.left,
                left: rect.left - listRect.left,
                width: rect.width
            };
        }

        var svgNs = 'http://www.w3.org/2000/svg';
        var svgEl = document.createElementNS(svgNs, 'svg');
        svgEl.setAttribute('class', 'builder-cross-ns-lines');
        svgEl.setAttribute('style', 'position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none;overflow:visible;');

        var defs = document.createElementNS(svgNs, 'defs');
        var marker = document.createElementNS(svgNs, 'marker');
        marker.setAttribute('id', 'arrCross');
        marker.setAttribute('markerWidth', '8');
        marker.setAttribute('markerHeight', '6');
        marker.setAttribute('refX', '8');
        marker.setAttribute('refY', '3');
        marker.setAttribute('orient', 'auto');
        var poly = document.createElementNS(svgNs, 'polygon');
        poly.setAttribute('points', '0 0, 8 3, 0 6');
        poly.setAttribute('fill', '#fbbf24');
        marker.appendChild(poly);
        defs.appendChild(marker);
        svgEl.appendChild(defs);

        for (var di = 0; di < compCross.length; di++) {
            var dep = compCross[di];
            var fromBox = cardMap[dep.fromNsId];
            var toBox = cardMap[dep.toNsId];
            if (!fromBox || !toBox || dep.fromNsId === dep.toNsId) continue;

            var offset = (di % 3) * 8 + 15;
            var path = document.createElementNS(svgNs, 'path');
            var x1 = fromBox.width - 5;
            var y1 = fromBox.midY;
            var x2 = toBox.width - 5;
            var y2 = toBox.midY;
            var cx = fromBox.width + offset;
            var d = 'M ' + x1 + ' ' + y1 + ' C ' + cx + ' ' + y1 + ', ' + cx + ' ' + y2 + ', ' + x2 + ' ' + y2;
            path.setAttribute('d', d);
            path.setAttribute('stroke', '#fbbf24');
            path.setAttribute('stroke-width', '1.5');
            path.setAttribute('stroke-dasharray', '6,3');
            path.setAttribute('fill', 'none');
            path.setAttribute('marker-end', 'url(#arrCross)');
            path.setAttribute('opacity', '0.6');
            svgEl.appendChild(path);
        }

        listEl.style.position = 'relative';
        listEl.appendChild(svgEl);
    }
}

var builder;
function initBuilder() {
    if (!builder) builder = new NamespaceBuilder();
    builder.render();
}
