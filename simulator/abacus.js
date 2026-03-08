const abacusState = {
    rods: [],
    display: '0',
    trace: [],
    maxTrace: 50,
    rendered: false,
    numRods: 13
};

function abacusInit() {
    abacusState.rods = [];
    for (let i = 0; i < abacusState.numRods; i++) {
        abacusState.rods.push({ heaven: 0, earth: 0 });
    }
    abacusState.display = '0';
}

function abacusTraceLog(lambdaExpr, desc) {
    abacusState.trace.unshift({ lambda: lambdaExpr, desc: desc, time: Date.now() });
    if (abacusState.trace.length > abacusState.maxTrace) abacusState.trace.pop();
}

function abacusGetValue() {
    let value = 0;
    for (let i = 0; i < abacusState.numRods; i++) {
        const rod = abacusState.rods[i];
        const placeValue = Math.pow(10, abacusState.numRods - 1 - i);
        value += (rod.heaven * 5 + rod.earth) * placeValue;
    }
    return value;
}

function abacusToggleHeaven(rodIndex) {
    const rod = abacusState.rods[rodIndex];
    rod.heaven = rod.heaven ? 0 : 1;
    const val = abacusGetValue();
    abacusState.display = val.toLocaleString();
    const place = Math.pow(10, abacusState.numRods - 1 - rodIndex);
    const action = rod.heaven ? 'lower' : 'raise';
    abacusTraceLog(
        `CALL Abacus.${rod.heaven ? 'Add' : 'Sub'}(${5 * place})`,
        `Heaven bead ${action} on rod ${rodIndex + 1} (×${place.toLocaleString()})`
    );
    abacusUpdateDisplay();
}

function abacusToggleEarth(rodIndex, beadIndex) {
    const rod = abacusState.rods[rodIndex];
    if (beadIndex < rod.earth) {
        rod.earth = beadIndex;
    } else {
        rod.earth = beadIndex + 1;
    }
    const val = abacusGetValue();
    abacusState.display = val.toLocaleString();
    const place = Math.pow(10, abacusState.numRods - 1 - rodIndex);
    abacusTraceLog(
        `CALL Abacus.Set(rod${rodIndex + 1}, ${rod.heaven * 5 + rod.earth})`,
        `Earth beads set to ${rod.earth} on rod ${rodIndex + 1} (×${place.toLocaleString()})`
    );
    abacusUpdateDisplay();
}

function abacusClear() {
    abacusInit();
    abacusTraceLog('CALL Abacus.Clear()', 'Reset all rods to zero');
    abacusUpdateDisplay();
}

function abacusUpdateDisplay() {
    abacusRenderDisplay();
}

function abacusRenderDisplay() {
    const container = document.getElementById('abacusContainer');
    if (!container) return;

    const screenVal = container.querySelector('.abacus-readout-value');
    if (screenVal) screenVal.textContent = abacusState.display;

    for (let i = 0; i < abacusState.numRods; i++) {
        const rod = abacusState.rods[i];
        const heavenBead = container.querySelector(`#abacusHeaven${i}`);
        if (heavenBead) {
            heavenBead.classList.toggle('active', rod.heaven === 1);
        }
        for (let j = 0; j < 4; j++) {
            const earthBead = container.querySelector(`#abacusEarth${i}_${j}`);
            if (earthBead) {
                earthBead.classList.toggle('active', j < rod.earth);
            }
        }
    }

    const sidebar = document.getElementById('sidebarTraceContent');
    const traceArea = sidebar ? sidebar.querySelector('.abacus-trace-area') : null;
    if (traceArea) {
        traceArea.innerHTML = abacusState.trace.map((t, i) =>
            `<div class="abacus-trace-entry${i === 0 ? ' abacus-trace-latest' : ''}">
                <div class="abacus-trace-lambda">${t.lambda}</div>
                <div class="abacus-trace-desc">${t.desc}</div>
            </div>`
        ).join('');
    }
}

function renderAbacusCalculator() {
    const container = document.getElementById('abacusContainer');
    if (!container) return;

    abacusInit();

    let rodsHTML = '';
    for (let i = 0; i < abacusState.numRods; i++) {
        const placeLabel = Math.pow(10, abacusState.numRods - 1 - i);
        let label = '';
        if (placeLabel >= 1000000000000) label = 'T';
        else if (placeLabel >= 1000000000) label = 'B';
        else if (placeLabel >= 1000000) label = 'M';
        else if (placeLabel >= 1000) label = 'K';
        else label = placeLabel.toString();

        rodsHTML += `
        <div class="abacus-rod">
            <div class="abacus-rod-label">${label}</div>
            <div class="abacus-heaven-zone">
                <div class="abacus-bead abacus-heaven-bead" id="abacusHeaven${i}" onclick="abacusToggleHeaven(${i})"></div>
            </div>
            <div class="abacus-beam-bar"></div>
            <div class="abacus-earth-zone">
                <div class="abacus-bead abacus-earth-bead" id="abacusEarth${i}_3" onclick="abacusToggleEarth(${i}, 3)"></div>
                <div class="abacus-bead abacus-earth-bead" id="abacusEarth${i}_2" onclick="abacusToggleEarth(${i}, 2)"></div>
                <div class="abacus-bead abacus-earth-bead" id="abacusEarth${i}_1" onclick="abacusToggleEarth(${i}, 1)"></div>
                <div class="abacus-bead abacus-earth-bead" id="abacusEarth${i}_0" onclick="abacusToggleEarth(${i}, 0)"></div>
            </div>
        </div>`;
    }

    container.innerHTML = `
    <div class="abacus-tile-grid">
        <div class="abacus-tile-column">
            <div class="abacus-tile abacus-tile-calc">
                <div class="abacus-frame">
                    <div class="abacus-title">
                        <span class="abacus-title-label">SOROBAN</span>
                        <span class="abacus-title-ns">NS[17] \u00b7 Abacus</span>
                    </div>
                    <div class="abacus-readout">
                        <span class="abacus-readout-value">0</span>
                    </div>
                    <div class="abacus-rods-area">
                        ${rodsHTML}
                    </div>
                    <div class="abacus-controls">
                        <button class="abacus-btn" onclick="abacusClear()">Clear</button>
                    </div>
                    <div class="abacus-place-info">Each rod: 1 heaven bead (5) + 4 earth beads (1 each) = 0\u20139 per digit</div>
                </div>
            </div>
        </div>


    </div>`;

    abacusState.rendered = true;
    abacusUpdateDisplay();
}
