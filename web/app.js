function switchView(viewId) {
    document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
    document.getElementById(viewId).classList.add('active');
}

function updateDisplay() {
    updateContextRegisters();
    updateDataRegisters();
    updateSystemState();
    updateFlags();
}

function updateContextRegisters() {
    const container = document.getElementById('contextRegs');
    container.innerHTML = '';
    
    const roles = {
        6: 'C-LIST LCA',
        7: 'NUCLEUS'
    };
    
    for (let i = 0; i < 8; i++) {
        const reg = simulator.contextRegs[i];
        const isNull = reg.name === 'NULL';
        const role = roles[i] || 'GENERAL';
        
        const row = document.createElement('div');
        row.className = `register-row ${isNull ? 'null' : ''}`;
        row.innerHTML = `
            <span class="name">CR${i}</span>
            <span class="role">${role}</span>
            <span class="value">${reg.name}</span>
            <span class="perms">${reg.perms.join('') || '---'}</span>
        `;
        container.appendChild(row);
    }
}

function updateDataRegisters() {
    const container = document.getElementById('dataRegs');
    container.innerHTML = '';
    
    for (let i = 0; i < 8; i++) {
        const value = simulator.dataRegs[i];
        const hexStr = value.toString(16).toUpperCase().padStart(16, '0');
        
        const row = document.createElement('div');
        row.className = 'register-row';
        row.innerHTML = `
            <span class="name">DR${i}</span>
            <span class="value">0x${hexStr}</span>
        `;
        container.appendChild(row);
    }
}

function updateSystemState() {
    document.getElementById('cr15Name').textContent = simulator.cr15.name;
    document.getElementById('cr8Name').textContent = simulator.cr8.name;
    document.getElementById('ipValue').textContent = simulator.ip;
    document.getElementById('stackDepth').textContent = simulator.stackDepth;
}

function updateFlags() {
    const flagIds = ['flagN', 'flagZ', 'flagC', 'flagV'];
    const flagNames = ['N', 'Z', 'C', 'V'];
    
    flagIds.forEach((id, i) => {
        const el = document.getElementById(id);
        if (simulator.flags[flagNames[i]]) {
            el.classList.add('active');
        } else {
            el.classList.remove('active');
        }
    });
}

function log(message, type = 'info') {
    const logContainer = document.getElementById('outputLog');
    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    entry.textContent = `> ${message}`;
    logContainer.appendChild(entry);
    logContainer.scrollTop = logContainer.scrollHeight;
}

function resetCPU() {
    simulator.reset();
    updateDisplay();
    document.getElementById('outputLog').innerHTML = '';
    log('CPU Reset - All registers cleared', 'info');
}

function stepInstruction() {
    simulator.ip++;
    updateDisplay();
    log(`Step: IP now ${simulator.ip}`, 'info');
}

function runProgram() {
    log('Run mode not yet implemented', 'info');
}

const instructionInfo = {
    ADD: { operands: ['dest', 'src'], help: 'DR[dest] = DR[dest] + DR[src]. Sets NZCV flags.' },
    SUB: { operands: ['dest', 'src'], help: 'DR[dest] = DR[dest] - DR[src]. Sets NZCV flags.' },
    MUL: { operands: ['dest', 'src'], help: 'DR[dest] = DR[dest] * DR[src]. Sets N, Z flags.' },
    NEG: { operands: ['dest', 'src'], help: 'DR[dest] = -DR[src] (two\'s complement negate). Sets NZCV flags.' },
    ADDI: { operands: ['dest', 'immediate'], help: 'DR[dest] = DR[dest] + immediate. Sets NZCV flags.' },
    SUBI: { operands: ['dest', 'immediate'], help: 'DR[dest] = DR[dest] - immediate. Sets NZCV flags.' },
    MOV: { operands: ['dest', 'src'], help: 'DR[dest] = DR[src]. Sets N, Z flags.' },
    MVN: { operands: ['dest', 'src'], help: 'DR[dest] = NOT DR[src] (bitwise). Sets N, Z flags.' },
    AND: { operands: ['dest', 'src'], help: 'DR[dest] = DR[dest] AND DR[src]. Sets N, Z flags.' },
    ORR: { operands: ['dest', 'src'], help: 'DR[dest] = DR[dest] OR DR[src]. Sets N, Z flags.' },
    EOR: { operands: ['dest', 'src'], help: 'DR[dest] = DR[dest] XOR DR[src]. Sets N, Z flags.' },
    BIC: { operands: ['dest', 'src'], help: 'DR[dest] = DR[dest] AND (NOT DR[src]). Bit clear. Sets N, Z flags.' },
    NOT: { operands: ['dest', 'src'], help: 'DR[dest] = NOT DR[src]. Sets N, Z flags.' },
    LSL: { operands: ['dest', 'src', 'amount'], help: 'Logical shift left. DR[dest] = DR[src] << amount. Sets N, Z, C flags.' },
    LSR: { operands: ['dest', 'src', 'amount'], help: 'Logical shift right. DR[dest] = DR[src] >> amount. Sets N, Z, C flags.' },
    ASR: { operands: ['dest', 'src', 'amount'], help: 'Arithmetic shift right (sign extends). Sets N, Z, C flags.' },
    ROR: { operands: ['dest', 'src', 'amount'], help: 'Rotate right. Bits that fall off wrap around. Sets N, Z, C flags.' },
    CMP: { operands: ['reg1', 'reg2'], help: 'Compare DR[reg1] - DR[reg2]. Sets flags only, no result stored.' },
    CMN: { operands: ['reg1', 'reg2'], help: 'Compare negative DR[reg1] + DR[reg2]. Sets flags only.' },
    TST: { operands: ['reg1', 'reg2'], help: 'Test bits DR[reg1] AND DR[reg2]. Sets N, Z flags only.' },
    TEQ: { operands: ['reg1', 'reg2'], help: 'Test equal DR[reg1] XOR DR[reg2]. Sets N, Z flags only.' }
};

function updateInstrHelp() {
    const instr = document.getElementById('instrSelect').value;
    const info = instructionInfo[instr];
    
    const operandContainer = document.getElementById('operandInputs');
    operandContainer.innerHTML = '';
    
    info.operands.forEach((op, i) => {
        if (op === 'immediate' || op === 'amount') {
            const input = document.createElement('input');
            input.type = 'number';
            input.id = `operand${i}`;
            input.placeholder = op;
            input.value = op === 'amount' ? '1' : '0';
            operandContainer.appendChild(input);
        } else {
            const select = document.createElement('select');
            select.id = `operand${i}`;
            for (let r = 0; r < 8; r++) {
                const option = document.createElement('option');
                option.value = r;
                option.textContent = `DR${r}`;
                select.appendChild(option);
            }
            if (i === 1) select.value = '1';
            operandContainer.appendChild(select);
        }
    });
    
    document.getElementById('instrHelp').textContent = info.help;
}

function executeCommand() {
    const instr = document.getElementById('instrSelect').value;
    const info = instructionInfo[instr];
    
    const args = info.operands.map((op, i) => {
        const el = document.getElementById(`operand${i}`);
        return parseInt(el.value);
    });
    
    try {
        const result = simulator.execute(instr, ...args);
        log(`${instr} ${args.join(' ')}: ${result}`, 'success');
        updateDisplay();
    } catch (e) {
        log(`Error: ${e.message}`, 'error');
    }
}

document.addEventListener('DOMContentLoaded', () => {
    updateDisplay();
    updateInstrHelp();
    updateCapabilityExplorer();
    log('PP250 Simulator Ready', 'info');
    log('Select an instruction and click Execute, or use Reset/Step/Run controls', 'info');
});

// ==================== CAPABILITY EXPLORER ====================

function generateGoldenKey() {
    let key = '';
    for (let i = 0; i < 48; i++) {
        key += Math.floor(Math.random() * 16).toString(16).toUpperCase();
    }
    return key.match(/.{1,8}/g).join('-');
}

function createTokenCard(cap, regLabel) {
    const isNull = cap.name === 'NULL';
    const card = document.createElement('div');
    card.className = `token-card ${isNull ? 'null-cap' : ''}`;
    card.onclick = (evt) => showCapabilityDetail(evt, cap, regLabel);
    
    const allPerms = ['R', 'W', 'X', 'L', 'S', 'E', 'B'];
    const permBadges = allPerms.map(p => {
        const hasIt = cap.perms.includes(p);
        return `<span class="perm-badge perm-${p.toLowerCase()} ${hasIt ? '' : 'inactive'}">${p}</span>`;
    }).join('');
    
    card.innerHTML = `
        <div class="token-header">
            <span class="token-name">${cap.name}</span>
            <span class="token-reg">${regLabel}</span>
        </div>
        <div class="token-perms">${permBadges}</div>
        ${cap.locked ? '<div class="lock-indicator">🔒 Locked</div>' : ''}
    `;
    
    return card;
}

function showCapabilityDetail(evt, cap, regLabel) {
    document.querySelectorAll('.token-card').forEach(c => c.classList.remove('selected'));
    if (evt && evt.currentTarget) {
        evt.currentTarget.classList.add('selected');
    }
    
    const panel = document.getElementById('capDetailPanel');
    const allPerms = ['R', 'W', 'X', 'L', 'S', 'E', 'B'];
    const permNames = {
        R: 'Read', W: 'Write', X: 'Execute',
        L: 'Load', S: 'Store', E: 'Enter', B: 'Bind'
    };
    
    const permDisplay = allPerms.map(p => {
        const hasIt = cap.perms.includes(p);
        return `<span class="perm-badge perm-${p.toLowerCase()} ${hasIt ? '' : 'inactive'}" title="${permNames[p]}">${p}</span>`;
    }).join('');
    
    const goldenKey = cap.goldenKey || generateGoldenKey();
    cap.goldenKey = goldenKey;
    
    panel.innerHTML = `
        <h2>${cap.name}</h2>
        <div class="cap-detail-grid">
            <div class="golden-key-display">
                <label>192-bit Golden Token</label>
                <div class="value">${goldenKey}</div>
            </div>
            
            <div class="cap-detail-item">
                <label>Register</label>
                <div class="value">${regLabel}</div>
            </div>
            
            <div class="cap-detail-item">
                <label>Location</label>
                <div class="value">${cap.location.type === 'Literal' ? `Literal: "${cap.location.name}"` : `Local @ ${cap.location.offset}`}</div>
            </div>
            
            <div class="cap-detail-item">
                <label>Status</label>
                <div class="value">${cap.locked ? '🔒 Locked (Immutable)' : '🔓 Unlocked (Mutable)'}</div>
            </div>
            
            <div class="cap-detail-item">
                <label>Permissions</label>
                <div class="perm-display">${permDisplay}</div>
            </div>
        </div>
        
        <div style="margin-top: 1.5rem; padding: 1rem; background: var(--bg-dark); border-radius: 6px;">
            <h3 style="color: var(--warning); margin-bottom: 0.5rem;">What This Capability Grants</h3>
            <ul style="color: var(--text-secondary); padding-left: 1.2rem; line-height: 1.8;">
                ${cap.perms.includes('R') ? '<li><strong>Read:</strong> Can load data from this object</li>' : ''}
                ${cap.perms.includes('W') ? '<li><strong>Write:</strong> Can save data to this object</li>' : ''}
                ${cap.perms.includes('X') ? '<li><strong>Execute:</strong> Can run code stored in this object</li>' : ''}
                ${cap.perms.includes('L') ? '<li><strong>Load:</strong> Can load child capabilities from this namespace</li>' : ''}
                ${cap.perms.includes('S') ? '<li><strong>Store:</strong> Can store capabilities to children</li>' : ''}
                ${cap.perms.includes('E') ? '<li><strong>Enter:</strong> Can CALL/SWITCH into this namespace</li>' : ''}
                ${cap.perms.includes('B') ? '<li><strong>Bind:</strong> Can save/bind this token into namespace DNA (persistent)</li>' : ''}
                ${cap.perms.length === 0 ? '<li>No permissions - this is a NULL capability</li>' : ''}
            </ul>
        </div>
    `;
}

function updateCapabilityExplorer() {
    const systemContainer = document.getElementById('systemTokens');
    const contextContainer = document.getElementById('contextTokens');
    const clistContainer = document.getElementById('clistTokens');
    
    if (!systemContainer) return;
    
    systemContainer.innerHTML = '';
    contextContainer.innerHTML = '';
    clistContainer.innerHTML = '';
    
    systemContainer.appendChild(createTokenCard(simulator.cr15, 'CR15 (Namespace)'));
    systemContainer.appendChild(createTokenCard(simulator.cr8, 'CR8 (Thread)'));
    
    for (let i = 0; i < 8; i++) {
        const cap = simulator.contextRegs[i];
        contextContainer.appendChild(createTokenCard(cap, `CR${i}`));
    }
    
    if (simulator.clist && simulator.clist.length > 0) {
        simulator.clist.forEach((cap, i) => {
            clistContainer.appendChild(createTokenCard(cap, `C-List[${i}]`));
        });
    } else {
        clistContainer.innerHTML = '<p style="color: var(--text-secondary); font-style: italic; padding: 0.5rem;">No capabilities in C-List</p>';
    }
}

function createSampleCapabilities() {
    simulator.cr15 = {
        name: "SYSTEM_ROOT",
        location: { type: "Literal", name: "system.namespace" },
        perms: ["R", "L", "S", "E", "B"],
        locked: true,
        goldenKey: generateGoldenKey()
    };
    
    simulator.cr8 = {
        name: "USER_ALICE",
        location: { type: "Local", offset: 0x2000 },
        perms: ["R", "W"],
        locked: false,
        goldenKey: generateGoldenKey()
    };
    
    simulator.contextRegs[0] = {
        name: "DataBuffer",
        location: { type: "Local", offset: 0x100 },
        perms: ["R", "W"],
        locked: false,
        goldenKey: generateGoldenKey()
    };
    
    simulator.contextRegs[1] = {
        name: "CodeSegment",
        location: { type: "Local", offset: 0x500 },
        perms: ["R", "X"],
        locked: true,
        goldenKey: generateGoldenKey()
    };
    
    simulator.contextRegs[2] = {
        name: "SecureVault",
        location: { type: "Local", offset: 0x800 },
        perms: ["R"],
        locked: true,
        goldenKey: generateGoldenKey()
    };
    
    simulator.contextRegs[6] = {
        name: "UserCList",
        location: { type: "Local", offset: 0x300 },
        perms: ["R", "L", "S", "B"],
        locked: false,
        goldenKey: generateGoldenKey()
    };
    
    simulator.contextRegs[7] = {
        name: "KernelCode",
        location: { type: "Literal", name: "kernel.entry" },
        perms: ["R", "X", "E"],
        locked: true,
        goldenKey: generateGoldenKey()
    };
    
    simulator.clist = [
        {
            name: "PrinterAccess",
            location: { type: "Local", offset: 0x10 },
            perms: ["W"],
            locked: false,
            goldenKey: generateGoldenKey()
        },
        {
            name: "NetworkSocket",
            location: { type: "Local", offset: 0x20 },
            perms: ["R", "W"],
            locked: false,
            goldenKey: generateGoldenKey()
        },
        {
            name: "FileSystem",
            location: { type: "Local", offset: 0x400 },
            perms: ["R", "W", "L", "S"],
            locked: false,
            goldenKey: generateGoldenKey()
        }
    ];
    
    updateCapabilityExplorer();
    updateDisplay();
    log('Sample capabilities loaded - click on tokens to explore!', 'success');
}

// ==================== INSTRUCTION VISUALIZER ====================

const vizInstrInfo = {
    ADD: { operands: ['dest', 'src'], twoReg: true, op: '+', desc: 'Add two registers' },
    SUB: { operands: ['dest', 'src'], twoReg: true, op: '-', desc: 'Subtract source from destination' },
    MUL: { operands: ['dest', 'src'], twoReg: true, op: '*', desc: 'Multiply two registers' },
    NEG: { operands: ['dest', 'src'], twoReg: true, op: 'NEG', desc: 'Negate source into destination' },
    AND: { operands: ['dest', 'src'], twoReg: true, op: 'AND', desc: 'Bitwise AND' },
    ORR: { operands: ['dest', 'src'], twoReg: true, op: 'OR', desc: 'Bitwise OR' },
    EOR: { operands: ['dest', 'src'], twoReg: true, op: 'XOR', desc: 'Bitwise exclusive OR' },
    NOT: { operands: ['dest', 'src'], twoReg: true, op: 'NOT', desc: 'Bitwise NOT' },
    MOV: { operands: ['dest', 'src'], twoReg: true, op: 'MOV', desc: 'Copy value between registers' },
    MVN: { operands: ['dest', 'src'], twoReg: true, op: 'MVN', desc: 'Move NOT (copy inverted value)' },
    LSL: { operands: ['dest', 'src', 'amt'], shift: true, op: '<<', desc: 'Logical shift left' },
    LSR: { operands: ['dest', 'src', 'amt'], shift: true, op: '>>', desc: 'Logical shift right' }
};

let vizState = {
    instr: null,
    dest: 0,
    src: 0,
    amt: 1,
    srcVal1: BigInt(0),
    srcVal2: BigInt(0),
    result: BigInt(0),
    step: 0,
    ready: false
};

function updateVizInstruction() {
    const instr = document.getElementById('vizInstrSelect').value;
    const info = vizInstrInfo[instr];
    const container = document.getElementById('vizOperands');
    
    let html = `
        <div class="viz-operand-row">
            <label>Destination (DR):</label>
            <input type="number" id="vizDest" min="0" max="7" value="0">
        </div>
        <div class="viz-operand-row">
            <label>Source (DR):</label>
            <input type="number" id="vizSrc" min="0" max="7" value="1">
        </div>
    `;
    
    if (info.shift) {
        html += `
            <div class="viz-operand-row">
                <label>Shift Amount:</label>
                <input type="number" id="vizAmt" min="1" max="63" value="4">
            </div>
        `;
    }
    
    html += `
        <div class="viz-operand-row">
            <label>Source Value (hex):</label>
            <input type="text" id="vizSrcValue" value="0x42" placeholder="0x...">
        </div>
    `;
    
    if (info.twoReg && !['NEG', 'NOT', 'MOV', 'MVN'].includes(instr)) {
        html += `
            <div class="viz-operand-row">
                <label>Dest Initial (hex):</label>
                <input type="text" id="vizDestValue" value="0x10" placeholder="0x...">
            </div>
        `;
    }
    
    container.innerHTML = html;
    document.getElementById('vizRunBtn').disabled = true;
    vizState.ready = false;
}

function setupVisualization() {
    const instr = document.getElementById('vizInstrSelect').value;
    const info = vizInstrInfo[instr];
    
    vizState.instr = instr;
    vizState.dest = parseInt(document.getElementById('vizDest').value) || 0;
    vizState.src = parseInt(document.getElementById('vizSrc').value) || 1;
    
    const srcValInput = document.getElementById('vizSrcValue').value;
    vizState.srcVal2 = BigInt(srcValInput.startsWith('0x') ? srcValInput : '0x' + srcValInput);
    
    const destValInput = document.getElementById('vizDestValue');
    if (destValInput) {
        const val = destValInput.value;
        vizState.srcVal1 = BigInt(val.startsWith('0x') ? val : '0x' + val);
    } else {
        vizState.srcVal1 = BigInt(0);
    }
    
    if (info.shift) {
        vizState.amt = parseInt(document.getElementById('vizAmt').value) || 1;
    }
    
    document.getElementById('vizSrcReg1').querySelector('.viz-reg-label').textContent = `DR${vizState.dest}`;
    document.getElementById('vizSrcReg1').querySelector('.viz-reg-value').textContent = formatHex(vizState.srcVal1);
    
    document.getElementById('vizSrcReg2').querySelector('.viz-reg-label').textContent = `DR${vizState.src}`;
    document.getElementById('vizSrcReg2').querySelector('.viz-reg-value').textContent = formatHex(vizState.srcVal2);
    
    document.getElementById('vizALU').querySelector('.viz-alu-op').textContent = info.op;
    document.getElementById('vizALUResult').textContent = '-';
    
    document.getElementById('vizDestReg').querySelector('.viz-reg-label').textContent = `DR${vizState.dest}`;
    document.getElementById('vizDestReg').querySelector('.viz-reg-value').textContent = '-';
    
    ['vizFlagN', 'vizFlagZ', 'vizFlagC', 'vizFlagV'].forEach(id => {
        document.getElementById(id).classList.remove('active', 'changed');
    });
    
    const steps = generateSteps(instr, info);
    const stepsContainer = document.getElementById('vizSteps');
    stepsContainer.innerHTML = steps.map((s, i) => 
        `<div class="viz-step" id="vizStep${i}"><span class="viz-step-num">${i + 1}</span>${s}</div>`
    ).join('');
    
    vizState.step = 0;
    vizState.ready = true;
    document.getElementById('vizRunBtn').disabled = false;
}

function generateSteps(instr, info) {
    const d = vizState.dest, s = vizState.src;
    const steps = [];
    
    steps.push(`Read value from <strong>DR${s}</strong>: ${formatHex(vizState.srcVal2)}`);
    
    if (info.twoReg && !['NEG', 'NOT', 'MOV', 'MVN'].includes(instr)) {
        steps.push(`Read current value from <strong>DR${d}</strong>: ${formatHex(vizState.srcVal1)}`);
    }
    
    let opDesc;
    switch (instr) {
        case 'ADD': opDesc = `Add: ${formatHex(vizState.srcVal1)} + ${formatHex(vizState.srcVal2)}`; break;
        case 'SUB': opDesc = `Subtract: ${formatHex(vizState.srcVal1)} - ${formatHex(vizState.srcVal2)}`; break;
        case 'MUL': opDesc = `Multiply: ${formatHex(vizState.srcVal1)} * ${formatHex(vizState.srcVal2)}`; break;
        case 'NEG': opDesc = `Negate: -${formatHex(vizState.srcVal2)}`; break;
        case 'AND': opDesc = `Bitwise AND: ${formatHex(vizState.srcVal1)} AND ${formatHex(vizState.srcVal2)}`; break;
        case 'ORR': opDesc = `Bitwise OR: ${formatHex(vizState.srcVal1)} OR ${formatHex(vizState.srcVal2)}`; break;
        case 'EOR': opDesc = `Bitwise XOR: ${formatHex(vizState.srcVal1)} XOR ${formatHex(vizState.srcVal2)}`; break;
        case 'NOT': opDesc = `Bitwise NOT: ~${formatHex(vizState.srcVal2)}`; break;
        case 'MOV': opDesc = `Copy value: ${formatHex(vizState.srcVal2)}`; break;
        case 'MVN': opDesc = `Move NOT: ~${formatHex(vizState.srcVal2)}`; break;
        case 'LSL': opDesc = `Shift left by ${vizState.amt}: ${formatHex(vizState.srcVal2)} << ${vizState.amt}`; break;
        case 'LSR': opDesc = `Shift right by ${vizState.amt}: ${formatHex(vizState.srcVal2)} >> ${vizState.amt}`; break;
        default: opDesc = `Execute ${instr}`;
    }
    steps.push(`ALU computes: ${opDesc}`);
    
    const result = computeResult(instr);
    vizState.result = result;
    steps.push(`Write result to <strong>DR${d}</strong>: ${formatHex(result)}`);
    
    steps.push(`Update condition flags (N, Z, C, V)`);
    
    return steps;
}

function computeResult(instr) {
    const mask = BigInt("0xFFFFFFFFFFFFFFFF");
    const a = vizState.srcVal1;
    const b = vizState.srcVal2;
    const amt = vizState.amt;
    
    let result;
    switch (instr) {
        case 'ADD': result = (a + b) & mask; break;
        case 'SUB': result = (a - b) & mask; break;
        case 'MUL': result = (a * b) & mask; break;
        case 'NEG': result = (-b) & mask; break;
        case 'AND': result = a & b; break;
        case 'ORR': result = a | b; break;
        case 'EOR': result = a ^ b; break;
        case 'NOT': result = (~b) & mask; break;
        case 'MOV': result = b; break;
        case 'MVN': result = (~b) & mask; break;
        case 'LSL': result = (b << BigInt(amt)) & mask; break;
        case 'LSR': result = b >> BigInt(amt); break;
        default: result = BigInt(0);
    }
    return result;
}

function formatHex(val) {
    if (typeof val === 'bigint') {
        return '0x' + val.toString(16).toUpperCase();
    }
    return '0x' + val.toString(16).toUpperCase();
}

async function runVisualization() {
    if (!vizState.ready) return;
    
    document.getElementById('vizRunBtn').disabled = true;
    const info = vizInstrInfo[vizState.instr];
    const totalSteps = info.twoReg && !['NEG', 'NOT', 'MOV', 'MVN'].includes(vizState.instr) ? 5 : 4;
    
    for (let i = 0; i < totalSteps; i++) {
        await animateStep(i, totalSteps);
        await sleep(800);
    }
    
    document.getElementById('vizRunBtn').disabled = false;
}

async function animateStep(stepNum, totalSteps) {
    document.querySelectorAll('.viz-step').forEach((el, i) => {
        el.classList.remove('active');
        if (i < stepNum) el.classList.add('done');
    });
    
    const currentStep = document.getElementById(`vizStep${stepNum}`);
    if (currentStep) {
        currentStep.classList.add('active');
    }
    
    document.querySelectorAll('.viz-reg-box').forEach(el => el.classList.remove('active', 'highlight'));
    document.getElementById('vizDataFlow').classList.remove('show');
    
    const instr = vizState.instr;
    const hasDestRead = !['NEG', 'NOT', 'MOV', 'MVN'].includes(instr);
    
    if (stepNum === 0) {
        document.getElementById('vizSrcReg2').classList.add('active');
    } else if (stepNum === 1 && hasDestRead) {
        document.getElementById('vizSrcReg1').classList.add('active');
    } else if ((hasDestRead && stepNum === 2) || (!hasDestRead && stepNum === 1)) {
        document.getElementById('vizDataFlow').classList.add('show');
        document.getElementById('vizALUResult').textContent = formatHex(vizState.result);
    } else if ((hasDestRead && stepNum === 3) || (!hasDestRead && stepNum === 2)) {
        document.getElementById('vizDestReg').classList.add('highlight');
        document.getElementById('vizDestReg').querySelector('.viz-reg-value').textContent = formatHex(vizState.result);
    } else {
        updateVizFlags();
    }
}

function updateVizFlags() {
    const result = vizState.result;
    const signBit = BigInt("0x8000000000000000");
    
    const n = (result & signBit) !== BigInt(0);
    const z = result === BigInt(0);
    
    const flags = { N: n, Z: z, C: false, V: false };
    
    ['N', 'Z', 'C', 'V'].forEach(f => {
        const el = document.getElementById(`vizFlag${f}`);
        if (flags[f]) {
            el.classList.add('active', 'changed');
        } else {
            el.classList.remove('active');
        }
    });
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

function resetVisualization() {
    vizState = {
        instr: null, dest: 0, src: 0, amt: 1,
        srcVal1: BigInt(0), srcVal2: BigInt(0),
        result: BigInt(0), step: 0, ready: false
    };
    
    document.getElementById('vizSrcReg1').querySelector('.viz-reg-label').textContent = 'DR?';
    document.getElementById('vizSrcReg1').querySelector('.viz-reg-value').textContent = '-';
    document.getElementById('vizSrcReg2').querySelector('.viz-reg-label').textContent = 'DR?';
    document.getElementById('vizSrcReg2').querySelector('.viz-reg-value').textContent = '-';
    document.getElementById('vizDestReg').querySelector('.viz-reg-label').textContent = 'DR?';
    document.getElementById('vizDestReg').querySelector('.viz-reg-value').textContent = '-';
    document.getElementById('vizALU').querySelector('.viz-alu-op').textContent = '?';
    document.getElementById('vizALUResult').textContent = '-';
    
    document.querySelectorAll('.viz-reg-box').forEach(el => el.classList.remove('active', 'highlight'));
    document.getElementById('vizDataFlow').classList.remove('show');
    
    ['vizFlagN', 'vizFlagZ', 'vizFlagC', 'vizFlagV'].forEach(id => {
        document.getElementById(id).classList.remove('active', 'changed');
    });
    
    document.getElementById('vizSteps').innerHTML = '<p class="viz-hint">Select an instruction and click Setup to begin</p>';
    document.getElementById('vizRunBtn').disabled = true;
}

document.addEventListener('DOMContentLoaded', () => {
    updateVizInstruction();
});
