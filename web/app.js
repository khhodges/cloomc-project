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
    
    const allPerms = ['R', 'W', 'X', 'L', 'S', 'E'];
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
    const allPerms = ['R', 'W', 'X', 'L', 'S', 'E'];
    const permNames = {
        R: 'Read', W: 'Write', X: 'Execute',
        L: 'Load', S: 'Store', E: 'Enter'
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
        perms: ["R", "L", "S", "E"],
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
        perms: ["R", "L", "S"],
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
