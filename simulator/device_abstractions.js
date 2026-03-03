class DeviceAbstractions {
    constructor(registry) {
        this.registry = registry;
        this.IO_SEGMENT = 0xFE00;
        this._deviceState = {
            uart: { txBuffer: [], rxBuffer: [], baud: 115200 },
            led: { state: 0x00, count: 6 },
            button: { pressed: false, eventQueue: [] },
            timer: { running: false, count: 0, alarm: 0, startTime: 0 },
            display: { buffer: [], width: 80, height: 25, cursorX: 0, cursorY: 0 }
        };
        this._bindAll();
    }

    _bindAll() {
        this._bindUART();
        this._bindLED();
        this._bindButton();
        this._bindTimer();
        this._bindDisplay();
    }

    _validateDeviceAccess(sim, deviceAddr, perm) {
        const segment = (deviceAddr >>> 8) & 0xFF;
        if (segment !== 0xFE) {
            return { ok: false, fault: 'SEGMENT', message: `Device access requires 0xFE segment, got 0x${segment.toString(16)}` };
        }
        return { ok: true };
    }

    _bindUART() {
        const dev = this._deviceState.uart;
        const self = this;

        this.registry.bindMethod(11, 'Send', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x00, 'S');
            if (!check.ok) return check;

            const data = args.data;
            if (data === undefined || data === null) {
                return { ok: false, fault: 'ARGS', message: 'UART.Send: data required' };
            }

            if (typeof data === 'string') {
                for (let i = 0; i < data.length; i++) {
                    dev.txBuffer.push(data.charCodeAt(i) & 0xFF);
                }
            } else {
                dev.txBuffer.push(data & 0xFF);
            }

            return {
                ok: true,
                result: { bytesSent: typeof data === 'string' ? data.length : 1 },
                message: `UART.Send: ${typeof data === 'string' ? data.length + ' bytes' : '1 byte'} queued at ${dev.baud} baud`
            };
        });

        this.registry.bindMethod(11, 'Receive', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x00, 'L');
            if (!check.ok) return check;

            if (dev.rxBuffer.length === 0) {
                return { ok: true, result: { data: -1, available: 0 }, message: 'UART.Receive: no data available' };
            }

            const byte = dev.rxBuffer.shift();
            return {
                ok: true,
                result: { data: byte, available: dev.rxBuffer.length },
                message: `UART.Receive: byte 0x${byte.toString(16).padStart(2, '0')}, ${dev.rxBuffer.length} remaining`
            };
        });

        this.registry.bindMethod(11, 'SetBaud', function(sim, args) {
            const baud = args.baud || 115200;
            const validRates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];
            if (!validRates.includes(baud)) {
                return { ok: false, fault: 'ARGS', message: `UART.SetBaud: invalid rate ${baud}` };
            }
            dev.baud = baud;
            return { ok: true, result: { baud: baud }, message: `UART.SetBaud: ${baud} baud` };
        });
    }

    _bindLED() {
        const dev = this._deviceState.led;
        const self = this;

        this.registry.bindMethod(12, 'Set', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x10, 'S');
            if (!check.ok) return check;

            const led = args.led;
            if (led === undefined || led < 0 || led >= dev.count) {
                return { ok: false, fault: 'ARGS', message: `LED.Set: led must be 0-${dev.count - 1}` };
            }
            dev.state |= (1 << led);

            return {
                ok: true,
                result: { led: led, state: dev.state },
                message: `LED.Set: LED ${led} ON (state=0b${dev.state.toString(2).padStart(dev.count, '0')})`
            };
        });

        this.registry.bindMethod(12, 'Clear', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x10, 'S');
            if (!check.ok) return check;

            const led = args.led;
            if (led === undefined || led < 0 || led >= dev.count) {
                return { ok: false, fault: 'ARGS', message: `LED.Clear: led must be 0-${dev.count - 1}` };
            }
            dev.state &= ~(1 << led);

            return {
                ok: true,
                result: { led: led, state: dev.state },
                message: `LED.Clear: LED ${led} OFF (state=0b${dev.state.toString(2).padStart(dev.count, '0')})`
            };
        });

        this.registry.bindMethod(12, 'Toggle', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x10, 'S');
            if (!check.ok) return check;

            const led = args.led;
            if (led === undefined || led < 0 || led >= dev.count) {
                return { ok: false, fault: 'ARGS', message: `LED.Toggle: led must be 0-${dev.count - 1}` };
            }
            dev.state ^= (1 << led);
            const isOn = (dev.state >> led) & 1;

            return {
                ok: true,
                result: { led: led, on: isOn, state: dev.state },
                message: `LED.Toggle: LED ${led} ${isOn ? 'ON' : 'OFF'} (state=0b${dev.state.toString(2).padStart(dev.count, '0')})`
            };
        });

        this.registry.bindMethod(12, 'Pattern', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x10, 'S');
            if (!check.ok) return check;

            const pattern = args.pattern & ((1 << dev.count) - 1);
            dev.state = pattern;

            return {
                ok: true,
                result: { state: dev.state },
                message: `LED.Pattern: set to 0b${dev.state.toString(2).padStart(dev.count, '0')}`
            };
        });
    }

    _bindButton() {
        const dev = this._deviceState.button;
        const self = this;

        this.registry.bindMethod(13, 'Read', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x20, 'L');
            if (!check.ok) return check;

            return {
                ok: true,
                result: { pressed: dev.pressed },
                message: `Button.Read: ${dev.pressed ? 'PRESSED' : 'RELEASED'}`
            };
        });

        this.registry.bindMethod(13, 'WaitPress', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x20, 'L');
            if (!check.ok) return check;

            return {
                ok: true,
                result: { pressed: dev.pressed, waiting: !dev.pressed },
                message: dev.pressed ? 'Button.WaitPress: button is pressed' : 'Button.WaitPress: waiting for press'
            };
        });

        this.registry.bindMethod(13, 'OnEvent', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x20, 'L');
            if (!check.ok) return check;

            if (dev.eventQueue.length === 0) {
                return { ok: true, result: { event: null }, message: 'Button.OnEvent: no events pending' };
            }

            const event = dev.eventQueue.shift();
            return {
                ok: true,
                result: { event: event },
                message: `Button.OnEvent: ${event.type} at t=${event.time}`
            };
        });
    }

    _bindTimer() {
        const dev = this._deviceState.timer;
        const self = this;

        this.registry.bindMethod(14, 'Start', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x30, 'S');
            if (!check.ok) return check;

            dev.running = true;
            dev.startTime = Date.now();
            dev.count = 0;

            return { ok: true, result: { running: true }, message: 'Timer.Start: timer started' };
        });

        this.registry.bindMethod(14, 'Stop', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x30, 'S');
            if (!check.ok) return check;

            dev.running = false;
            if (dev.startTime) {
                dev.count = Date.now() - dev.startTime;
            }

            return {
                ok: true,
                result: { running: false, elapsed: dev.count },
                message: `Timer.Stop: stopped at ${dev.count}ms`
            };
        });

        this.registry.bindMethod(14, 'Read', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x30, 'L');
            if (!check.ok) return check;

            let elapsed = dev.count;
            if (dev.running && dev.startTime) {
                elapsed = Date.now() - dev.startTime;
            }

            return {
                ok: true,
                result: { elapsed: elapsed, running: dev.running },
                message: `Timer.Read: ${elapsed}ms${dev.running ? ' (running)' : ' (stopped)'}`
            };
        });

        this.registry.bindMethod(14, 'SetAlarm', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x30, 'S');
            if (!check.ok) return check;

            const ms = args.ms || 1000;
            dev.alarm = ms;

            return { ok: true, result: { alarm: ms }, message: `Timer.SetAlarm: alarm set for ${ms}ms` };
        });
    }

    _bindDisplay() {
        const dev = this._deviceState.display;
        const self = this;

        this.registry.bindMethod(15, 'Write', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x40, 'S');
            if (!check.ok) return check;

            const text = args.text || '';
            for (let i = 0; i < text.length; i++) {
                const ch = text[i];
                if (ch === '\n') {
                    dev.cursorX = 0;
                    dev.cursorY++;
                    if (dev.cursorY >= dev.height) {
                        dev.buffer.shift();
                        dev.cursorY = dev.height - 1;
                    }
                } else {
                    if (!dev.buffer[dev.cursorY]) dev.buffer[dev.cursorY] = [];
                    dev.buffer[dev.cursorY][dev.cursorX] = ch;
                    dev.cursorX++;
                    if (dev.cursorX >= dev.width) {
                        dev.cursorX = 0;
                        dev.cursorY++;
                        if (dev.cursorY >= dev.height) {
                            dev.buffer.shift();
                            dev.cursorY = dev.height - 1;
                        }
                    }
                }
            }

            return {
                ok: true,
                result: { x: dev.cursorX, y: dev.cursorY },
                message: `Display.Write: "${text}" at (${dev.cursorX}, ${dev.cursorY})`
            };
        });

        this.registry.bindMethod(15, 'Clear', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x40, 'S');
            if (!check.ok) return check;

            dev.buffer = [];
            dev.cursorX = 0;
            dev.cursorY = 0;

            return { ok: true, result: {}, message: 'Display.Clear: screen cleared' };
        });

        this.registry.bindMethod(15, 'Scroll', function(sim, args) {
            const check = self._validateDeviceAccess(sim, self.IO_SEGMENT | 0x40, 'S');
            if (!check.ok) return check;

            const lines = args.lines || 1;
            for (let i = 0; i < lines; i++) {
                dev.buffer.shift();
            }

            return {
                ok: true,
                result: { lines: lines, bufferHeight: dev.buffer.length },
                message: `Display.Scroll: scrolled ${lines} line(s)`
            };
        });
    }

    simulateButtonPress() {
        this._deviceState.button.pressed = true;
        this._deviceState.button.eventQueue.push({ type: 'press', time: Date.now() });
    }

    simulateButtonRelease() {
        this._deviceState.button.pressed = false;
        this._deviceState.button.eventQueue.push({ type: 'release', time: Date.now() });
    }

    simulateUARTReceive(data) {
        if (typeof data === 'string') {
            for (let i = 0; i < data.length; i++) {
                this._deviceState.uart.rxBuffer.push(data.charCodeAt(i) & 0xFF);
            }
        } else if (Array.isArray(data)) {
            for (const b of data) {
                this._deviceState.uart.rxBuffer.push(b & 0xFF);
            }
        } else {
            this._deviceState.uart.rxBuffer.push(data & 0xFF);
        }
    }

    getDeviceState() {
        return JSON.parse(JSON.stringify(this._deviceState));
    }

    getLEDState() {
        return this._deviceState.led.state;
    }

    getDisplayText() {
        const lines = [];
        for (let y = 0; y < this._deviceState.display.buffer.length; y++) {
            const row = this._deviceState.display.buffer[y] || [];
            lines.push(row.join(''));
        }
        return lines.join('\n');
    }
}

if (typeof module !== 'undefined' && module.exports) {
    module.exports = DeviceAbstractions;
}
