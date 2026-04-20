import React, { useState, useEffect, useRef } from "react";
import { Terminal, Settings, Play, Square, Code2, AlertTriangle, Monitor, HardDrive, Cpu, Activity, Database, Cable } from "lucide-react";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";

export function TerminalFirst() {
  const [isConnected, setIsConnected] = useState(true);
  const [isRunning, setIsRunning] = useState(true);
  const [editorOpen, setEditorOpen] = useState(false);
  
  const endOfTerminalRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom of terminal
  useEffect(() => {
    endOfTerminalRef.current?.scrollIntoView({ behavior: "smooth" });
  }, []);

  return (
    <div className="w-full h-screen bg-[#1a1a2e] text-[#eaeaea] font-mono flex flex-col overflow-hidden selection:bg-[#fbbf24] selection:text-[#0f0f23]">
      {/* Top Bar */}
      <div className="h-[40px] flex-shrink-0 border-b border-white/10 flex items-center justify-between px-4 bg-[#0f0f23] shadow-md z-10">
        <div className="flex items-center gap-3">
          <div className="text-[#fbbf24] font-bold text-xl leading-none flex items-center gap-2">
            <span>λ</span>
            <span className="text-sm font-sans tracking-widest uppercase text-white/80">Church Machine</span>
          </div>
        </div>

        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2 text-xs">
            <span className="text-white/40">STATUS</span>
            {isRunning ? (
              <Badge variant="outline" className="bg-emerald-500/10 text-emerald-400 border-emerald-500/20 font-mono rounded-none">
                <Activity className="w-3 h-3 mr-1 animate-pulse" /> RUNNING
              </Badge>
            ) : (
              <Badge variant="outline" className="bg-amber-500/10 text-amber-400 border-amber-500/20 font-mono rounded-none">
                <Square className="w-3 h-3 mr-1" /> HALTED
              </Badge>
            )}
          </div>
          
          <div className="h-4 w-px bg-white/10" />

          <Button 
            variant="outline" 
            size="sm" 
            className={`h-7 text-xs rounded-none font-mono ${isConnected ? 'bg-[#fbbf24]/10 text-[#fbbf24] border-[#fbbf24]/30 hover:bg-[#fbbf24]/20' : 'text-white/60 border-white/20'}`}
            onClick={() => setIsConnected(!isConnected)}
          >
            <Cable className="w-3 h-3 mr-2" />
            {isConnected ? "DISCONNECT" : "CONNECT BOARD"}
          </Button>
        </div>
      </div>

      {/* Main Layout */}
      <div className="flex-1 flex overflow-hidden relative">
        
        {/* LEFT: Terminal (55%) */}
        <div className="w-[55%] flex flex-col border-r border-white/10 relative bg-[#0a0a15] overflow-hidden">
          {/* Subtle Scanline Overlay */}
          <div className="absolute inset-0 pointer-events-none bg-[linear-gradient(rgba(18,16,16,0)_50%,rgba(0,0,0,0.25)_50%),linear-gradient(90deg,rgba(255,0,0,0.06),rgba(0,255,0,0.02),rgba(0,0,255,0.06))] bg-[length:100%_4px,3px_100%] opacity-20 z-10"></div>
          
          <div className="flex items-center justify-between px-4 py-2 border-b border-white/5 bg-[#0f0f23]/80 backdrop-blur-sm sticky top-0 z-20">
            <div className="flex items-center gap-2 text-xs font-semibold text-white/50">
              <Terminal className="w-4 h-4 text-emerald-400" />
              <span>MATH REPL / SERIAL TTY</span>
            </div>
            <div className="flex gap-2">
              <span className="text-[10px] text-white/30">BAUD 115200</span>
              <span className="text-[10px] text-emerald-400/70">COM3</span>
            </div>
          </div>

          <div className="flex-1 overflow-y-auto p-4 text-sm leading-relaxed pb-32">
            <div className="text-white/40 mb-4">
              [SYSTEM] Connected to Church Machine FPGA (Target: Tang Nano 20k)<br/>
              [SYSTEM] Boot ROM loaded successfully.<br/>
              [SYSTEM] Namespace initialized. Type :help for commands.
            </div>

            <TerminalLine prompt="λ" cmd="> ns lookup .std.math.fib" />
            <TerminalLine type="output" text="ADDRESS: 0x4A00BEEF" color="cyan" />
            <TerminalLine type="output" text="PERMISSIONS: RX" color="emerald" />
            
            <br />
            
            <TerminalLine prompt="λ" cmd="> math (call .std.math.fib 10)" />
            <TerminalLine type="output" text="[TRACE] CALL 0x4A00BEEF" color="white/50" />
            <TerminalLine type="output" text="[TRACE] RET  0x4A00BEEF" color="white/50" />
            <TerminalLine type="output" text="RESULT: 55 (0x37)" color="yellow" />
            
            <br />
            
            <TerminalLine prompt="λ" cmd="> dump cr14" />
            <TerminalLine type="output" text="Capability Register 14 (CODE)" color="white/70" />
            <TerminalLine type="output" text="BASE : 0x4A000000" color="cyan" />
            <TerminalLine type="output" text="BOUND: 0x4A00FFFF" color="cyan" />
            <TerminalLine type="output" text="PERMS: R-X----" color="emerald" />
            
            <br />

            <TerminalLine prompt="λ" cmd="> math (call .sys.kernel.panic)" />
            <TerminalLine type="output" text="[FAULT] CAPABILITY VIOLATION" color="red" />
            <TerminalLine type="output" text="ATTEMPTED EXECUTE WITHOUT 'X' PERMISSION AT 0x00000000" color="red" />
            
            <br />
            
            <TerminalLine prompt="λ" cmd="> sys stats" />
            <TerminalLine type="output" text="UPTIME    : 14m 22s" color="yellow" />
            <TerminalLine type="output" text="GC CYCLES : 4" color="yellow" />
            <TerminalLine type="output" text="MEM USAGE : 14.2 KB / 128.0 KB" color="yellow" />

            <br />

            <div className="flex items-center text-emerald-400">
              <span className="mr-3 font-bold text-[#fbbf24]">λ</span>
              <span className="animate-pulse">_</span>
            </div>
            
            <div ref={endOfTerminalRef} />
          </div>
        </div>

        {/* RIGHT: Machine State (45%) */}
        <div className="w-[45%] flex flex-col bg-[#0f0f23] overflow-y-auto">
          <div className="p-4 border-b border-white/5 flex items-center justify-between sticky top-0 bg-[#0f0f23]/95 backdrop-blur z-10">
            <div className="flex items-center gap-2 text-sm font-semibold text-white/80">
              <Cpu className="w-4 h-4 text-[#fbbf24]" />
              HARDWARE REGISTERS
            </div>
            <Badge variant="outline" className="rounded-none bg-white/5 border-white/10 text-xs font-mono">
              CYCLE: 4,091,882
            </Badge>
          </div>

          <div className="p-6 flex flex-col gap-8">
            
            {/* Core Pointers */}
            <div>
              <h3 className="text-[10px] uppercase tracking-widest text-white/40 mb-3 font-sans">Core Pointers</h3>
              <div className="grid grid-cols-2 gap-3">
                <RegisterCard name="PC" label="Program Counter" value="0x4A00BF02" highlight />
                <RegisterCard name="SP" label="Stack Pointer" value="0x7FFFFEE0" />
                <RegisterCard name="TP" label="Thread Pointer" value="0x10004000" />
                <RegisterCard name="NS" label="Namespace" value="0x80000000" />
              </div>
            </div>

            {/* Capability Registers */}
            <div>
              <div className="flex items-center justify-between mb-3">
                <h3 className="text-[10px] uppercase tracking-widest text-white/40 font-sans">Capability Registers</h3>
                <span className="text-[10px] text-emerald-400/70">LIVE</span>
              </div>
              <div className="grid grid-cols-1 gap-2">
                {[
                  { id: "CR0", base: "0x00000000", perms: "RWX", note: "ROOT" },
                  { id: "CR1", base: "0x10000000", perms: "RW-", note: "DATA" },
                  { id: "CR2", base: "0x20000000", perms: "R--", note: "CONST" },
                  { id: "CR3", base: "0x00000000", perms: "---", note: "NULL" },
                  { id: "CR14", base: "0x4A000000", perms: "R-X", note: "CODE", highlight: true },
                  { id: "CR15", base: "0x80000000", perms: "RW-", note: "NAMESPACE", highlight: true },
                ].map((reg) => (
                  <div key={reg.id} className={`flex items-center justify-between p-2 px-3 border ${reg.highlight ? 'border-[#fbbf24]/30 bg-[#fbbf24]/5' : 'border-white/5 bg-white/5'} font-mono text-xs`}>
                    <div className="flex items-center gap-4">
                      <span className={`font-bold w-8 ${reg.highlight ? 'text-[#fbbf24]' : 'text-white/60'}`}>{reg.id}</span>
                      <span className="text-cyan-400/80">{reg.base}</span>
                    </div>
                    <div className="flex items-center gap-4">
                      <span className={reg.perms.includes('---') ? 'text-white/20' : 'text-emerald-400/80'}>{reg.perms}</span>
                      <span className="text-[10px] text-white/40 w-16 text-right">{reg.note}</span>
                    </div>
                  </div>
                ))}
              </div>
              <div className="mt-2 text-center">
                <button className="text-[10px] text-white/30 hover:text-white/60 transition-colors uppercase tracking-widest font-sans">
                  Show All 16 Registers
                </button>
              </div>
            </div>

            {/* Hardware Fault Log */}
            <div>
              <h3 className="text-[10px] uppercase tracking-widest text-[#e94560]/70 mb-3 font-sans flex items-center gap-2">
                <AlertTriangle className="w-3 h-3" /> Recent Faults
              </h3>
              <div className="border border-[#e94560]/20 bg-[#e94560]/5 p-3 text-xs flex flex-col gap-2">
                <div className="flex items-start justify-between text-[#e94560]">
                  <span>CAP_VIOLATION</span>
                  <span className="text-[10px] opacity-70">12s ago</span>
                </div>
                <div className="text-white/60 text-[10px] leading-tight">
                  CR14 (CODE) Execution attempted outside bounds at 0x00000000. Fault handled by OS trap.
                </div>
              </div>
            </div>

          </div>
        </div>

        {/* Floating Editor Button */}
        <div className="absolute bottom-6 left-1/2 -translate-x-1/2 z-50">
          <Button 
            className="rounded-full bg-[#fbbf24] hover:bg-[#fbbf24]/90 text-[#0f0f23] shadow-[0_0_20px_rgba(251,191,36,0.3)] border border-[#fbbf24]/50 font-sans font-bold px-6 h-12 flex items-center gap-2 transition-all hover:scale-105"
            onClick={() => setEditorOpen(true)}
          >
            <Code2 className="w-5 h-5" />
            OPEN EDITOR
          </Button>
        </div>

        {/* Editor Modal Overlay (Fake) */}
        {editorOpen && (
          <div className="absolute inset-0 bg-[#0a0a15]/80 backdrop-blur-md z-50 flex items-center justify-center p-8">
            <div className="w-full max-w-4xl h-full max-h-[600px] bg-[#0f0f23] border border-[#fbbf24]/30 shadow-2xl flex flex-col animate-in fade-in zoom-in-95 duration-200">
              <div className="h-10 border-b border-white/10 flex items-center justify-between px-4">
                <div className="flex items-center gap-2 text-sm text-[#fbbf24]">
                  <Code2 className="w-4 h-4" />
                  <span className="font-mono">math.cloomc</span>
                </div>
                <button 
                  onClick={() => setEditorOpen(false)}
                  className="text-white/50 hover:text-white"
                >
                  <Settings className="w-4 h-4" />
                </button>
              </div>
              <div className="flex-1 p-4 font-mono text-sm text-white/70 overflow-auto">
                <div className="text-emerald-400/50 mb-4">;; Standard Math Library</div>
                <div className="mb-2"><span className="text-[#fbbf24]">define</span> fib (n) {'{'}</div>
                <div className="pl-4 mb-2"><span className="text-pink-400">if</span> {'(<= n 1)'}</div>
                <div className="pl-8 mb-2">n</div>
                <div className="pl-4 mb-2"><span className="text-pink-400">else</span></div>
                <div className="pl-8 mb-2"><span className="text-cyan-400">+</span> (fib (<span className="text-cyan-400">-</span> n 1)) (fib (<span className="text-cyan-400">-</span> n 2))</div>
                <div>{'}'}</div>
              </div>
              <div className="h-12 border-t border-white/10 flex items-center justify-end px-4 gap-3 bg-white/5">
                <Button variant="ghost" size="sm" onClick={() => setEditorOpen(false)} className="text-white/60 hover:text-white rounded-none">
                  CANCEL
                </Button>
                <Button size="sm" className="bg-[#fbbf24] hover:bg-[#fbbf24]/90 text-[#0f0f23] rounded-none font-bold">
                  COMPILE & UPLOAD
                </Button>
              </div>
            </div>
          </div>
        )}

      </div>
    </div>
  );
}

// Subcomponents

function TerminalLine({ prompt, cmd, text, type = "input", color = "emerald" }: any) {
  if (type === "output") {
    const colorClasses: Record<string, string> = {
      cyan: "text-cyan-400",
      emerald: "text-emerald-400",
      yellow: "text-yellow-400",
      red: "text-[#e94560]",
      "white/50": "text-white/50",
      "white/70": "text-white/70",
    };
    return (
      <div className={`mb-1 ${colorClasses[color] || color}`}>
        {text}
      </div>
    );
  }

  return (
    <div className="flex items-start mb-1 text-emerald-400">
      {prompt && <span className="mr-3 font-bold text-[#fbbf24] select-none">{prompt}</span>}
      <span>{cmd}</span>
    </div>
  );
}

function RegisterCard({ name, label, value, highlight = false }: any) {
  return (
    <div className={`p-3 border ${highlight ? 'border-[#fbbf24]/50 bg-[#fbbf24]/10' : 'border-white/10 bg-white/5'} flex flex-col gap-1 relative overflow-hidden group hover:border-white/30 transition-colors`}>
      {highlight && <div className="absolute top-0 right-0 w-8 h-8 bg-[#fbbf24]/20 rounded-bl-full pointer-events-none" />}
      <div className="flex items-center justify-between">
        <span className={`font-bold text-sm ${highlight ? 'text-[#fbbf24]' : 'text-white/80'}`}>{name}</span>
        <span className="text-[9px] text-white/30 uppercase tracking-wider">{label}</span>
      </div>
      <div className={`font-mono text-sm tracking-wider mt-1 ${highlight ? 'text-white' : 'text-cyan-400/80'}`}>
        {value}
      </div>
    </div>
  );
}
