import React, { useState } from "react";
import {
  ResizableHandle,
  ResizablePanel,
  ResizablePanelGroup,
} from "@/components/ui/resizable";
import { 
  Pencil, 
  Play, 
  GitBranch, 
  Cpu, 
  Network, 
  Terminal,
  Pause,
  SkipForward,
  RotateCcw,
  Activity,
  Database,
  ArrowRight,
  MonitorPlay,
  Settings2
} from "lucide-react";
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from "@/components/ui/tooltip";

export function LabBench() {
  const [isPlaying, setIsPlaying] = useState(true);
  
  return (
    <div className="flex flex-col w-full h-screen bg-[#1a1a2e] text-[#eaeaea] overflow-hidden font-mono text-sm">
      {/* Top Header */}
      <header className="h-[36px] bg-[#0a0a1a] border-b border-[#2a2a4e] flex items-center justify-between px-4 shrink-0">
        <div className="flex items-center gap-4">
          <div className="text-[#fbbf24] font-bold text-lg tracking-wider flex items-center gap-2">
            <span className="text-xl">λ</span> CHURCH MACHINE
          </div>
          <div className="h-4 w-[1px] bg-[#2a2a4e] mx-2" />
          <div className="flex items-center gap-2 text-xs text-slate-400">
            <span className="flex h-2 w-2 rounded-full bg-emerald-500 animate-pulse"></span>
            SYS_CLK: 12.5 MHz
          </div>
        </div>
        
        <div className="flex items-center gap-1">
          <button className="h-7 px-3 flex items-center gap-2 bg-[#2a2a4e] hover:bg-[#3a3a5e] rounded border border-[#3a3a5e] transition-colors text-xs text-[#eaeaea]">
            <RotateCcw className="w-3 h-3" /> Reset
          </button>
          <button 
            className="h-7 px-3 flex items-center gap-2 bg-[#1a2e1a] hover:bg-[#2a3e2a] text-emerald-400 border border-[#2a4e2a] rounded transition-colors text-xs ml-2"
            onClick={() => setIsPlaying(!isPlaying)}
          >
            {isPlaying ? <Pause className="w-3 h-3" /> : <Play className="w-3 h-3" />} 
            {isPlaying ? "Pause" : "Run"}
          </button>
          <button className="h-7 px-3 flex items-center gap-2 bg-[#2a2a4e] hover:bg-[#3a3a5e] rounded border border-[#3a3a5e] transition-colors text-xs text-[#eaeaea]">
            <SkipForward className="w-3 h-3" /> Step
          </button>
        </div>
      </header>

      <div className="flex flex-1 overflow-hidden">
        {/* Left Sidebar */}
        <aside className="w-[48px] bg-[#0a0a1a] border-r border-[#2a2a4e] flex flex-col items-center py-4 gap-4 shrink-0">
          <TooltipProvider delayDuration={0}>
            <SidebarIcon icon={<Pencil />} label="Editor" />
            <SidebarIcon icon={<MonitorPlay />} label="Simulator" active />
            <SidebarIcon icon={<GitBranch />} label="Pipeline" active />
            <SidebarIcon icon={<Cpu />} label="Registers" />
            <SidebarIcon icon={<Network />} label="Namespace" />
            <SidebarIcon icon={<Terminal />} label="REPL" />
            
            <div className="mt-auto flex flex-col gap-4">
              <SidebarIcon icon={<Database />} label="Memory Dump" />
              <SidebarIcon icon={<Settings2 />} label="Settings" />
            </div>
          </TooltipProvider>
        </aside>

        {/* Main Workspace (Split Pane) */}
        <main className="flex-1 overflow-hidden p-2">
          <ResizablePanelGroup direction="horizontal" className="rounded-md border border-[#2a2a4e]">
            {/* Left Pane: Simulator */}
            <ResizablePanel defaultSize={55} minSize={30}>
              <div className="h-full flex flex-col bg-[#111122]">
                <div className="h-8 border-b border-[#2a2a4e] bg-[#1a1a2e] flex items-center px-3 justify-between">
                  <div className="flex items-center gap-2 text-xs font-semibold text-[#fbbf24]">
                    <MonitorPlay className="w-3 h-3" /> SIMULATOR STATE
                  </div>
                  <div className="text-[10px] text-slate-400">T=4,192,834</div>
                </div>
                
                <div className="p-4 flex-1 overflow-auto">
                  <div className="grid grid-cols-2 gap-4">
                    {/* Status Panel */}
                    <div className="border border-[#2a2a4e] rounded bg-[#0a0a1a] p-3 flex flex-col gap-2">
                      <h3 className="text-xs text-slate-500 uppercase font-semibold mb-1">Execution Core</h3>
                      <div className="flex justify-between text-xs">
                        <span className="text-slate-400">PC:</span>
                        <span className="text-emerald-400">0x0000041C</span>
                      </div>
                      <div className="flex justify-between text-xs">
                        <span className="text-slate-400">Instruction:</span>
                        <span className="text-[#fbbf24]">LOAD CR0, CR2, 0x14</span>
                      </div>
                      <div className="flex justify-between text-xs">
                        <span className="text-slate-400">State:</span>
                        <span className="text-cyan-400">RUNNING</span>
                      </div>
                      <div className="flex justify-between text-xs">
                        <span className="text-slate-400">Privilege:</span>
                        <span className="text-[#e94560]">RING 0</span>
                      </div>
                    </div>

                    {/* Memory Map preview */}
                    <div className="border border-[#2a2a4e] rounded bg-[#0a0a1a] p-3 flex flex-col gap-2">
                      <h3 className="text-xs text-slate-500 uppercase font-semibold mb-1">Memory Segments</h3>
                      <div className="flex items-center gap-2 text-xs">
                        <div className="w-2 h-2 bg-[#e94560] rounded-full"></div>
                        <span className="text-slate-400 flex-1">BOOT</span>
                        <span>0x00..0xFF</span>
                      </div>
                      <div className="flex items-center gap-2 text-xs">
                        <div className="w-2 h-2 bg-[#fbbf24] rounded-full"></div>
                        <span className="text-slate-400 flex-1">CODE</span>
                        <span>0x100..0x7FF</span>
                      </div>
                      <div className="flex items-center gap-2 text-xs">
                        <div className="w-2 h-2 bg-emerald-400 rounded-full animate-pulse"></div>
                        <span className="text-slate-400 flex-1">HEAP</span>
                        <span>0x800..0xFFF</span>
                      </div>
                    </div>
                  </div>

                  {/* Active Code Segment */}
                  <div className="mt-4 border border-[#2a2a4e] rounded bg-[#0a0a1a] flex flex-col h-[300px]">
                    <h3 className="text-xs text-slate-500 uppercase font-semibold p-3 border-b border-[#2a2a4e]">Live Trace</h3>
                    <div className="flex-1 overflow-auto p-3 text-xs font-mono leading-relaxed">
                      <div className="text-slate-500">0x00000410:  NOP</div>
                      <div className="text-slate-500">0x00000414:  STORE DR1, DR2, 0x08</div>
                      <div className="text-slate-500">0x00000418:  ADD DR0, DR1, DR2</div>
                      <div className="bg-[#2a2a4e] text-[#fbbf24] px-1 py-0.5 -mx-1 flex justify-between">
                        <span>0x0000041C:  LOAD CR0, CR2, 0x14</span>
                        <span className="text-slate-400">&lt;-- PC</span>
                      </div>
                      <div className="text-slate-400">0x00000420:  CALL CR0, 0x04</div>
                      <div className="text-slate-400">0x00000424:  JMP 0x00000410</div>
                      <div className="text-slate-400">0x00000428:  NOP</div>
                      <div className="text-slate-400">0x0000042C:  NOP</div>
                    </div>
                  </div>
                </div>
              </div>
            </ResizablePanel>

            <ResizableHandle className="w-1 bg-[#0a0a1a] border-x border-[#2a2a4e] hover:bg-[#fbbf24] transition-colors" />

            {/* Right Pane: Pipeline */}
            <ResizablePanel defaultSize={45} minSize={30}>
              <div className="h-full flex flex-col bg-[#111122]">
                <div className="h-8 border-b border-[#2a2a4e] bg-[#1a1a2e] flex items-center px-3 justify-between">
                  <div className="flex items-center gap-2 text-xs font-semibold text-[#fbbf24]">
                    <GitBranch className="w-3 h-3" /> PIPELINE TRACE
                  </div>
                </div>
                
                <div className="p-4 flex-1 overflow-auto flex flex-col gap-6">
                  {/* Pipeline Visualizer */}
                  <div className="flex flex-col gap-4">
                    <PipelineStage 
                      stage="FETCH" 
                      instruction="CALL CR0, 0x04" 
                      address="0x00000420"
                      status="active"
                    />
                    <div className="w-px h-4 bg-[#2a2a4e] ml-6"></div>
                    <PipelineStage 
                      stage="DECODE" 
                      instruction="LOAD CR0, CR2, 0x14" 
                      address="0x0000041C"
                      status="active"
                    />
                    <div className="w-px h-4 bg-[#2a2a4e] ml-6"></div>
                    <PipelineStage 
                      stage="EXECUTE" 
                      instruction="ADD DR0, DR1, DR2" 
                      address="0x00000418"
                      status="stall"
                      detail="Stall: ALU dependency"
                    />
                    <div className="w-px h-4 bg-[#e94560] ml-6"></div>
                    <PipelineStage 
                      stage="WRITEBACK" 
                      instruction="STORE DR1, DR2, 0x08" 
                      address="0x00000414"
                      status="done"
                    />
                  </div>

                  {/* Forwarding Network */}
                  <div className="border border-[#2a2a4e] rounded bg-[#0a0a1a] p-3 mt-4">
                    <h3 className="text-xs text-slate-500 uppercase font-semibold mb-3">Forwarding Network</h3>
                    <div className="space-y-2 text-xs">
                      <div className="flex items-center justify-between p-2 bg-[#1a1a2e] rounded border border-[#2a2a4e]">
                        <span className="text-slate-400">EX → ID</span>
                        <ArrowRight className="w-3 h-3 text-emerald-400" />
                        <span className="text-emerald-400">DR1 (Val: 0x0A)</span>
                      </div>
                      <div className="flex items-center justify-between p-2 bg-[#1a1a2e] rounded border border-[#2a2a4e] opacity-50">
                        <span className="text-slate-400">MEM → EX</span>
                        <ArrowRight className="w-3 h-3 text-slate-600" />
                        <span className="text-slate-500">None</span>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </ResizablePanel>
          </ResizablePanelGroup>
        </main>
      </div>
    </div>
  );
}

function SidebarIcon({ icon, label, active = false }: { icon: React.ReactNode, label: string, active?: boolean }) {
  return (
    <Tooltip>
      <TooltipTrigger asChild>
        <button 
          className={`w-10 h-10 flex items-center justify-center rounded-lg transition-colors ${
            active 
              ? 'bg-[#fbbf24]/10 text-[#fbbf24] border border-[#fbbf24]/30' 
              : 'text-slate-400 hover:text-[#eaeaea] hover:bg-[#2a2a4e]'
          }`}
        >
          {React.cloneElement(icon as React.ReactElement, { className: "w-5 h-5" })}
        </button>
      </TooltipTrigger>
      <TooltipContent side="right" className="bg-[#0a0a1a] text-[#eaeaea] border-[#2a2a4e]">
        {label}
      </TooltipContent>
    </Tooltip>
  );
}

function PipelineStage({ 
  stage, 
  instruction, 
  address, 
  status,
  detail
}: { 
  stage: string, 
  instruction: string, 
  address: string, 
  status: 'active' | 'stall' | 'done',
  detail?: string
}) {
  const statusColors = {
    active: "border-[#fbbf24] bg-[#fbbf24]/10",
    stall: "border-[#e94560] bg-[#e94560]/10",
    done: "border-[#2a2a4e] bg-[#1a1a2e]"
  };

  const statusTextColors = {
    active: "text-[#fbbf24]",
    stall: "text-[#e94560]",
    done: "text-slate-500"
  };

  return (
    <div className={`border rounded-md p-3 flex flex-col gap-2 ${statusColors[status]}`}>
      <div className="flex items-center justify-between">
        <span className={`text-xs font-bold ${statusTextColors[status]}`}>{stage}</span>
        <span className="text-[10px] text-slate-400 font-mono">{address}</span>
      </div>
      <div className="font-mono text-xs text-[#eaeaea]">{instruction}</div>
      {detail && (
        <div className="text-[10px] text-[#e94560] font-mono mt-1">{detail}</div>
      )}
    </div>
  );
}
