import React, { useState } from "react";
import { Search, Terminal, Activity, Database, LayoutTemplate, Map, FileCode2, History, Zap, Settings, Command } from "lucide-react";
import { cn } from "@/lib/utils";

export function CommandPalette() {
  const [isOpen, setIsOpen] = useState(true); // Default to open for mockup purposes

  return (
    <div className="w-full h-screen bg-[#0f0f23] text-[#eaeaea] font-mono flex flex-col relative overflow-hidden selection:bg-[#fbbf24]/30">
      {/* Top Status Bar */}
      <header className="h-8 shrink-0 bg-[#1a1a2e] border-b border-[#2a2a40] flex items-center px-4 text-xs tracking-wide shadow-sm z-10">
        <div className="flex items-center gap-6 w-full">
          <div className="flex items-center gap-2 font-bold text-[#fbbf24]">
            <span className="text-lg leading-none">λ</span>
            <span>CHURCH MACHINE</span>
          </div>
          
          <div className="flex-1 flex items-center justify-end gap-6 text-gray-400">
            <span className="flex items-center gap-2">
              <span className="w-2 h-2 rounded-full bg-emerald-500 animate-pulse"></span>
              RUNNING
            </span>
            <span>PC: <span className="text-[#fbbf24]">0x00400000</span></span>
            <span>CYCLE: 14,920,111</span>
            <span>FAULTS: <span className="text-emerald-400">0</span></span>
          </div>
        </div>
      </header>

      {/* Editor Content (Background View) */}
      <main className="flex-1 overflow-auto relative flex">
        {/* Line Numbers */}
        <div className="w-12 shrink-0 border-r border-[#2a2a40] bg-[#0f0f23] text-gray-600 text-right py-4 pr-3 select-none flex flex-col gap-1 text-sm">
          {Array.from({ length: 30 }).map((_, i) => (
            <div key={i}>{i + 1}</div>
          ))}
        </div>
        
        {/* Code Content */}
        <div className="flex-1 p-4 overflow-auto text-sm leading-relaxed flex flex-col gap-1">
          <div className="text-gray-500 italic">// Initialize main system namespace</div>
          <div><span className="text-pink-500">const</span> sys <span className="text-pink-500">=</span> require(<span className="text-green-400">'system'</span>);</div>
          <div><span className="text-pink-500">const</span> mem <span className="text-pink-500">=</span> require(<span className="text-green-400">'memory'</span>);</div>
          <br/>
          <div className="text-gray-500 italic">// Allocate data registers</div>
          <div><span className="text-pink-500">let</span> r0 <span className="text-pink-500">=</span> sys.alloc(<span className="text-orange-400">0x1000</span>);</div>
          <div><span className="text-pink-500">let</span> r1 <span className="text-pink-500">=</span> sys.alloc(<span className="text-orange-400">0x2000</span>);</div>
          <br/>
          <div className="text-gray-500 italic">// Main computation loop</div>
          <div><span className="text-[#fbbf24]">function</span> <span className="text-blue-400">processTicks</span>(limit) {'{'}</div>
          <div className="pl-4"><span className="text-pink-500">for</span> (<span className="text-pink-500">let</span> i <span className="text-pink-500">=</span> <span className="text-orange-400">0</span>; i <span className="text-pink-500">&lt;</span> limit; i<span className="text-pink-500">++</span>) {'{'}</div>
          <div className="pl-8">mem.write(r0, i);</div>
          <div className="pl-8">sys.tick();</div>
          <div className="pl-4">{'}'}</div>
          <div>{'}'}</div>
          <br/>
          <div><span className="text-blue-400">processTicks</span>(<span className="text-orange-400">1000000</span>);</div>
        </div>
      </main>

      {/* Keyboard Hint Pill */}
      <div 
        className="absolute bottom-6 right-6 flex items-center gap-2 bg-[#1a1a2e] border border-[#2a2a40] px-3 py-1.5 rounded-md text-sm text-gray-400 shadow-lg cursor-pointer hover:text-white transition-colors"
        onClick={() => setIsOpen(!isOpen)}
      >
        <span>Command Palette</span>
        <div className="flex items-center gap-1">
          <kbd className="bg-[#2a2a40] px-1.5 py-0.5 rounded text-xs text-gray-300">⌘</kbd>
          <kbd className="bg-[#2a2a40] px-1.5 py-0.5 rounded text-xs text-gray-300">K</kbd>
        </div>
      </div>

      {/* Command Palette Overlay */}
      {isOpen && (
        <div className="absolute inset-0 bg-[#0f0f23]/80 backdrop-blur-sm z-50 flex items-start justify-center pt-[15vh]">
          <div className="w-[640px] max-w-[90vw] bg-[#1a1a2e] border border-[#3a3a50] rounded-xl shadow-2xl overflow-hidden flex flex-col shadow-black/50">
            {/* Input */}
            <div className="flex items-center px-4 border-b border-[#2a2a40] h-14 shrink-0 bg-[#1a1a2e]">
              <Search className="w-5 h-5 text-gray-400 shrink-0" />
              <input 
                type="text" 
                placeholder="Search commands, views, or queries..." 
                className="flex-1 bg-transparent border-none outline-none px-4 text-[#eaeaea] placeholder:text-gray-500 text-lg font-sans"
                autoFocus
              />
              <kbd className="hidden sm:inline-flex bg-[#2a2a40] px-2 py-1 rounded text-xs text-gray-400 shrink-0 border border-[#3a3a50]">ESC</kbd>
            </div>

            {/* Results */}
            <div className="flex-1 overflow-y-auto max-h-[60vh] p-2 flex flex-col gap-4 custom-scrollbar font-sans">
              
              <div className="flex flex-col gap-1">
                <div className="px-3 py-1.5 text-xs font-semibold text-gray-500 uppercase tracking-wider font-mono">Views</div>
                
                <CommandItem icon={<Terminal className="w-4 h-4" />} title="Create Editor" shortcut="G E" selected />
                <CommandItem icon={<Activity className="w-4 h-4" />} title="Simulator" shortcut="G S" />
                <CommandItem icon={<LayoutTemplate className="w-4 h-4" />} title="Pipeline Visualizer" />
                <CommandItem icon={<Map className="w-4 h-4" />} title="Namespace Diagram" />
                <CommandItem icon={<Database className="w-4 h-4" />} title="Memory Dump" />
              </div>

              <div className="flex flex-col gap-1">
                <div className="px-3 py-1.5 text-xs font-semibold text-gray-500 uppercase tracking-wider font-mono">Queries</div>
                
                <CommandItem icon={<Zap className="w-4 h-4" />} title="Dump Registers" subtitle="Outputs R0-R31 to console" />
                <CommandItem icon={<FileCode2 className="w-4 h-4" />} title="Show Program Counter (PC)" />
                <CommandItem icon={<Settings className="w-4 h-4" />} title="Toggle Hardware Trace" />
              </div>

              <div className="flex flex-col gap-1">
                <div className="px-3 py-1.5 text-xs font-semibold text-gray-500 uppercase tracking-wider font-mono">Recent</div>
                
                <CommandItem icon={<History className="w-4 h-4" />} title="run tests/core_ops.S" />
                <CommandItem icon={<History className="w-4 h-4" />} title="make build-fpga" />
              </div>
              
            </div>
            
            {/* Footer */}
            <div className="border-t border-[#2a2a40] h-10 shrink-0 bg-[#12121e] flex items-center justify-between px-4 text-xs text-gray-500">
              <div className="flex items-center gap-4">
                <span className="flex items-center gap-1"><kbd className="bg-[#2a2a40] px-1 rounded border border-[#3a3a50]">↑</kbd><kbd className="bg-[#2a2a40] px-1 rounded border border-[#3a3a50]">↓</kbd> to navigate</span>
                <span className="flex items-center gap-1"><kbd className="bg-[#2a2a40] px-1 rounded border border-[#3a3a50]">↵</kbd> to select</span>
              </div>
              <div>Church Machine IDE v2.4.0</div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

function CommandItem({ 
  icon, 
  title, 
  subtitle, 
  shortcut, 
  selected = false 
}: { 
  icon: React.ReactNode; 
  title: string; 
  subtitle?: string; 
  shortcut?: string;
  selected?: boolean;
}) {
  return (
    <div className={cn(
      "flex items-center px-3 py-2.5 rounded-lg cursor-pointer group transition-colors",
      selected ? "bg-[#fbbf24]/10 text-[#fbbf24]" : "hover:bg-[#2a2a40] text-gray-300"
    )}>
      <div className={cn(
        "w-8 h-8 rounded flex items-center justify-center shrink-0 mr-3 transition-colors",
        selected ? "bg-[#fbbf24]/20 text-[#fbbf24]" : "bg-[#2a2a40] text-gray-400 group-hover:bg-[#3a3a50]"
      )}>
        {icon}
      </div>
      <div className="flex-1 flex flex-col">
        <div className="font-medium text-sm">{title}</div>
        {subtitle && <div className="text-xs opacity-60 mt-0.5">{subtitle}</div>}
      </div>
      {shortcut && (
        <div className="flex items-center gap-1 shrink-0 ml-4 opacity-60">
          {shortcut.split(' ').map((key, i) => (
            <kbd key={i} className="bg-[#2a2a40] px-1.5 py-0.5 rounded text-xs border border-[#3a3a50]">{key}</kbd>
          ))}
        </div>
      )}
    </div>
  );
}
