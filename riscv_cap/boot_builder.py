#!/usr/bin/env python3
"""
Boot Image Builder for RV32-Cap (Sim-32) Simulator

Constructs a binary memory image containing:
- Namespace table at base address
- Thread object (data registers + packed PC)
- C-List with Golden Tokens
- Initial code segment

Output: JSON file with structured boot image data
"""
import json
import struct
import sys


class BootImageBuilder:
    def __init__(self, mem_size=65536):
        self.memory = bytearray(mem_size)
        self.namespace_table = []
        self.clist = []
        self.thread_objects = []

    def compute_seal(self, location, limit):
        """Compute 25-bit MAC seal matching simulator.js computeSeal()"""
        h = 0x5A5A5A5A
        h = ((h ^ location) * 0x01000193) & 0xFFFFFFFF
        h = ((h ^ limit) * 0x01000193) & 0xFFFFFFFF
        h = (h ^ (h >> 16)) & 0xFFFFFFFF
        return h & 0x01FFFFFF

    def make_version_seals(self, version, location, limit):
        """Create versionSeals word: version(7) | seal(25)"""
        seal = self.compute_seal(location, limit)
        return (((version & 0x7F) << 25) | (seal & 0x01FFFFFF)) & 0xFFFFFFFF

    def get_perm_bits(self, perms):
        """Convert permission dict to 6-bit field matching simulator.js getPermBits()"""
        bits = 0
        perm_order = ['R', 'W', 'X', 'L', 'S', 'E']
        for i, name in enumerate(perm_order):
            if perms.get(name, 0):
                bits |= (1 << i)
        return bits & 0x3F

    def create_gt(self, version, index, perms, gt_type):
        """Create 32-bit Golden Token matching simulator.js createGT()"""
        v = ((version & 0x7F) << 25) & 0xFFFFFFFF
        i = ((index & 0x1FFFF) << 8) & 0xFFFFFFFF
        p = (self.get_perm_bits(perms) << 2) & 0xFFFFFFFF
        t = gt_type & 0x3
        return (v | i | p | t) & 0xFFFFFFFF

    def parse_gt(self, gt32):
        """Parse a 32-bit Golden Token matching simulator.js parseGT()"""
        gt32 = gt32 & 0xFFFFFFFF
        version = (gt32 >> 25) & 0x7F
        index = (gt32 >> 8) & 0x1FFFF
        perm_bits = (gt32 >> 2) & 0x3F
        gt_type = gt32 & 0x3
        type_names = ['Inform', 'Outform', 'NULL', 'Abstract']
        return {
            'version': version,
            'index': index,
            'permissions': {
                'R': (perm_bits >> 0) & 1,
                'W': (perm_bits >> 1) & 1,
                'X': (perm_bits >> 2) & 1,
                'L': (perm_bits >> 3) & 1,
                'S': (perm_bits >> 4) & 1,
                'E': (perm_bits >> 5) & 1,
            },
            'type': gt_type,
            'typeName': type_names[gt_type & 0x3],
        }

    def validate_mac(self, entry):
        """Validate MAC seal on a namespace entry"""
        if not entry:
            return False
        stored_seal = entry['versionSeals'] & 0x01FFFFFF
        computed_seal = self.compute_seal(entry['location'], entry['limit'])
        return stored_seal == computed_seal

    def add_namespace_entry(self, location, limit, version=0):
        """Add a namespace entry"""
        idx = len(self.namespace_table)
        entry = {
            'index': idx,
            'location': location,
            'limit': limit,
            'versionSeals': self.make_version_seals(version, location, limit),
            'gBit': 0,
        }
        self.namespace_table.append(entry)
        return idx

    def add_thread_object(self, base_addr, data_regs=None, pc=0, flags=None):
        """Create thread object at base_addr in memory
        Layout: x0-x31 (32 words) + packed PC (1 word) = 33 words"""
        if data_regs is None:
            data_regs = [0] * 32
        if flags is None:
            flags = {'N': 0, 'Z': 0, 'C': 0, 'V': 0}

        obj = {
            'base_addr': base_addr,
            'data_regs': data_regs[:32],
            'pc': pc,
            'flags': flags,
        }
        self.thread_objects.append(obj)

        for i, val in enumerate(data_regs[:32]):
            addr = base_addr + i * 4
            struct.pack_into('<I', self.memory, addr, val & 0xFFFFFFFF)

        addr = base_addr + 32 * 4
        struct.pack_into('<I', self.memory, addr, pc & 0xFFFFFFFF)

        return obj

    def add_clist_entry(self, base_addr, slot, gt):
        """Write a Golden Token to C-List at base_addr + slot*4"""
        addr = base_addr + slot * 4
        struct.pack_into('<I', self.memory, addr, gt & 0xFFFFFFFF)
        self.clist.append({'addr': addr, 'slot': slot, 'gt': gt})
        return addr

    def write_code(self, base_addr, instructions):
        """Write a list of 32-bit instructions to memory"""
        for i, insn in enumerate(instructions):
            addr = base_addr + i * 4
            struct.pack_into('<I', self.memory, addr, insn & 0xFFFFFFFF)

    def build_default_image(self):
        """Build the default boot image matching simulator._bootSequence()"""
        self.add_namespace_entry(0x00000000, 0x0000FFFF)
        self.add_namespace_entry(0x00000000, 0x00003FFF)
        self.add_namespace_entry(0x00004000, 0x00007FFF)
        self.add_namespace_entry(0x00008000, 0x000000FF)

        self.add_thread_object(0x8000)

        boot_crs = {
            15: self.create_gt(0, 0, {}, 3),
            8: self.create_gt(0, 3, {}, 3),
            7: self.create_gt(0, 0, {'E': 1}, 3),
            6: self.create_gt(0, 1, {'L': 1, 'S': 1}, 3),
        }

        return {
            'namespace_table': self.namespace_table,
            'thread_objects': self.thread_objects,
            'clist': self.clist,
            'boot_crs': {str(k): v for k, v in boot_crs.items()},
            'memory_hex': self.memory.hex(),
            'memory_size': len(self.memory),
        }

    def export_json(self, filename):
        """Export boot image as JSON"""
        image = self.build_default_image()
        with open(filename, 'w') as f:
            json.dump(image, f, indent=2)
        return image

    def export_binary(self, filename):
        """Export raw memory as binary"""
        self.build_default_image()
        with open(filename, 'wb') as f:
            f.write(self.memory)


if __name__ == '__main__':
    builder = BootImageBuilder()
    outfile = sys.argv[1] if len(sys.argv) > 1 else 'boot_image.json'
    image = builder.export_json(outfile)
    print(f"Boot image exported to {outfile}")
    print(f"  Namespace entries: {len(image['namespace_table'])}")
    print(f"  Thread objects: {len(image['thread_objects'])}")
    print(f"  C-List entries: {len(image['clist'])}")
    print(f"  Memory size: {image['memory_size']} bytes")
    for cr_id, gt_val in image['boot_crs'].items():
        parsed = builder.parse_gt(gt_val)
        active_perms = [k for k, v in parsed['permissions'].items() if v]
        print(f"  CR{cr_id}: GT=0x{gt_val:08X} ver={parsed['version']} idx={parsed['index']} "
              f"perms={','.join(active_perms)} type={parsed['typeName']}")
