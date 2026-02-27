import struct
import sys

def bin_to_uf2(bin_path, uf2_path):
    data = open(bin_path, 'rb').read()
    out = open(uf2_path, 'wb')
    bs = 256
    nb = (len(data) + bs - 1) // bs
    for i in range(nb):
        chunk = data[i * bs:(i + 1) * bs]
        chunk += b'\x00' * (bs - len(chunk))
        hdr = struct.pack('<IIIIIIII',
            0x0A324655, 0x9E5D5157, 0x00002000,
            i * bs, bs, i, nb, 0x00)
        out.write(hdr + chunk + b'\x00' * (512 - 32 - bs - 4) + struct.pack('<I', 0x0AB16F30))
    out.close()
    print(f'UF2: {uf2_path} ({nb} blocks)')

if __name__ == '__main__':
    bin_to_uf2(sys.argv[1], sys.argv[2])
