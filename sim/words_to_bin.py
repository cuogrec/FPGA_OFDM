# words_out.txt (RTL) -> rtl_dma.bin (dang dump DDR little-endian, nhu dma_bits)
import struct
w = [int(l.split()[0], 16) for l in open('words_out.txt')]
open('rtl_dma.bin', 'wb').write(b''.join(struct.pack('<I', x) for x in w))
print(len(w), "words ->", len(w)*4, "bytes")
