"""qtest: read 300 blocks via the Quadra 700 pseudo-DMA window, compare."""
import socket, subprocess, sys, os, time

qemu, img, sock = sys.argv[1], sys.argv[2], sys.argv[3]
ESP = 0x50f0f000
PDMA = 0x50f0f100
R = lambda n: ESP + n * 16
TCLO, TCMID, FIFO, CMD, BUSID, INTR, SEQ, CFG1, TCHI = 0, 1, 2, 3, 4, 5, 6, 8, 14

if os.path.exists(sock):
    os.unlink(sock)
srv = socket.socket(socket.AF_UNIX); srv.bind(sock); srv.listen(1)
p = subprocess.Popen([qemu, '-M', 'q700', '-accel', 'qtest', '-display', 'none',
                      '-qtest', 'unix:' + sock, '-device', 'scsi-hd,scsi-id=0,drive=d0',
                      '-drive', 'file=%s,format=raw,if=none,id=d0,snapshot=on' % img],
                     stderr=subprocess.DEVNULL)
c, _ = srv.accept(); f = c.makefile('rw')

def q(cmd):
    f.write(cmd + '\n'); f.flush()
    while True:
        line = f.readline()
        if line.startswith('IRQ'):
            continue
        assert line.startswith('OK'), (cmd, line)
        return line.split()[1:] if len(line.split()) > 1 else []

wb = lambda a, v: q('writeb 0x%x 0x%x' % (a, v))
rb = lambda a: int(q('readb 0x%x' % a)[0], 16)
rw = lambda a: int(q('readw 0x%x' % a)[0], 16)

def settc(n):
    wb(R(TCLO), n & 0xff); wb(R(TCMID), (n >> 8) & 0xff); wb(R(TCHI), (n >> 16) & 0xff)

blocks = 300
cdb = [0x28, 0, 0, 0, 0, 0, 0, blocks >> 8, blocks & 0xff, 0]   # READ(10) LBA 0
wb(R(CMD), 0x02)                     # chip reset

def select_and_send(cdb):
    wb(R(BUSID), 0)                  # target 0
    wb(R(CMD), 0x01)                 # flush FIFO
    settc(1)
    wb(R(CMD), 0xc1)                 # select without ATN, DMA
    wb(R(CFG1), 0xc7)
    for b in cdb[:-1]:
        wb(R(FIFO), b)               # CDB bytes through the FIFO register
    wb(PDMA, cdb[-1])                # last byte through pseudo DMA
    return rb(R(BUSID)) & 7

for attempt in range(3):
    phase = select_and_send(cdb)
    print('attempt', attempt, 'phase after CDB', phase, 'intr', hex(rb(R(INTR))))
    if phase == 1:                   # DATA IN
        break
    wb(R(CMD), 0x11)                 # initiator command complete sequence
    rb(R(FIFO)); rb(R(FIFO)); rb(R(INTR))
    wb(R(CMD), 0x12)                 # message accepted
    rb(R(INTR))

data = bytearray()
total = blocks * 512
chunk = 0xC000
while len(data) < total:
    n = min(chunk, total - len(data))
    settc(n)
    wb(R(CMD), 0x90)                 # transfer information, DMA
    for _ in range(n // 2):          # blind 16-bit reads, like the ROM
        v = rw(PDMA)
        data += bytes([v >> 8, v & 0xff])
    rb(R(INTR))

ref = open(img, 'rb').read(total)
bad = [i for i in range(total) if data[i] != ref[i]]
print('bytes', len(data), 'mismatches', len(bad), 'first bad', bad[0] if bad else None)
c.close(); p.kill()
sys.exit(1 if bad else 0)
