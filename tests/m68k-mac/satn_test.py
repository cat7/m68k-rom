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


def finish():
    wb(R(CMD), 0x11); rb(R(FIFO)); rb(R(FIFO)); rb(R(INTR))
    wb(R(CMD), 0x12); rb(R(INTR))

def satn(cdb):
    wb(R(BUSID), 0); wb(R(CMD), 0x01); settc(1)
    wb(R(CMD), 0xc2)                 # select with ATN, DMA
    seq = rb(R(SEQ)) & 7
    phase1 = rb(R(BUSID)) & 7
    wb(R(CFG1), 0xc7)
    wb(R(FIFO), 0x80)                # IDENTIFY message through FIFO register
    phase2 = rb(R(BUSID)) & 7
    for b in cdb[:-1]:
        wb(R(FIFO), b)
    wb(PDMA, cdb[-1])
    phase3 = rb(R(BUSID)) & 7
    print('seq', seq, 'phases', phase1, phase2, phase3, 'intr', hex(rb(R(INTR))))
    return phase3

cdb = [0x28, 0, 0, 0, 0, 0, 0, 0, 4, 0]   # READ(10) 4 blocks
for attempt in range(3):
    if satn(cdb) == 1:
        break
    finish()
settc(2048); wb(R(CMD), 0x90)
data = bytearray()
for _ in range(1024):
    v = rw(PDMA); data += bytes([v >> 8, v & 0xff])
ok = data == open(img, 'rb').read(2048)
print('SelAtn read ok', ok)
c.close(); p.kill()
sys.exit(0 if ok else 1)
