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

def command(cdb, want):
  for attempt in range(3):
      phase = select_and_send(cdb)
      print(hex(cdb[0]), 'attempt', attempt, 'phase after CDB', phase, 'intr', hex(rb(R(INTR))))
      if phase == want:
          return

      wb(R(CMD), 0x11)                 # initiator command complete sequence
      rb(R(FIFO)); rb(R(FIFO)); rb(R(INTR))
      wb(R(CMD), 0x12)                 # message accepted
      rb(R(INTR))


def finish():
    wb(R(CMD), 0x11); rb(R(FIFO)); rb(R(FIFO)); rb(R(INTR))
    wb(R(CMD), 0x12); rb(R(INTR))

ww = lambda a, v: q('writew 0x%x 0x%x' % (a, v))
total = blocks * 512
chunk = 0xC000
new = os.urandom(total)

# WRITE(10) 300 blocks at LBA 0 through blind 16-bit PDMA writes
command([0x2a, 0, 0, 0, 0, 0, 0, blocks >> 8, blocks & 0xff, 0], 0)
pos = 0
while pos < total:
    n = min(chunk, total - pos)
    settc(n)
    wb(R(CMD), 0x90)
    for k in range(pos, pos + n, 2):
        ww(PDMA, (new[k] << 8) | new[k + 1])
    pos += n
    rb(R(INTR))
print('after write phase', rb(R(BUSID)) & 7)
finish()

# read back
command(cdb, 1)
data = bytearray()
while len(data) < total:
    n = min(chunk, total - len(data))
    settc(n)
    wb(R(CMD), 0x90)
    for _ in range(n // 2):
        v = rw(PDMA)
        data += bytes([v >> 8, v & 0xff])
    rb(R(INTR))
bad = [i for i in range(total) if data[i] != new[i]]
print('write/read back bytes', len(data), 'mismatches', len(bad), 'first bad', bad[0] if bad else None)
c.close(); p.kill()
sys.exit(1 if bad else 0)
