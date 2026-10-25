"""Drive a QEMU m68k Mac through the HMP monitor (TCP)."""
import socket, sys, time, re, subprocess

class Mon:
    def __init__(self, port):
        self.s = socket.create_connection(('127.0.0.1', port))
        self.s.settimeout(5)
        self._read_prompt()

    def _read_prompt(self):
        buf = b''
        while not buf.endswith(b'(qemu) '):
            buf += self.s.recv(65536)
        return buf.decode('latin1')

    def cmd(self, c):
        self.s.sendall((c + '\n').encode())
        out = self._read_prompt()
        out = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', out)
        return out

    def words(self, addr, n):
        out = self.cmd('xp/%dhx 0x%x' % (n, addr))
        return [int(v, 16) for v in re.findall(r'0x([0-9a-f]{4})\b', out.split(':', 1)[1])][:n]

    def mouse(self):
        v, h = self.words(0x830, 2)  # Mouse low memory global: v, h
        return h, v

    def move_to(self, x, y, tries=60):
        for _ in range(tries):
            cx, cy = self.mouse()
            dx, dy = x - cx, y - cy
            if abs(dx) <= 1 and abs(dy) <= 1:
                return True
            def step(d):
                if d == 0:
                    return 0
                s = int(d / 3)
                if s == 0:
                    s = 1 if d > 0 else -1
                return max(-15, min(15, s))
            self.cmd('mouse_move %d %d' % (step(dx), step(dy)))
            time.sleep(0.6)
        return False

    def click(self, double=False):
        for _ in range(2 if double else 1):
            self.cmd('mouse_button 1'); time.sleep(0.12)
            self.cmd('mouse_button 0'); time.sleep(0.12)

    def drag_to(self, x, y):
        self.cmd('mouse_button 1'); time.sleep(0.2)
        self.move_to(x, y)
        time.sleep(0.2); self.cmd('mouse_button 0')

    def shot(self, path):
        self.cmd('screendump %s.ppm' % path)
        subprocess.run(['sips', '-s', 'format', 'png', path + '.ppm', '--out', path + '.png'],
                       capture_output=True)

    def key(self, k):
        self.cmd('sendkey ' + k)
