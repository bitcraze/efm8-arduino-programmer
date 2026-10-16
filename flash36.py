#! python3.6
import serial, sys, struct, time


if len(sys.argv) < 3:
    print ("usage: %s <port> <firmware.hex> <ESC_idx>" % sys.argv[0])
    sys.exit(1)

class PI():
    def __init__(self, com):
        self.ser = serial.Serial(com, 1000000, timeout = 1)
        print(f"Serial opened: {com}, baud={self.ser.baudrate}, timeout={self.ser.timeout}s", flush=True)
        time.sleep(2)

    def expect_ack(self, expected, operation):
        """Read a binary acknowledgement, allowing #DBG lines from the Uno."""
        started = time.monotonic()
        deadline = started + 5
        print(f"[TX] {operation}: waiting for 0x{expected:02x}", flush=True)
        while time.monotonic() < deadline:
            response = self.ser.read(1)
            if response == b'#':
                line = self.ser.readline()
                print("[Arduino] #" + line.decode('ascii', errors='replace').rstrip(), flush=True)
                continue
            if not response:
                continue
            elapsed = time.monotonic() - started
            print(f"[RX] {operation}: {response.hex(' ')} after {elapsed:.3f}s", flush=True)
            if response == b'\x44':
                raise RuntimeError("Target erase failed; flashing stopped. See Arduino C2 status above.")
            if response != bytes([expected]):
                raise RuntimeError(f"{operation}: expected 0x{expected:02x}, received {response!r}")
            return
        raise TimeoutError(
            f"{operation}: no acknowledgement 0x{expected:02x} within 5s; "
            f"serial bytes pending={self.ser.in_waiting}. "
            "Check the last Arduino C2 stage above (if the debug sketch is installed)."
        )


    def conf(self):

        # init Programming Interface (PI)
        while True:
            try:
                self.ser.write(b'\x01\x00')
                self.expect_ack(0x81, "PI init (01 00)")
                break
            except Exception as exc:
                print (f"error com: {exc}")
                sys.exit(1)
        time.sleep(0.5)
        print ("PI initiated")

    def check_written(self, buf, buf_size, address):
        # Four payload bytes: count, address[23:16], address[15:8], address[7:0].
        self.ser.write(bytes([0x05, 0x04, buf_size]) + address.to_bytes(3, 'big'))
        self.expect_ack(0x85, f"Readback at 0x{address:04x}, {buf_size} bytes")
        dump = self.ser.read(buf_size)
        if dump != bytes.fromhex(buf):
            raise RuntimeError(f"Readback mismatch at 0x{address:04x}: expected {buf}, got {dump.hex()} ({len(dump)}/{buf_size} bytes)")

    def reset_device(self):
        for attempt in range(1, 6):
            try:
                self.ser.write(b'\x02\x00')
                self.expect_ack(0x82, f"Device reset (02 00), attempt {attempt}/5")
                print("Device reset")
                return
            except (TimeoutError, RuntimeError) as exc:
                print(f"Reset attempt {attempt}/5 failed: {exc}", flush=True)
                if attempt == 5:
                    raise RuntimeError("Device reset failed after 5 attempts") from exc
                time.sleep(0.1)

    def switch_esc(self, index):
        cmd = int.to_bytes((index+7), length=1, byteorder='big')
        self.ser.write(cmd+b'\x00')
        self.expect_ack(cmd[0] + 0x80, f"Select ESC {index} ({cmd.hex()} 00)")

    def _isNextAddressAdjacent(self, thisLine, nextLine):
        assert thisLine[7:9] == '00' and nextLine[7:9] == '00', "Lines must be data records"
        thisStartAddr = int(thisLine[3:7], 16)
        nextStartAddr = int(nextLine[3:7], 16)
        thisByteCount = int(thisLine[1:3], 16)
        thisEndAddr = thisStartAddr + thisByteCount
        return thisEndAddr == nextStartAddr

    def prog(self, firmware, esc_index = 1):

        print ("Connected")

        #f = open(firmware,'r').readlines()
        f = firmware.splitlines()
        self.switch_esc(esc_index)
        self.conf()

        
        print(f'Switched to ESC {esc_index}')
        # erase device
        self.ser.write(b'\x04\x00')
        self.expect_ack(0x84, "Device erase (04 00)")

        print ("Device erased")
		
        # write hex file
        total = 0
        buf = ''
        buf_size = 0
        for i in f[:-1]:  # skip last line
            assert(i[0] == ':')            
            size = int(i[1:3],16)
            assert(size + 4 < 256)       
            if buf_size == 0:
                addrh = int(i[3:5],16)
                addrl = int(i[5:7],16)
            assert(i[7:9] == '00')
            data = i[9:9 + size*2]
            assert(len(data) == size*2)   
            buf += data
            buf_size += size

            if buf_size >= 256-0x20 or i == f[-2] or (not self._isNextAddressAdjacent(i, f[f.index(i)+1])): #The hex file is not always in address order. We need to check this before concatenating the lines
                while True:
                    try:
                        crc = addrh + addrl
                        for i in range(0,len(buf),2):
                            val=buf[i]+buf[i+1]
                            crc+=int(val,16)
                        assert(len(buf)/2 == buf_size)
                        self.ser.write([0x3, buf_size + 4 + 1, buf_size, 0, addrh, addrl, crc & 0xff])
                        for i in range(0,len(buf),2):
                            val=buf[i]+buf[i+1]
                            string = bytes.fromhex(val)
                            self.ser.write(string)
                        self.expect_ack(0x83, f"Flash write at 0x{addrh:02x}{addrl:02x}, {buf_size} bytes")
                        self.check_written(buf,buf_size, (addrh << 8) | addrl)
                        break
                    except Exception as e:
                        # Late replies could be mistaken for ACKs if we retry here.
                        raise RuntimeError(f"Flashing stopped at 0x{addrh:02x}{addrl:02x}: {e}") from e
                total += buf_size
                buf_size = 0
                buf = ''
                print ("Wrote %d bytes" % total)

        self.reset_device()

print ("Once")
port=sys.argv[1]
firmware=open(sys.argv[2], 'r').read()
esc_idx= 1
if len(sys.argv) > 3:
    esc_idx = int(sys.argv[3])
programmers = PI(port)

programmers.prog(firmware,esc_idx)
