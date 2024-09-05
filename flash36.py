#! python3.6
import serial, sys, struct, time


if len(sys.argv) < 3:
    print ("usage: %s <port> <firmware.hex> <ESC_idx>" % sys.argv[0])
    sys.exit(1)

class PI():
    def __init__(self, com):
        self.ser = serial.Serial(com, 1000000, timeout = 1)
        time.sleep(2)


    def conf(self):

        # init Programming Interface (PI)
        while True:
            try:
                self.ser.write(b'\x01\x00')
                x=self.ser.read(1)
                assert x == b'\x81'
                break
            except:
                print ("error com")			
                sys.exit(1)

        print ("PI initiated")

    def check_written(self, buf, buf_size):
        self.ser.write(b'\x05\x00')
        if(self.ser.read(1)==b'\x85'):
            dump = self.ser.read(buf_size)
            assert(dump==bytes.fromhex(buf))

    def switch_esc(self, index):
        cmd = int.to_bytes(index+7)
        self.ser.write(cmd+b'\x00')
        r = self.ser.read(1)
        assert r == (cmd[0] + b'\x80'[0]).to_bytes(1,'big')

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
        assert self.ser.read(1)==b'\x84'

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

            attempts = 0
            if buf_size >= 256-0x20 or i == f[-2] or (not self._isNextAddressAdjacent(i, f[f.index(i)+1])): #The hex file is not always in address order. We need to check this before concatenating the lines
                while True:
                    try:
                        print (hex(addrh), hex(addrl), buf)
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
                        ret =self.ser.read(1)
                        if ret == b'\x83':
                            pass
                        else:
                            print ("error flash write returned ", hex(ret))
                            raise RuntimeError('bad crc')
                        self.check_written(buf,buf_size)
                        break
                    except Exception as e:
                        attempts += 1
                        self.conf()
                        print ("attempts:",attempts)
                        assert attempts < 10
                total += buf_size
                buf_size = 0
                buf = ''
                print ("Wrote %d bytes" % total)

        # reset device
        self.ser.write(b'\x02\x00')
        assert self.ser.read(1)==b'\x82' 

        # reset device
        self.ser.write(b'\x02\x00')
        assert self.ser.read(1)==b'\x82' 

        # reset device
        self.ser.write(b'\x02\x00')
        assert self.ser.read(1)==b'\x82' 

        print ("Device reset")

print ("Once")
port=sys.argv[1]
firmware=open(sys.argv[2], 'r').read()
esc_idx= 1
if len(sys.argv) > 3:
    esc_idx = int(sys.argv[3])
programmers = PI(port)

programmers.prog(firmware,esc_idx)
