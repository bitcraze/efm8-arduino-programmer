/**
 * Based on https://gist.github.com/racerxdl/c9a592808acdd9cd178e6e97c83f8baf
 * which was based on: https://github.com/jaromir-sukuba/efm8prog/
 * Use his SW to program EFM8 using arduino uno.
 * www.caron.ws 04/05/2018
**/

// GPIO is manipulated through PORT mappings for better speed.
// Flashes EFM8 at about 10kB/s
// Baud rate: 1000000


// Digital pin 6 on uno. Port and pin nbr is needed for pulsing the clock
#define C2CK_PORT   PORTD
#define C2CK_PIN    6
#define C2CK_GPIO   6


#define ESC_1_C2D_GPIO 5
#define ESC_2_C2D_GPIO 11
#define ESC_3_C2D_GPIO 12
#define ESC_4_C2D_GPIO 2



#define LED LED_BUILTIN 

#define  INBUSY    0x02
#define OUTREADY  0x01

// Debug lines share the binary serial channel; use the updated flash36.py.
// Set to 0 when using other flasher scripts. Never print inside C2 bit pulses.
#define C2_DEBUG 0
#if C2_DEBUG
#define C2_TRACE(message) Serial.println(F("#DBG " message))
#else
#define C2_TRACE(message) do {} while (0)
#endif

static void c2_send_bits (unsigned char data, unsigned char len);
static unsigned char c2_read_bits (unsigned char len);

void c2_rst (void);
void c2_write_addr (unsigned char addr);
static unsigned char c2_read_addr (void);
static unsigned char c2_read_data (void);
static void c2_write_data (unsigned char addr);

unsigned char c2_init_PI (void);
unsigned char c2_read_flash_block (unsigned int addr, unsigned char * data, unsigned char len);
static unsigned char c2_poll_bit_low (unsigned char mask);
static unsigned char c2_poll_bit_high (unsigned char mask);
unsigned char c2_write_flash_block (unsigned int addr, unsigned char * data, unsigned char len);
unsigned char c2_erase_device (void);

uint8_t esc_c2d = ESC_1_C2D_GPIO;

// Bound the entire serial command, including all nested C2 polling loops.
// A timeout stays latched until an explicit PI init or reset command.
const unsigned long C2_COMMAND_TIMEOUT_MS = 1000;
const unsigned long C2_ERASE_TIMEOUT_MS = 10000;
unsigned long c2_command_timeout_ms = C2_COMMAND_TIMEOUT_MS;
unsigned long c2_command_started;
bool c2_failed = false;
uint8_t c2_error = 0x40;
// Block stages: 1=command, 2=address high, 3=address low, 4=length,
// 5=data, 6=completion. Waits: 1=write handshake, 2=read handshake,
// 3=OUTREADY high, 4=INBUSY low.
uint8_t c2_stage = 0, c2_wait = 0, c2_byte = 0;

void teardown();

static bool c2_can_continue() {
  if (c2_failed) return false;
  if ((unsigned long)(millis() - c2_command_started) >= c2_command_timeout_ms) {
    c2_failed = true;
    teardown();
    C2_TRACE("C2 command timed out; clock stopped and pins released");
#if C2_DEBUG
    Serial.print(F("#DBG timeout stage=")); Serial.print(c2_stage);
    Serial.print(F(" byte=")); Serial.print(c2_byte);
    Serial.print(F(" wait=")); Serial.println(c2_wait);
#endif
    return false;
  }
  return true;
}

static bool c2_check_status(uint8_t status) {
  if (c2_failed) return false;
  if (status == 0x0D) return true;
  c2_failed = true;
  c2_error = 0x41;
  teardown();
#if C2_DEBUG
  Serial.print(F("#DBG C2 rejected block stage=")); Serial.print(c2_stage);
  Serial.print(F(" status=0x")); Serial.println(status, HEX);
#endif
  return false;
}

void switch_esc(uint8_t idx){

  switch (idx)
  {
  case 1:
    esc_c2d = ESC_1_C2D_GPIO;
    break;
  case 2:
    esc_c2d = ESC_2_C2D_GPIO;
    break;
  case 3:
    esc_c2d = ESC_3_C2D_GPIO;
    break;   
  case 4:
    esc_c2d = ESC_4_C2D_GPIO;
    break;
  default:
    break;
  }
}

void c2_rst() {
  // teardown() releases C2CK; retries must enable its driver again.
  pinMode(C2CK_GPIO, OUTPUT);
  digitalWrite(C2CK_GPIO, LOW);
  delayMicroseconds(100);
  digitalWrite(C2CK_GPIO, HIGH);
  delayMicroseconds(500);
}

// At 16 MHz: 4 delay cycles + 2 cycles for PORTD's SBI = 375 ns low.
// An ISR stretched this to 6.5+ us in captures (C2 permits at most 5 us).
// Preserve interrupt state and protect only the clock-low interval.
#define c2_pulse_clk() do { \
  uint8_t c2_saved_sreg = SREG; \
  cli(); \
  C2CK_PORT &= ~(1<<C2CK_PIN); \
  __builtin_avr_delay_cycles(4); \
  C2CK_PORT |= (1<<C2CK_PIN); \
  SREG = c2_saved_sreg; \
} while (0)

// START and STOP are clock strobes with the master's data driver disabled.
static void c2_frame_strobe() {
  if (!c2_can_continue()) return;
  pinMode(esc_c2d, INPUT);
  digitalWrite(esc_c2d, LOW); // Disable the Uno's internal pull-up as well.
  c2_pulse_clk();
}

static unsigned char c2_read_bits (unsigned char len) {
  if (!c2_can_continue()) return 0;
  unsigned char i, data, mask;
  mask = 0x01 << (len-1);
  data = 0;
  pinMode(esc_c2d, INPUT);
  for (i=0;i<len;i++) {
    c2_pulse_clk();
    data = data >> 1;
    if (digitalRead(esc_c2d) == HIGH) {
      data = data | mask;
    }
  }
  //pinMode(esc_c2d, OUTPUT);

  return data;
}

static void c2_send_bits (unsigned char data, unsigned char len) {
  if (!c2_can_continue()) return;
  unsigned char i;
  pinMode(esc_c2d, OUTPUT);
  for (i=0;i<len;i++) {
    if (data&0x01) {
      digitalWrite(esc_c2d, HIGH);
    } else {
      digitalWrite(esc_c2d, LOW);
    }
    c2_pulse_clk();
    data = data >> 1;
  }
  pinMode(esc_c2d, INPUT);
}

static void c2_write_data (unsigned char data) {
  unsigned char retval;
  c2_frame_strobe();
  c2_send_bits(0x1, 2);
  c2_send_bits(0x0, 2);
  c2_send_bits(data, 8);
  retval = 0;
  c2_wait = 1;
  while (retval == 0 && c2_can_continue()) {
    retval = c2_read_bits(1);
  }
  c2_frame_strobe();
}

static unsigned char c2_poll_bit_high (unsigned char mask) {
  c2_wait = 3;
  unsigned char retval;
  retval = c2_read_addr();
  while ((retval&mask)==0 && c2_can_continue()) retval = c2_read_addr();
  return retval;
}

static unsigned char c2_poll_bit_low (unsigned char mask) {
  c2_wait = 4;
  unsigned char retval;
  retval = c2_read_addr();
  while (retval&mask && c2_can_continue()) retval = c2_read_addr();
  return retval;
}

unsigned char c2_read_flash_block (unsigned int addr, unsigned char * data, unsigned char len) {
  unsigned char retval,i;
  c2_stage = 1;
  c2_write_addr(0xB4);
  c2_write_data(0x06);
  c2_poll_bit_low(INBUSY);
  c2_poll_bit_high(OUTREADY);
  retval = c2_read_data();
  if (!c2_check_status(retval)) return 0;
  c2_stage = 2;
  c2_write_data(addr>>8);
  c2_poll_bit_low(INBUSY);
  c2_stage = 3;
  c2_write_data(addr&0xFF);
  c2_poll_bit_low(INBUSY);
  c2_stage = 4;
  c2_write_data(len);
  c2_poll_bit_low(INBUSY);
  c2_poll_bit_high(OUTREADY);
  retval = c2_read_data();
  if (!c2_check_status(retval)) return 0;
  for (i=0;i<len;i++) {
    c2_stage = 5; c2_byte = i;
    c2_poll_bit_high(OUTREADY);
    retval = c2_read_data();
    data[i] = retval;
  }
  return i;
}

unsigned char c2_write_flash_block (unsigned int addr, unsigned char * data, unsigned char len) {
  unsigned char retval,i;
  c2_stage = 1;
  c2_write_addr(0xB4);
  c2_write_data(0x07);
  c2_poll_bit_low(INBUSY);
  c2_poll_bit_high(OUTREADY);
  retval = c2_read_data();
#if C2_DEBUG
  Serial.print(F("#DBG write command status=0x")); Serial.println(retval, HEX);
#endif
  if (!c2_check_status(retval)) return 0;
  c2_stage = 2;
  c2_write_data(addr>>8);
  c2_poll_bit_low(INBUSY);
  c2_stage = 3;
  c2_write_data(addr&0xFF);
  c2_poll_bit_low(INBUSY);
  c2_stage = 4;
  c2_write_data(len);
  c2_poll_bit_low(INBUSY);  
  // AN127: after length/INBUSY, send data without reading another status.
  for (i=0;i<len;i++) {
    c2_stage = 5; c2_byte = i;
    c2_write_data(data[i] );
    c2_poll_bit_low(INBUSY);
  } 
  c2_stage = 6;
  c2_poll_bit_high(OUTREADY);
  // Consume and validate completion before issuing another PI command.
  retval = c2_read_data();
  return c2_check_status(retval);
}

unsigned char c2_erase_device (void) {
  unsigned char retval;
  C2_TRACE("erase: select FP data register 0xB4");
  c2_write_addr(0xB4);
  C2_TRACE("erase: send command 0x03 (waiting for C2 write handshake)");
  c2_write_data(0x03);
  C2_TRACE("erase: wait INBUSY low");
  c2_poll_bit_low(INBUSY);
  C2_TRACE("erase: wait OUTREADY high");
  c2_poll_bit_high(OUTREADY);
  C2_TRACE("erase: read command status (waiting for C2 read handshake)");
  retval = c2_read_data();
#if C2_DEBUG
  Serial.print(F("#DBG erase: command status=0x"));
  Serial.println(retval, HEX);
#endif
  if (retval != 0x0D) return 0;
  C2_TRACE("erase: send key 0xDE");
  c2_write_data(0xDE);
  C2_TRACE("erase: key 0xDE, wait INBUSY low");
  c2_poll_bit_low(INBUSY);  
  C2_TRACE("erase: send key 0xAD");
  c2_write_data(0xAD);
  C2_TRACE("erase: key 0xAD, wait INBUSY low");
  c2_poll_bit_low(INBUSY);  
  C2_TRACE("erase: send key 0xA5");
  c2_write_data(0xA5);
  C2_TRACE("erase: key 0xA5, wait INBUSY low");
  c2_poll_bit_low(INBUSY);  
  C2_TRACE("erase: wait completion OUTREADY high");
  c2_poll_bit_high(OUTREADY);
  C2_TRACE("erase: read completion status");
  retval = c2_read_data();
#if C2_DEBUG
  Serial.print(F("#DBG erase: completion status=0x"));
  Serial.println(retval, HEX);
#endif
  if(retval != 0x0D) return 0;
  return 1;
}

unsigned char c2_write_sfr (unsigned char addr, unsigned char val) {
  unsigned char retval;
  c2_write_addr(0xB4);
  c2_write_data(0x0A);
  c2_poll_bit_low(INBUSY);
  c2_poll_bit_high(OUTREADY);
  retval = c2_read_data();
  c2_write_data(addr);
  c2_poll_bit_low(INBUSY);  
  c2_write_data(0x1);
  c2_poll_bit_low(INBUSY);  
  c2_write_data(val);
  c2_poll_bit_low(INBUSY);  
}

unsigned char c2_init_PI (void) {
  C2_TRACE("init: reset target");
 // pinMode(esc_c2d, OUTPUT);
  pinMode(C2CK_GPIO, OUTPUT);
  c2_rst();
  c2_write_addr(0x02);
  //Unlock flash for writing
  C2_TRACE("init: write FPCTL 0x02");
  c2_write_data(0x02);
  C2_TRACE("init: write FPCTL 0x04");
  c2_write_data(0x04);
  C2_TRACE("init: write FPCTL 0x01");
  c2_write_data(0x01);
  delay(20);
  // An undriven C2D line can appear to complete every write handshake.
  // Read the device identity before claiming that initialization succeeded.
  c2_write_addr(0x00);
  unsigned char device_id = c2_read_data();
#if C2_DEBUG
  Serial.print(F("#DBG init: device ID=0x")); Serial.println(device_id, HEX);
#endif
  if (!c2_failed && device_id == 0xFF) {
    c2_failed = true;
    c2_error = 0x42;
    teardown();
    C2_TRACE("init: invalid device ID; target communication not established");
  }
  if (!c2_failed && device_id == 0x32) {
    // EFM8BB2 setup from AN127 table 3.6. These use direct C2 SFR
    // accesses, not the PI Direct Write command used by c2_write_sfr().
    C2_TRACE("init: EFM8BB2 VDD monitor and oscillator setup");
    c2_write_addr(0xFF);
    c2_write_data(0x80);
    delayMicroseconds(5);
    c2_write_addr(0xEF);
    c2_write_data(0x02);
    c2_write_addr(0xA9);
    c2_write_data(0x00);
  }
  if (!c2_failed) C2_TRACE("init: sequence completed; device identity responded");
  return 0;
}


//Not used
unsigned char c2_init_PI_sfr (void) {
  c2_rst();
  c2_write_addr(0x02);
  c2_write_data(0x02);
  c2_write_data(0x04);
  c2_write_data(0x01);

  // set up SFRs
  delay(25);
  c2_write_sfr(0xff, 0x80);
  delay(1);
  c2_write_sfr(0xef, 0x02);
  delay(1);
  c2_write_sfr(0xA9, 0x00);
  return 0;
}

static unsigned char c2_read_data() {
  unsigned char retval;
  c2_frame_strobe();
  c2_send_bits(0x0, 2);
  c2_send_bits(0x0, 2);
  retval = 0;
  c2_wait = 2;
  while (retval == 0 && c2_can_continue()) {
    retval = c2_read_bits(1);
  }
  retval = c2_read_bits(8);
  c2_frame_strobe();
  return retval;
}

static unsigned char c2_read_addr() {
  if (!c2_can_continue()) return 0;
  unsigned char retval;
  c2_frame_strobe();
  c2_send_bits(0x2, 2);
  retval = c2_read_bits(8);
  c2_frame_strobe();
  return retval;
}

void c2_write_addr(unsigned char addr) {
  if (!c2_can_continue()) return;
  c2_frame_strobe();
  c2_send_bits(0x3,2);
  c2_send_bits(addr,8);
  c2_frame_strobe();
}

void setup() {
  Serial.begin(1000000);
  
  pinMode(ESC_1_C2D_GPIO, INPUT);
  pinMode(ESC_2_C2D_GPIO, INPUT);
  pinMode(ESC_3_C2D_GPIO, INPUT);
  pinMode(ESC_4_C2D_GPIO, INPUT);
  pinMode(C2CK_GPIO, INPUT);
  
  digitalWrite(LED, LOW);
  digitalWrite(C2CK_GPIO, HIGH);
  delay(300);
}

void teardown() {
  digitalWrite(esc_c2d, LOW);
  pinMode(ESC_1_C2D_GPIO, INPUT);
  pinMode(ESC_2_C2D_GPIO, INPUT);
  pinMode(ESC_3_C2D_GPIO, INPUT);
  pinMode(ESC_4_C2D_GPIO, INPUT);
  pinMode(C2CK_GPIO, INPUT);
}

unsigned int i;
unsigned char retval;
unsigned char rx_message[300],rx_message_ptr;
unsigned char rx,main_state,bytes_to_receive,rx_state;
unsigned char flash_buffer[300];
unsigned long addr;

unsigned char rx_state_machine (unsigned char state, unsigned char rx_char) {
  if (state==0) {
      rx_message_ptr = 0;
      rx_message[rx_message_ptr++] = rx_char;
      return 1;
  }
  if (state==1) {
      bytes_to_receive = rx_char;
      rx_message[rx_message_ptr++] = rx_char;
      if (bytes_to_receive==0) return 3;
      return 2;
  }
  if (state==2) {
      rx_message[rx_message_ptr++] = rx_char;
      bytes_to_receive--;
      if (bytes_to_receive==0) return 3;
  }
  return state;  
}

#define swap(x) ((((x)>>8) & 0xff) | (((x)<<8) & 0xff00))

void loop() {
  unsigned char crc;
  unsigned char newcrc;
  unsigned char c;
  unsigned long coff;
  if (Serial.available()) {
    rx = Serial.read();
    rx_state = rx_state_machine(rx_state, rx);
    if (rx_state == 3) {
      c2_command_started = millis();
      c2_command_timeout_ms = rx_message[0] == 0x04
          ? C2_ERASE_TIMEOUT_MS : C2_COMMAND_TIMEOUT_MS;
      c2_stage = 0; c2_wait = 0; c2_byte = 0;
      if (rx_message[0] == 0x01 || rx_message[0] == 0x02) {
        c2_failed = false;
        c2_error = 0x40;
      } else if (c2_failed && rx_message[0] >= 0x03 && rx_message[0] <= 0x07) {
        // Do not restart a timed-out transfer when more host bytes arrive.
        Serial.write(c2_error);
        rx_state = 0;
        return;
      }
      switch (rx_message[0]) {
        case 0x0:
        Serial.write(0x80);
        break;
        case 0x01:
          c2_init_PI();
          if (!c2_failed) Serial.write(0x81);
          rx_state = 0;
          break;
        case 0x02:
          c2_rst();
          teardown();
          Serial.write(0x82);
          rx_state = 0;
          break;
        case 0x03:
          addr = (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
          crc = rx_message[6];
          newcrc = rx_message[5] + rx_message[4];
          for (i=0;i<rx_message[2];i++) {
            flash_buffer[i] = rx_message[i+7];
          }

          
          for(i=0; i < rx_message[2]; i++)
          {
            newcrc += flash_buffer[i];
          }

          if (crc != newcrc)
          {
            Serial.write(0x43);
            rx_state = 0;
            break;
          }

          
          c = rx_message[2];
          coff = 0;
          c2_write_flash_block(addr, flash_buffer,c);
         // while (c)
         
         // {
          //  c2_write_flash_block(addr + coff, flash_buffer + coff,min(c, 4));
          //  coff += 4;
          //  c -= min(c, 4);
          //}
          //delay(1);
          if (!c2_failed) Serial.write(0x83);
          rx_state = 0;
          break;
        case 0x04:
          C2_TRACE("erase: command received");
#if C2_DEBUG
          Serial.print(F("#DBG pins: C2CK="));
          Serial.print(C2CK_GPIO);
          Serial.print(F(" C2D="));
          Serial.println(esc_c2d);
#endif
          c = c2_erase_device();
#if C2_DEBUG
          Serial.print(F("#DBG erase: result="));
          Serial.println(c);
#endif
          // 0x44 is an erase failure, never acknowledge a failed erase as success.
          if (!c2_failed) Serial.write(c ? 0x84 : 0x44);
          rx_state = 0;
          break;
        case 0x05:
          // Reject old/incomplete readback frames instead of using stale buffer bytes.
          if (rx_message[1] != 4 || rx_message[2] == 0 || rx_message[3] != 0) {
            Serial.write(0x45);
            rx_state = 0;
            break;
          }
          addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
          c2_read_flash_block(addr,flash_buffer,rx_message[2]);
          if (!c2_failed) {
            // Complete C2 work before acknowledging; never send a partial dump.
            Serial.write(0x85);
            for (i=0;i<rx_message[2];i++) {
              Serial.write(flash_buffer[i]);
            }
          }
          rx_state = 0;
          break;
        case 0x06:
          c2_write_addr(rx_message[3]);
          c2_write_data(rx_message[4]);
          if (!c2_failed) Serial.write(0x86);
          rx_state = 0;
          break;
        case 0x07:
          c2_write_addr(rx_message[3]);
          c = c2_read_data();
          if (!c2_failed) {
            Serial.write(c);
            Serial.write(0x87);
          }
          rx_state = 0;
          break;
        case 0x08:
          Serial.write(0x88);
          switch_esc(1);
          rx_state = 0;
          break;
        case 0x09:
          Serial.write(0x89);
          switch_esc(2);
          rx_state = 0;
          break;
        case 0xa:
          Serial.write(0x8a);
          switch_esc(3);
          rx_state = 0;
          break;
        case 0xb:
          Serial.write(0x8b);
          switch_esc(4);
          rx_state = 0;
          break;
      }
      if (c2_failed && rx_message[0] >= 0x01 && rx_message[0] <= 0x07) {
        teardown();
        Serial.write(c2_error);
        rx_state = 0;
      }
    }
  }
}
