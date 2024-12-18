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
  digitalWrite(C2CK_GPIO, LOW);
  delayMicroseconds(55);
  digitalWrite(C2CK_GPIO, HIGH);
  delayMicroseconds(55);
}

#define c2_pulse_clk()\
  C2CK_PORT &= ~(1<<C2CK_PIN); \
  C2CK_PORT |= (1<<C2CK_PIN);


static unsigned char c2_read_bits (unsigned char len) {
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
  c2_send_bits(0x0, 1);
  c2_send_bits(0x1, 2);
  c2_send_bits(0x0, 2);
  c2_send_bits(data, 8);
  retval = 0;
  while (retval == 0) {
    retval = c2_read_bits(1);
  }
  c2_send_bits(0x0, 1);
}

static unsigned char c2_poll_bit_high (unsigned char mask) {
  unsigned char retval;
  retval = c2_read_addr();
  while ((retval&mask)==0) retval = c2_read_addr();
}

static unsigned char c2_poll_bit_low (unsigned char mask) {
  unsigned char retval;
  retval = c2_read_addr();
  while (retval&mask) retval = c2_read_addr();
}

unsigned char c2_read_flash_block (unsigned int addr, unsigned char * data, unsigned char len) {
  unsigned char retval,i;
  c2_write_addr(0xB4);
  c2_write_data(0x06);
  c2_poll_bit_low(INBUSY);
  c2_poll_bit_high(OUTREADY);
  retval = c2_read_data();
  c2_write_data(addr>>8);
  c2_poll_bit_low(INBUSY);
  c2_write_data(addr&0xFF);
  c2_poll_bit_low(INBUSY);
  c2_write_data(len);
  c2_poll_bit_low(INBUSY);
  c2_poll_bit_high(OUTREADY);
  retval = c2_read_data();
  for (i=0;i<len;i++) {
    c2_poll_bit_high(OUTREADY);
    retval = c2_read_data();
    data[i] = retval;
  }
  return i;
}

unsigned char c2_write_flash_block (unsigned int addr, unsigned char * data, unsigned char len) {
  unsigned char retval,i;
  c2_write_addr(0xB4);
  c2_write_data(0x07);
  c2_poll_bit_low(INBUSY);
  c2_poll_bit_high(OUTREADY);
  retval = c2_read_data();
  c2_write_data(addr>>8);
  c2_poll_bit_low(INBUSY);
  c2_write_data(addr&0xFF);
  c2_poll_bit_low(INBUSY);
  c2_write_data(len);
  c2_poll_bit_low(INBUSY);  
  c2_poll_bit_high(OUTREADY);
  retval = c2_read_data();  
  for (i=0;i<len;i++) {
    c2_write_data(data[i] );
    c2_poll_bit_low(INBUSY);
  } 
  c2_poll_bit_high(OUTREADY);
}

unsigned char c2_erase_device (void) {
  unsigned char retval;
  c2_write_addr(0xB4);
  c2_write_data(0x03);
  c2_poll_bit_low(INBUSY);
  c2_poll_bit_high(OUTREADY);
  retval = c2_read_data();
  if (retval != 0x0D) return 0;
  c2_write_data(0xDE);
  c2_poll_bit_low(INBUSY);  
  c2_write_data(0xAD);
  c2_poll_bit_low(INBUSY);  
  c2_write_data(0xA5);
  c2_poll_bit_low(INBUSY);  
  c2_poll_bit_high(OUTREADY);
  retval = c2_read_data();
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
 // pinMode(esc_c2d, OUTPUT);
  pinMode(C2CK_GPIO, OUTPUT);
  c2_rst();
  c2_write_addr(0x02);
  //Unlock flash for writing
  c2_write_data(0x02);
  c2_write_data(0x04);
  c2_write_data(0x01);
  delay(20);
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
  c2_send_bits(0x0, 1);
  c2_send_bits(0x0, 2);
  c2_send_bits(0x0, 2);
  retval = 0;
  while (retval == 0) {
    retval = c2_read_bits(1);
  }
  retval = c2_read_bits(8);
  c2_send_bits(0x0, 1);
  return retval;
}

static unsigned char c2_read_addr() {
  unsigned char retval;
  c2_send_bits(0x0, 1);
  c2_send_bits(0x2, 2);
  retval = c2_read_bits(8);
  c2_send_bits(0x0, 1);
  return retval;
}

void c2_write_addr(unsigned char addr) {
  c2_send_bits(0x0,1);
  c2_send_bits(0x3,2);
  c2_send_bits(addr,8);
  c2_send_bits(0x0,1);  
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
      switch (rx_message[0]) {
        case 0x0:
        Serial.write(0x80);
        break;
        case 0x01:
          c2_init_PI();
          Serial.write(0x81);
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
          Serial.write(0x83);
          rx_state = 0;
          break;
        case 0x04:
          c2_erase_device();
          Serial.write(0x84);
          rx_state = 0;
          break;
        case 0x05:
          Serial.write(0x85);
          addr = (((unsigned long)(rx_message[3]))<<16) + (((unsigned long)(rx_message[4]))<<8) + (((unsigned long)(rx_message[5]))<<0);
          c2_read_flash_block(addr,flash_buffer,rx_message[2]);
          for (i=0;i<rx_message[2];i++) {
            Serial.write(flash_buffer[i]);
          }
          rx_state = 0;
          break;
        case 0x06:
          c2_write_addr(rx_message[3]);
          c2_write_data(rx_message[4]);
          Serial.write(0x86);
          rx_state = 0;
          break;
        case 0x07:
          c2_write_addr(rx_message[3]);
          Serial.write(c2_read_data());
          Serial.write(0x87);
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
    }
  }
}
