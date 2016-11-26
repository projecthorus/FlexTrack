/*---------------------------------------------------*\
|                                                     |
| LoRa radio code, for downlink, uplink and repeating |
|                                                     |
| Messages can be timed using a GPS reference, to     |
| comply with the TDMA timing requirements.           |
|                                                     |
| Connections:                                        |
|                                                     |
|               Arduino  X - RFM98W DIO5              |
|               Arduino  X - RFM98W DIO0              |
|                                                     |
|               Arduino  X  - RFM98W NSS              |
|               Arduino 11 - RFM98W MOSI              |
|               Arduino 12 - RFM98W MISO              |
|               Arduino 13 - RFM98W CLK               |
|                                                     |
\*---------------------------------------------------*/

#ifdef LORA_NSS

#include <SPI.h>
#include <EEPROM.h>
#include <string.h>

// RFM98 registers
#define REG_FIFO                    0x00
#define REG_OPMODE                  0x01
#define REG_FIFO_ADDR_PTR           0x0D 
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LENGTH          0x22
#define REG_HOP_PERIOD              0x24
#define REG_FREQ_ERROR              0x28
#define REG_DETECT_OPT              0x31
#define	REG_DETECTION_THRESHOLD     0x37
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_TEMP                    0x3C
#define REG_PKT_SNR                 0x19
#define REG_PKT_RSSI                0x1A
#define REG_RSSI                    0x1B

// MODES
#define RF98_MODE_RX_CONTINUOUS     0x85
#define RF98_MODE_TX                0x83
#define RF98_MODE_SLEEP             0x80
#define RF98_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              255

// Modem Config 1
#define EXPLICIT_MODE               0x00
#define IMPLICIT_MODE               0x01

#define ERROR_CODING_4_5            0x02
#define ERROR_CODING_4_6            0x04
#define ERROR_CODING_4_7            0x06
#define ERROR_CODING_4_8            0x08

#define BANDWIDTH_7K8               0x00
#define BANDWIDTH_10K4              0x10
#define BANDWIDTH_15K6              0x20
#define BANDWIDTH_20K8              0x30
#define BANDWIDTH_31K25             0x40
#define BANDWIDTH_41K7              0x50
#define BANDWIDTH_62K5              0x60
#define BANDWIDTH_125K              0x70
#define BANDWIDTH_250K              0x80
#define BANDWIDTH_500K              0x90

// Modem Config 2
#define SPREADING_6                 0x60
#define SPREADING_7                 0x70
#define SPREADING_8                 0x80
#define SPREADING_9                 0x90
#define SPREADING_10                0xA0
#define SPREADING_11                0xB0
#define SPREADING_12                0xC0

#define CRC_OFF                     0x00
#define CRC_ON                      0x04


// POWER AMPLIFIER CONFIG
#define REG_PA_CONFIG               0x09
#define PA_MAX_BOOST                0x8F    // 100mW (max 869.4 - 869.65)
#define PA_LOW_BOOST                0x81
#define PA_MED_BOOST                0x8A
#define PA_MAX_UK                   0x88    // 10mW (max 434)
#define PA_MAX_VK                   0x8C    // 25mW (Max power per Australian LIPD standard)
#define PA_OFF_BOOST                0x00
#define RFO_MIN                     0x00

// 20DBm
#define REG_PA_DAC                  0x4D
#define PA_DAC_20                   0x87

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23  // 0010 0011
#define LNA_OFF_GAIN                0x00

// Packet Types
#define TELEM_PACKET 0
#define TEXT_PACKET 1
#define CUTDOWN_PACKET 2
#define PARAM_PACKET 3
#define ACK_PACKET 4
#define SHORT_TELEM_PACKET 5


typedef enum {lmIdle, lmListening, lmSending} tLoRaMode;

tLoRaMode LoRaMode;
byte currentMode = 0x81;
int TargetID;
struct TBinaryPacket PacketToRepeat;
byte SendRepeatedPacket;
byte RepeatedPacketType=0;
int ImplicitOrExplicit;
uint8_t GroundCount;
int AirCount;
int BadCRCCount;
unsigned char Sentence[SENTENCE_LENGTH];
unsigned long LastLoRaTX=0;



uint8_t LoRa_Slot = 0;
uint8_t LoRa_CycleTime = 0;

void SetupLoRa(void)
{
  setupRFM98();
}

void setupRFM98(void)
{
  int ErrorCoding;
  int Bandwidth;
  int SpreadingFactor;
  int LowDataRateOptimize;
  int PayloadLength;
  
  // initialize the pins
  #ifdef LORA_RESET
    pinMode(LORA_RESET, OUTPUT);
    digitalWrite(LORA_RESET, HIGH);
    delay(10);          // Module needs this before it's ready
  #endif
  pinMode(LORA_NSS, OUTPUT);
  pinMode(LORA_DIO0, INPUT);
  pinMode(LORA_DIO5, INPUT);

  SPI.begin();
  
  // LoRa mode 
  setLoRaMode();

  // Frequency
  setFrequency(LORA_FREQUENCY);

  // LoRa settings for various modes.  We support modes 2 (repeater mode), 1 (normally used for SSDV) and 0 (normal slow telemetry mode).
  #if LORA_MODE == 2
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_62K5;
    SpreadingFactor = SPREADING_8;
    LowDataRateOptimize = 0;		
  #endif

  #if LORA_MODE == 1
    ImplicitOrExplicit = IMPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_5;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_6;
    LowDataRateOptimize = 0;    
  #endif

  // Modified for Project Horus fork. Because yay 125kHz bandwidth.
  #if LORA_MODE == 0  
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_125K;
    SpreadingFactor = SPREADING_10;
    LowDataRateOptimize = 0x08;   
  #endif

  // Not sure whats going on here.
  PayloadLength = ImplicitOrExplicit == IMPLICIT_MODE ? 255 : 0;

  writeRegister(REG_MODEM_CONFIG, ImplicitOrExplicit | ErrorCoding | Bandwidth);
  writeRegister(REG_MODEM_CONFIG2, SpreadingFactor | CRC_ON);
  writeRegister(REG_MODEM_CONFIG3, 0x04 | LowDataRateOptimize);									// 0x04: AGC sets LNA gain
  
  // writeRegister(REG_DETECT_OPT, (SpreadingFactor == SPREADING_6) ? 0x05 : 0x03);					// 0x05 For SF6; 0x03 otherwise
  writeRegister(REG_DETECT_OPT, (readRegister(REG_DETECT_OPT) & 0xF8) | ((SpreadingFactor == SPREADING_6) ? 0x05 : 0x03));  // 0x05 For SF6; 0x03 otherwise
  
  writeRegister(REG_DETECTION_THRESHOLD, (SpreadingFactor == SPREADING_6) ? 0x0C : 0x0A);		// 0x0C for SF6, 0x0A otherwise  
  
  writeRegister(REG_PAYLOAD_LENGTH, PayloadLength);
  writeRegister(REG_RX_NB_BYTES, PayloadLength);
  
  // Change the DIO mapping to 01 so we can listen for TxDone on the interrupt
  writeRegister(REG_DIO_MAPPING_1,0x40);
  writeRegister(REG_DIO_MAPPING_2,0x00);
  
  // Go to standby mode
  setMode(RF98_MODE_STANDBY);
  
  Serial.println("Setup Complete");
}

void setFrequency(double Frequency)
{
  unsigned long FrequencyValue;
    
  Serial.print("Frequency is ");
  Serial.println(Frequency);

  Frequency = Frequency * 7110656 / 434;
  FrequencyValue = (unsigned long)(Frequency);

  Serial.print("FrequencyValue is ");
  Serial.println(FrequencyValue);

  writeRegister(0x06, (FrequencyValue >> 16) & 0xFF);    // Set frequency
  writeRegister(0x07, (FrequencyValue >> 8) & 0xFF);
  writeRegister(0x08, FrequencyValue & 0xFF);
}

void setLoRaMode()
{
  Serial.println("Setting LoRa Mode");
  setMode(RF98_MODE_SLEEP);
  writeRegister(REG_OPMODE,0x80);
   
  Serial.println("LoRa Mode Set");
}

/////////////////////////////////////
//    Method:   Change the LoRa module mode
//////////////////////////////////////
void setMode(byte newMode)
{
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RF98_MODE_TX:
      writeRegister(REG_LNA, LNA_OFF_GAIN);  // TURN LNA OFF FOR TRANSMITT
      writeRegister(REG_PA_CONFIG, PA_MAX_VK); // Modified for Project Horus fork
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      
      break;
    case RF98_MODE_RX_CONTINUOUS:
      writeRegister(REG_PA_CONFIG, PA_OFF_BOOST);  // TURN PA OFF FOR RECIEVE??
      writeRegister(REG_LNA, LNA_MAX_GAIN);  // MAX GAIN FOR RECIEVE
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_SLEEP:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_STANDBY:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
  
  if(newMode != RF98_MODE_SLEEP)
  {
    while(digitalRead(LORA_DIO5) == 0)
    {
    } 
  }
   
  return;
}


/////////////////////////////////////
//    Method:   Read LoRa module Register
//////////////////////////////////////

byte readRegister(byte addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}

/////////////////////////////////////
//    Method:   Write LoRa module Register
//////////////////////////////////////

void writeRegister(byte addr, byte value)
{
  select();
  SPI.transfer(addr | 0x80); // OR address with 10000000 to indicate write enable;
  SPI.transfer(value);
  unselect();
}

/////////////////////////////////////
//    Method:   Select Transceiver
//////////////////////////////////////
void select() 
{
  digitalWrite(LORA_NSS, LOW);
}

/////////////////////////////////////
//    Method:   UNSelect Transceiver
//////////////////////////////////////
void unselect() 
{
  digitalWrite(LORA_NSS, HIGH);
}

// Horus Update:
// Desired behaviour while listening is to immediately respond to a received packet, then go back to listening again.
// It's the ground station's responsibility to transmit at an appropriate time that our response doesn't clash with another payload.

void CheckLoRaRx(void)
{
  if (LoRaMode == lmListening)
  {
    if (digitalRead(LORA_DIO0))
    {
      int Bytes;
					
      Bytes = receiveMessage(Sentence, sizeof(Sentence));
      // Record some RF stats on the received message.
      uint8_t pkt_rssi = (uint8_t)readRegister(REG_PKT_RSSI);
      uint8_t pkt_snr = (uint8_t)readRegister(REG_PKT_SNR);
      Serial.print("Rx "); Serial.print(Bytes); Serial.println(" bytes");
      RepeatedPacketType = 0;
      
      Bytes = min(Bytes, sizeof(Sentence));

      // Delay for a short amount of time, to allow the ground stations to switch back into RX mode.
      delay(250);
					
      if (Bytes > 0)
      {
        // Only act on messages addressed to us.
        if (Sentence[2] != PAYLOAD_ID){
          Serial.println(F("Packet not for us. Ignoring"));
          return;
        }

        // If we're here, the message is directed at us!
        if (Sentence[0] == TELEM_PACKET)
        {
          // Binary Telemetry. Ignore. Not even sure we should ever get here.
          Serial.println(F("Rx Binary Packet")); 
        }
        else if (Sentence[0] == TEXT_PACKET)
        {
          // Text Message Packet. Repeat Immediately.

          // Set the 'is repeated' flag, then re-transmit the text message.
          Sentence[1] = 1;

          SendLoRaPacket(Sentence, Bytes);

          GroundCount++;
        }
        else if (Sentence[0] == CUTDOWN_PACKET)
        {          
          // Cut-down command. Check the auth code, ACK, then act on the command.
          if(!(Sentence[3]==AUTH_0 && Sentence[4]==AUTH_1 && Sentence[5] == AUTH_2))
          {
            return;
          }

          // Capture parameter and param-value from packet before overwrite
          uint8_t cut_time = Sentence[6];
          uint8_t cutdown_count = 0;
          
          //Send ACK packet with appropriate values
          ackPacket(pkt_rssi, pkt_snr, cut_time, 0);
          
          //Sanitize
          if(cut_time > 10)
          {
            cut_time = 10;
          }

          if(cut_time < 2)
          {
            cut_time = 1;
          }

          //Perform cutdown burn
          digitalWrite(LED_WARN, HIGH);
          digitalWrite(PYRO_ENABLE, HIGH);

          for(uint8_t i=0; i<cut_time; i++)
          {
            delay(1000);
          }

          digitalWrite(PYRO_ENABLE,LOW);
          digitalWrite(LED_WARN,LOW);

          cutdown_count = EEPROM.read(EEPROM_CUTDOWN_ATT);
          cutdown_count = cutdown_count + 1;
          EEPROM.write(EEPROM_CUTDOWN_ATT, cutdown_count);
        
          GroundCount++;
        }
        else if (Sentence[0] == PARAM_PACKET)
        {
          // Parameter change command. ACK immediately unless password missmatch.
          
          if(!(Sentence[3]==AUTH_0 && Sentence[4]==AUTH_1 && Sentence[5] == AUTH_2))
          {
            return;
          }

          // Capture parameter and param-value from packet before overwrite
          uint8_t index = Sentence[6];
          uint8_t value = Sentence[7];
         
          // Now work out what parameter we have been asked to change.
          if(index == 0)
          { // Dummy parameter. No changes.
              // Nothing
          }
          else if(index == 1) //TODO may be obsolete due to shift to TDMA based network
          { 
            
          }
          else if(index == 2)  // Enable/disable TDMA mode
          {
            
          }
          else if(index == 3)  // TDMA Slot Number
          {
            
          }

          else if(index == 4) // Payload ID
          {

          //sanitize input
            if(value > 0 && value < 255)
            {
              PAYLOAD_ID = value;
              EEPROM.write(EEPROM_PAYLOAD_ID_ADDR, value);
            }
            
          }
          
          else if(index == 5) // Number of payloads
          {
            
          //sanitize input
            if(value > 0 && value < 255)
            {
              TOTAL_PAYLOADS = value;
              EEPROM.write(EEPROM_PAYLOAD_NUM_ADDR, value);
            }
            
          }

          //Send ACK packet with appropriate values
          ackPacket(pkt_rssi, pkt_snr, index, value);     
          GroundCount++;
        }
        else if (Sentence[0] == SHORT_TELEM_PACKET)
        {
          
        }
        else{
          return;
        }
      }
    }
  }
}

int TimeToSend(void)
{
  // TODO: LoRa_Slot and LoRa_CycleTime need to be calculated *at runtime* not constants written at compile time, per MJ
  LoRa_Slot = (PAYLOAD_ID-1)*5;
  LoRa_CycleTime = 5*TOTAL_PAYLOADS; // Set to zero to send continuously
  
  int CycleSeconds;
	
  SendRepeatedPacket = 0; // Horus: I don't think this variable is used any more.

  if (LoRa_CycleTime == 0)
  {
    // Not using time to decide when we can send
    // Horus: Should modify this so that it listens for a bit instead of just TXing constantly.
    // Could do this by checking a stored value against millis()
    return 1;
  }

  if (millis() > (LastLoRaTX + LoRa_CycleTime*1000+2000))
  {
    // Timed out. Transmit anyway. 
    // Horus: Not 100% sure we want this. May end up with collisions.
    return 1;
  }
  // Horus: Note that this TDMA stuffis only going to work if we have GPS lock.
  // We should probably consider a backup option (that's better than the 'timeout' solution above.)
  if (GPS.Satellites > 1) 
  {
    static int LastCycleSeconds=-1;

    // Can't Tx twice at the same time
    CycleSeconds = GPS.SecondsInDay % LoRa_CycleTime;
    
    if (CycleSeconds != LastCycleSeconds)
    {
      LastCycleSeconds = CycleSeconds;
      
      // If we're in our slot, then transmit.
      if (CycleSeconds == LoRa_Slot)
      {
        SendRepeatedPacket = 0;
        return 1;
      }
    }
  }
    
  return 0;
}

int LoRaIsFree(void)
{
  if ((LoRaMode != lmSending) || digitalRead(LORA_DIO0))
  {
    // Either not sending, or was but now it's sent.  Clear the flag if we need to
    if (LoRaMode == lmSending)
    {
      // Clear that IRQ flag
      writeRegister( REG_IRQ_FLAGS, 0x08); 
      LoRaMode = lmIdle;
    }
				
    // Now we test to see if we're doing TDM or not
    // For TDM, if it's not a slot that we send in, then we should be in listening mode
    // Otherwise, we just send
				
    if (TimeToSend())
    {
      // Either sending continuously, or it's our slot to send in
      // printf("Channel %d is free\n", Channel);
					
      return 1;
    }
    
    if (LoRa_CycleTime > 0)
    {
      // TDM system and not time to send, so we can listen
      if (LoRaMode == lmIdle)
      {
        startReceiving();
      }
    }
  }
  
  return 0;
}

void SendLoRaPacket(unsigned char *buffer, int Length)
{
  int i;
  
  LastLoRaTX = millis();
  
  Serial.print("Sending "); Serial.print(Length);Serial.println(" bytes");
  
  setMode(RF98_MODE_STANDBY);

  writeRegister(REG_DIO_MAPPING_1, 0x40);		// 01 00 00 00 maps DIO0 to TxDone
  writeRegister(REG_FIFO_TX_BASE_AD, 0x00);  // Update the address ptr to the current tx base address
  writeRegister(REG_FIFO_ADDR_PTR, 0x00); 
  if (ImplicitOrExplicit == EXPLICIT_MODE)
  {
    writeRegister(REG_PAYLOAD_LENGTH, Length);
  }
  select();
  // tell SPI which address you want to write to
  SPI.transfer(REG_FIFO | 0x80);

  // loop over the payload and put it on the buffer 
  for (i = 0; i < Length; i++)
  {
    SPI.transfer(buffer[i]);
  }
  unselect();

  // go into transmit mode
  setMode(RF98_MODE_TX);
  
  LoRaMode = lmSending;
}

void ackPacket(uint8_t rssi, uint8_t snr, uint8_t paramIndex, uint8_t paramValue)
{
  const uint8_t ACK_LENGTH = 8;
  unsigned char AckPayload[ACK_LENGTH];
  
  AckPayload[0] = 4; // ACK Packet.
  AckPayload[1] = 0;
  AckPayload[2] = PAYLOAD_ID;
  AckPayload[3] = rssi;
  AckPayload[4] = snr;
  AckPayload[5] = 3;
  AckPayload[6] = paramIndex;  
  AckPayload[7] = paramValue;  
  
  //Send ACK packet
  SendLoRaPacket(AckPayload, ACK_LENGTH);
}

void startReceiving(void)
{
  writeRegister(REG_DIO_MAPPING_1, 0x00);		// 00 00 00 00 maps DIO0 to RxDone
	
  writeRegister(REG_FIFO_RX_BASE_AD, 0);
  writeRegister(REG_FIFO_ADDR_PTR, 0);
	  
  // Setup Receive Continuous Mode
  setMode(RF98_MODE_RX_CONTINUOUS); 
		
  LoRaMode = lmListening;
}

int receiveMessage(unsigned char *message, int MaxLength)
{
  int i, Bytes, currentAddr, x;

  Bytes = 0;
	
  x = readRegister(REG_IRQ_FLAGS);
  
  // clear the rxDone flag
  writeRegister(REG_IRQ_FLAGS, 0x40); 
    
  // check for payload crc issues (0x20 is the bit we are looking for
  if((x & 0x20) == 0x20)
  {
    // CRC Error
    writeRegister(REG_IRQ_FLAGS, 0x20);		// reset the crc flags
    BadCRCCount++;
  }
  else
  {
    currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
    Bytes = readRegister(REG_RX_NB_BYTES);
    Bytes = min(Bytes, MaxLength-1);

    writeRegister(REG_FIFO_ADDR_PTR, currentAddr);   
		
    for(i = 0; i < Bytes; i++)
    {
      message[i] = (unsigned char)readRegister(REG_FIFO);
    }
    message[Bytes] = '\0';

    // Clear all flags
    writeRegister(REG_IRQ_FLAGS, 0xFF); 
  }
  
  return Bytes;
}

// Updated for Project Horus binary packet format.
int BuildLoRaPositionPacket(unsigned char *TxLine)
{
  struct TBinaryPacket BinaryPacket;

  SentenceCounter++;
  BinaryPacket.PacketType = 0;
  BinaryPacket.PayloadFlags = 0;
  BinaryPacket.PayloadIDs = LORA_ID;
  BinaryPacket.Counter = SentenceCounter;
  BinaryPacket.BiSeconds = GPS.SecondsInDay / 2L;
  BinaryPacket.Latitude = GPS.Latitude;
  BinaryPacket.Longitude = GPS.Longitude;
  // Sanitise the GPS altitude before we convert to a uint16_t
  if(GPS.Altitude<0){GPS.Altitude = 0;}
  BinaryPacket.Altitude = (uint16_t)GPS.Altitude;
  BinaryPacket.Speed = 0;//GPS.Speed; // Currently the GPS library doesn't get speed-over-ground. We calculate this on the ground instead, using position deltas.
  BinaryPacket.BattVoltage = GetBattVoltage();
  BinaryPacket.PyroVoltage = GetPyroVoltage();
  BinaryPacket.Sats = GPS.Satellites;
  BinaryPacket.Temp = readRegister(REG_TEMP);
  BinaryPacket.rxPktCount = GroundCount;
  BinaryPacket.rxRSSI = readRegister(REG_RSSI);
  BinaryPacket.telemFlags = 0; // Horus: We could use this field to indicate a cutdown command has succeeded.

  memcpy(TxLine, &BinaryPacket, sizeof(BinaryPacket));
	
  return sizeof(struct TBinaryPacket);
}


void CheckLoRa(void)
{

  CheckLoRaRx();
		
  if (LoRaIsFree())
  {		
    // Transmit a binary packet.
    Serial.println("Time to send.");
    int PacketLength;

    PacketLength = BuildLoRaPositionPacket(Sentence);
    Serial.println(F("LoRa: Tx Binary packet"));
						
    SendLoRaPacket(Sentence, PacketLength);		
    
  }
}

#endif

