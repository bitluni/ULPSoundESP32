#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <driver/dac.h>
#include <soc/rtc.h>
#include <math.h>

#include "tetris.h"

unsigned long samplingRate = 44100;
const int opcodeCount = 17;
const int dacTableStart1 = 2048 - 512;
const int dacTableStart2 = dacTableStart1 - 512;
const int totalSampleWords = 2048 - 512 - (opcodeCount + 1);
const int totalSamples = totalSampleWords * 2;
const int indexAddress = opcodeCount;
const int bufferStart = indexAddress + 1;

void startULPSound()
{
  //calculate the actual ULP clock
  unsigned long rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 1000);
  unsigned long rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;

  //initialize DACs
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);
  dac_output_voltage(DAC_CHANNEL_1, 128);
  dac_output_voltage(DAC_CHANNEL_2, 128);

  int retAddress1 = 13;

  int loopCycles = 84;
  Serial.print("Real RTC clock: ");
  Serial.println(rtc_fast_freq_hz);
  int dt = (rtc_fast_freq_hz / samplingRate) - loopCycles;
  if(dt < 0)
    Serial.println("Sampling rate too high"); 
  Serial.print("dt: ");
  Serial.println(dt);
const ulp_insn_t mono[] = {
  //reset offset register
  I_MOVI(R3, 0),
  //delay to get the right sampling rate
  I_DELAY(dt),                // 6 + dt
  //reset sample index
  I_MOVI(R0, 0),              // 6
  //write the index back to memory for the main cpu
  I_ST(R0, R3, indexAddress), // 8
  //divide index by two since we store two samples in each dword
  I_RSHI(R2, R0, 1),          // 6
  //load the samples
  I_LD(R1, R2, bufferStart),  // 8
  //get if odd or even sample
  I_ANDI(R2, R0, 1),          // 6
  //multiply by 8
  I_LSHI(R2, R2, 3),          // 6
  //shift the bits to have the right sample in the lower 8 bits
  I_RSHR(R1, R1, R2),         // 6
  //mask the lower 8 bits
  I_ANDI(R1, R1, 255),        // 6
  //multiply by 2
  I_LSHI(R1, R1, 1),          // 6
  //add start position
  I_ADDI(R1, R1, dacTableStart1),// 6
  //jump to the dac opcode
  I_BXR(R1),                  // 4
  //here we get back from writing a sample
  //increment the sample index
  I_ADDI(R0, R0, 1),          // 6
  //if reached end of the buffer, jump relative to index reset
  I_BGE(-13, totalSamples),   // 4
  //wait to get the right sample rate (2 cycles more to compensate the index reset)
  I_DELAY((unsigned int)dt + 2),            // 8 + dt
  //if not, jump absolute to where index is written to memory
  I_BXI(3)};                  // 4
// write io and jump back another 12 + 4

  size_t load_addr = 0;
  size_t size = sizeof(mono)/sizeof(ulp_insn_t);
  ulp_process_macros_and_load(load_addr, mono, &size);
//  this is how to get the opcodes
//  for(int i = 0; i < size; i++)
//    Serial.println(RTC_SLOW_MEM[i], HEX);
  
  //create DAC opcode tables
  for(int i = 0; i < 256; i++)
  {
    RTC_SLOW_MEM[dacTableStart1 + i * 2] = 0x1D4C0121 | (i << 10); //dac0
    RTC_SLOW_MEM[dacTableStart1 + 1 + i * 2] = 0x80000000 + retAddress1 * 4;
  }

  //set all samples to 128 (silence)
  for(int i = 0; i < totalSampleWords; i++)
    RTC_SLOW_MEM[bufferStart + i] = 0x8080;
    
  //start
  RTC_SLOW_MEM[indexAddress] = 0;
  ulp_run(0);
  while(RTC_SLOW_MEM[indexAddress] == 0) 
    delay(1);
}

long currentSample = 0;
unsigned char nextSample()
{
  static long pos = 0;

  if(pos >= sampleCount)
    pos = 0;
  return (unsigned char)((int)samples[pos++] + 128);
}

int lastFilledWord = 0;

void fillSamples()
{
  int currentSample = RTC_SLOW_MEM[indexAddress] & 0xffff;
  int currentWord = currentSample >> 1;
  while(lastFilledWord != currentWord)
  {
    unsigned int w = nextSample();
    w |= nextSample() << 8;
    RTC_SLOW_MEM[bufferStart + lastFilledWord] = w;
    lastFilledWord++;
    if(lastFilledWord == totalSampleWords)
      lastFilledWord = 0;
  }
}

void setup() 
{
  Serial.begin(115200);
  Serial.print("Total samples: ");
  Serial.println(totalSamples);  
  startULPSound();
}

void loop() 
{
  delay(10);
  //Serial.println(RTC_SLOW_MEM[indexAddress] & 0xffff);
  fillSamples();
}
