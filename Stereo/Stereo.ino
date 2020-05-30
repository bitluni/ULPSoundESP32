#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <driver/dac.h>
#include <soc/rtc.h>
#include <math.h>

#include "Sound.h"

// Connect left channel on pin 25
// Connect right channel on pin 26

// Convert your mp3/acc/flac/wav ... with :
// https://bitluni.net/wp-content/uploads/2018/03/WavetableEditor.html
// or use audacity and

const unsigned long samplingRate = 44100;

const int opcodeCount = 17;
const int dacTableStart1 = 2048 - 512;
const int dacTableStart2 = dacTableStart1 - 512;
const int totalSampleWords = 2048 - 512 - 512 - (opcodeCount + 1);
const int totalSamples = totalSampleWords * 2;
const int indexAddress = opcodeCount;
const int bufferStart = indexAddress + 1;

void startULPSound() {

  //calculate the actual ULP clock
  unsigned long rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 1000);
  unsigned long rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period; 

  //initialize DACs
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);
  dac_output_voltage(DAC_CHANNEL_1, 128);
  dac_output_voltage(DAC_CHANNEL_2, 128);

  int retAddress1 = 9;
  int retAddress2 = 13;

  int loopCycles = 100;

  Serial.print("Real RTC clock: ");
  Serial.println(rtc_fast_freq_hz);
  
  int dt = (rtc_fast_freq_hz / samplingRate) - loopCycles;

  //if(dt < 0)
  const ulp_insn_t stereo[] = {
    //reset offset register
    I_MOVI(R3, 0),
    //delay to get the right sampling rate
    I_DELAY(dt), // 6 + dt
    //reset sample index
    I_MOVI(R0, 0), // 6
    //write the index back to memory for the main cpu
    I_ST(R0, R3, indexAddress), // 8
    //load the samples
    I_LD(R1, R0, bufferStart), // 8
    //mask the lower 8 bits
    I_ANDI(R2, R1, 255), // 6
    //multiply by 2
    I_LSHI(R2, R2, 1), // 6
    //add start position
    I_ADDI(R2, R2, dacTableStart1),// 6
    //jump to the dac opcode
    I_BXR(R2), // 4
    //back from first dac
    //mask the upper 8 bits
    I_ANDI(R1, R1, 0xff00), // 6
    //shift the upper bits to right and multiply by 2
    I_RSHI(R1, R1, 8 - 1), // 6
    //add start position of second dac table
    I_ADDI(R1, R1, dacTableStart2),// 6
    //jump to the dac opcode
    I_BXR(R1), // 4
    //here we get back from writing the second sample
    //increment the sample index
    I_ADDI(R0, R0, 1), // 6
    //if reached end of the buffer, jump relative to index reset
    I_BGE(-13, totalSampleWords), // 4
    //wait to get the right sample rate (2 cycles more to compensate the index reset)
    I_DELAY((unsigned int)dt + 2), // 8 + dt
    //if not, jump absolute to where index is written to memory
    I_BXI(3)
  }; // 4
  // write io and jump back another 12 + 4 + 12 + 4

  size_t load_addr = 0;
  size_t size = sizeof(stereo) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(load_addr, stereo, &size);
  // this is how to get the opcodes
  // for(int i = 0; i < size; i++)
  // Serial.println(RTC_SLOW_MEM[i], HEX);

  //create DAC opcode tables
  for (int i = 0; i < 256; i++)
  {
    RTC_SLOW_MEM[dacTableStart1 + i * 2] = 0x1D4C0121 | (i << 10); //dac1
    RTC_SLOW_MEM[dacTableStart1 + 1 + i * 2] = 0x80000000 + retAddress1 * 4;
    RTC_SLOW_MEM[dacTableStart2 + i * 2] = 0x1D4C0122 | (i << 10); //dac2
    RTC_SLOW_MEM[dacTableStart2 + 1 + i * 2] = 0x80000000 + retAddress2 * 4;
  }

  //set all samples to 128 (silence)
  for (int i = 0; i < totalSampleWords; i++)
    RTC_SLOW_MEM[bufferStart + i] = 0x8080;

  //start
  RTC_SLOW_MEM[indexAddress] = 0;
  ulp_run(0);

  //wait until sure the index of current sample was written
  while (RTC_SLOW_MEM[indexAddress] == 0) vTaskDelay(100 / portTICK_PERIOD_MS);

}

unsigned char nextSampleLeft() {
  static long pos = 0;
  if (pos >= soundsOffsets[2])
    pos = 0;
  return (unsigned char)((int)soundsSamples[pos++] + 128);
}

unsigned char nextSampleRight() {
  static long pos = 0;
  if (pos >= soundsOffsets[1])
    pos = 0;
  return (unsigned char)((int)soundsSamples[pos++] + 128);
}

int lastFilledWord = 0;

void sound_task(void *pvParameters) {

  startULPSound();

  while (1) {

    int currentSample = RTC_SLOW_MEM[indexAddress] & 0xffff;
    int currentWord = currentSample >> 1;

    while (lastFilledWord != currentWord) {
      unsigned int w = nextSampleLeft();
      w |= nextSampleRight() << 8;
      RTC_SLOW_MEM[bufferStart + lastFilledWord] = w;
      lastFilledWord++;
      if (lastFilledWord == totalSampleWords)
        lastFilledWord = 0;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}

void setup()
{
  Serial.begin(115200);
  Serial.println(totalSampleWords);
  Serial.print("Total stereo samples :");
  Serial.print("Buffer length: ");
  Serial.println((float)totalSampleWords / samplingRate, 3);

  xTaskCreatePinnedToCore(sound_task, "sound task", 1024*6, NULL, 2, NULL, 1); //start the main task
}

void loop()
{
  vTaskDelete(NULL);
}