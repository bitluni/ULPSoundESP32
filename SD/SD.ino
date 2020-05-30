#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <driver/dac.h>
#include <soc/rtc.h>
#include <math.h>

#include <SD_MMC.h>
#include <FS.h>

// Connect speaker on pin 25 and SD card in MMC mode

// Convert your mp3/acc/flac/wav ... with :
// https://bitluni.net/wp-content/uploads/2018/03/WavetableEditor.html
// or use audacity and an hexadecimal editor (like HxD).

unsigned long samplingRate = 44100;
const int opcodeCount = 17;
const int dacTableStart1 = 2048 - 512;
const int dacTableStart2 = dacTableStart1 - 512;
const int totalSampleWords = 2048 - 512 - (opcodeCount + 1);
const int totalSamples = totalSampleWords * 2;
const int indexAddress = opcodeCount;
const int bufferStart = indexAddress + 1;

signed char Buffer[2][totalSampleWords * 2];

TaskHandle_t Fill_Buffer_Handle;

byte Current_Buffer = 1;

File file;

void startULPSound()
{
  pinMode(2, INPUT_PULLUP);

  if (!SD_MMC.begin())
  {
    Serial.println(F("Cannot init SD"));
  }

  file = SD_MMC.open("/sound.wav");
  if (!file)
  {
    Serial.println(F("cannot open file"));
  }
  file.seek(0);

    Serial.println(F("Calc ULP Clock"));
  //calculate the actual ULP clock
  unsigned long rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 1000);
  unsigned long rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;

    Serial.println(F("Init dac"));
  //initialize DACs
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);
  dac_output_voltage(DAC_CHANNEL_1, 128);
  dac_output_voltage(DAC_CHANNEL_2, 128);

  int retAddress1 = 13;

  int loopCycles = 100;
  Serial.print("Real RTC clock: ");
  Serial.println(rtc_fast_freq_hz);
  int dt = (rtc_fast_freq_hz / samplingRate) - loopCycles;
  if (dt < 0)
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
    I_BXI(3)
  };                  // 4
  // write io and jump back another 12 + 4

    Serial.println(F("Load ULP"));
  size_t load_addr = 0;
  size_t size = sizeof(mono) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(load_addr, mono, &size);
  //  this is how to get the opcodes
  //  for(int i = 0; i < size; i++)
  //    Serial.println(RTC_SLOW_MEM[i], HEX);

  //create DAC opcode tables
  for (int i = 0; i < 256; i++)
  {
    RTC_SLOW_MEM[dacTableStart1 + i * 2] = 0x1D4C0121 | (i << 10); //dac0
    RTC_SLOW_MEM[dacTableStart1 + 1 + i * 2] = 0x80000000 + retAddress1 * 4;
  }

  //set all samples to 128 (silence)
  for (int i = 0; i < totalSampleWords; i++)
    RTC_SLOW_MEM[bufferStart + i] = 0x8080;

  //start
  RTC_SLOW_MEM[indexAddress] = 0;
  ulp_run(0);
  while (RTC_SLOW_MEM[indexAddress] == 0) {delay(1);}
  
}

long currentSample = 0;

long pos = 0;

unsigned char nextSample()
{
  return (unsigned char)((int)Buffer[Current_Buffer][pos++]); //+128
}

int lastFilledWord = 0;

void Fill_Buffer(void *pvParameters)
{
      Serial.println(F("fill buffer"));
  (void)pvParameters;
  for (;;) {
    if (Current_Buffer == 1)
    {
      file.read((byte *)Buffer[0], totalSampleWords * 2);
    }
    else {
      file.read((byte *)Buffer[1], totalSampleWords * 2);
    }
    vTaskSuspend(NULL);
  }
}

void fillSamples(void *pvParameters)
{
  (void)pvParameters;
      Serial.println(F("fillSamples"));
  for (;;) {
    int currentSample = RTC_SLOW_MEM[indexAddress] & 0xffff;
    int currentWord = currentSample >> 1;
    while (lastFilledWord != currentWord)
    {
      unsigned int w = nextSample();
      w |= nextSample() << 8;
      RTC_SLOW_MEM[bufferStart + lastFilledWord] = w;
      lastFilledWord++;
      if (lastFilledWord == totalSampleWords)
      {
        lastFilledWord = 0;
        pos = 0;
        if (Current_Buffer == 1)
        {
          Current_Buffer = 0;
        }
        else
        {
          Current_Buffer = 1;
        }
        vTaskResume(Fill_Buffer_Handle);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.print("Total samples: ");
  Serial.println(totalSamples);
  startULPSound();

  xTaskCreatePinnedToCore(Fill_Buffer, "Fill Buffer", 1024 * 6, NULL, 2, &Fill_Buffer_Handle, 1);
  Current_Buffer = 0;
  xTaskCreatePinnedToCore(fillSamples, "Fill Sample", 1024 * 6, NULL, 2, NULL, 0);
  
}

void loop()
{
  vTaskDelete(NULL);
}
