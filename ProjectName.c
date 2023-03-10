#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "vsync_detector.pio.h"
#include "pio_6bpp_color_read.pio.h"
#include "flash_adc.pio.h"

#define ADBUS0_PIN 0
#define ADBUS1_PIN 1
#define ADBUS2_PIN 2
#define ADBUS3_PIN 3
#define ADBUS4_PIN 4
#define ADBUS5_PIN 5
#define ADBUS6_PIN 6
#define ADBUS7_PIN 7

// ACBUS0 tied low for Chip Select
#define ACBUS1_PIN 8

// tied read hi
// #define ACBUS2_PIN 12

#define ACBUS3_PIN 22
// ACBUS4 tied high for SIWU not used (Sleep Immediate Wake up?)

// tied reset hi
// #define RESET_PIN 15

#define TXE ACBUS1_PIN
// #define RD ACBUS2_PIN // could actually just be tied hi for a dumb transmitter.
#define WR ACBUS3_PIN

#define BUSMASK ((1 << ADBUS0_PIN) | (1 << ADBUS1_PIN) | (1 << ADBUS2_PIN) | (1 << ADBUS3_PIN) | (1 << ADBUS4_PIN) | (1 << ADBUS5_PIN) | (1 << ADBUS6_PIN) | (1 << ADBUS7_PIN))


#define IMAGE_SIZE_PIXELS (320*224)
#define PIXELS_PER_WORD 5 // 5px*6bpp+2bits
#define IMAGE_SIZE_WORDS (IMAGE_SIZE_PIXELS/PIXELS_PER_WORD)
#define IMAGE_SIZE_BYTES (IMAGE_SIZE_WORDS*4)

//------------- prototypes -------------//
static void processNextFrame(void);
static inline void write_byte(uint8_t b);
static void pipeImage(void* _unsafe_image_ptr_plz);

PIO vidPIO = pio0;
uint sm_pixels;
uint offset_pixels;
uint32_t IMAGE_DATA[IMAGE_SIZE_WORDS+1];
int dma_chan;

bool frameReady;

// sync reading
// const uint SYNC_OUT_PIN = 22; // this will have to be disabled with FTDI, not enough pins
const uint SYNC_IN_PIN = 15;

// pixel reading
// const uint RGB_IN_STATUS_OUT_PIN = 28;  // this will have to be disabled with FTDI, not enough pins
const uint RGB_IN_START_PIN = 9; // 9 10 b, 11 12 g, 13 14 r

const uint BLUE_ADC_IN_PIN = 16; // 16, 17, 18
const uint BLUE_ADC_OUT_PIN = 9;

const uint GREEN_ADC_IN_PIN = 19; // 19, 20, 21
const uint GREEN_ADC_OUT_PIN = 11;

const uint RED_ADC_IN_PIN = 26; // 26, 27, 28
const uint RED_ADC_OUT_PIN = 13;

int main()
{
    // Init FTDI Pins.
    gpio_init_mask(BUSMASK);
    gpio_set_dir_out_masked(BUSMASK);

    gpio_init(TXE);
    gpio_set_dir(TXE, false);
    gpio_init(WR);
    gpio_set_dir(WR, true);

    gpio_put(WR, true);

    // Init image capture PIO stuff
    PIO adcPIO = pio1;
    uint offset_flashadc = pio_add_program(adcPIO, &flash_adc_program);
    uint sm_flashadc_blue = pio_claim_unused_sm(adcPIO, true);

    flash_adc_program_init(adcPIO, sm_flashadc_blue, offset_flashadc, BLUE_ADC_IN_PIN, BLUE_ADC_OUT_PIN);
    gpio_set_input_enabled(BLUE_ADC_IN_PIN, true);
    gpio_set_input_enabled(BLUE_ADC_IN_PIN+1, true);
    gpio_set_input_enabled(BLUE_ADC_IN_PIN+2, true);

    uint sm_flashadc_grn = pio_claim_unused_sm(adcPIO, true);
    flash_adc_program_init(adcPIO, sm_flashadc_grn, offset_flashadc, GREEN_ADC_IN_PIN, GREEN_ADC_OUT_PIN);
    gpio_set_input_enabled(GREEN_ADC_IN_PIN, true);
    gpio_set_input_enabled(GREEN_ADC_IN_PIN+1, true);
    gpio_set_input_enabled(GREEN_ADC_IN_PIN+2, true);
    
    uint sm_flashadc_red = pio_claim_unused_sm(adcPIO, true);
    flash_adc_program_init(adcPIO, sm_flashadc_red, offset_flashadc, RED_ADC_IN_PIN, RED_ADC_OUT_PIN);
    gpio_set_input_enabled(RED_ADC_IN_PIN, true);
    gpio_set_input_enabled(RED_ADC_IN_PIN+1, true);
    gpio_set_input_enabled(RED_ADC_IN_PIN+2, true);

    // gpio_set_pulls(BLUE_ADC_IN_PIN, false, true);
    // gpio_set_pulls(BLUE_ADC_IN_PIN+1, false, true);
    // gpio_set_pulls(BLUE_ADC_IN_PIN+2, false, true);
    // gpio_set_pulls(GREEN_ADC_IN_PIN, false, true);
    // gpio_set_pulls(GREEN_ADC_IN_PIN+1, false, true);
    // gpio_set_pulls(GREEN_ADC_IN_PIN+2, false, true);
    // gpio_set_pulls(RED_ADC_IN_PIN, false, true);
    // gpio_set_pulls(RED_ADC_IN_PIN+1, false, true);
    // gpio_set_pulls(RED_ADC_IN_PIN+2, false, true);

    // image termination sequence
    // 0b00000011 00000011 00000011 00000011
    // would be
    // 000000 110000 001100 000011 000000 11
    // black  red    green  blue   black
    // unlikely sequence of colors to happen by chance
    IMAGE_DATA[IMAGE_SIZE_WORDS] = 0b00000011000000110000001100000011;


    uint offset_sync = pio_add_program(vidPIO, &vsync_detector_program);
    uint sm_sync = pio_claim_unused_sm(vidPIO, true);

    gpio_set_input_enabled(SYNC_IN_PIN, true);
    
    // PIO pixelReadPIO = pio1;
    offset_pixels = pio_add_program(vidPIO, &pio_6bpp_color_read_program);
    sm_pixels = pio_claim_unused_sm(vidPIO, true);

    for (uint i=0; i<6; i++) {
        gpio_set_input_enabled(RGB_IN_START_PIN+i, true);
    }

    // Force 224 vertical sync in vsync register
    pio_sm_put_blocking(vidPIO, sm_sync, 223);
    pio_sm_exec_wait_blocking(vidPIO, sm_sync, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(vidPIO, sm_sync, pio_encode_mov(pio_y, pio_osr));
    // force 320 pixels wide into hsync
    pio_sm_put_blocking(vidPIO, sm_pixels, 63);
    pio_sm_exec_wait_blocking(vidPIO, sm_pixels, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(vidPIO, sm_pixels, pio_encode_mov(pio_y, pio_osr));

    vsync_detector_program_init(vidPIO, sm_sync, offset_sync, SYNC_IN_PIN);
    pio_6bpp_color_read_program_init(vidPIO, sm_pixels, offset_pixels, RGB_IN_START_PIN);

    // Get a free dma channel, panic() if there are none
    dma_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false); // We will pull from the RX FIFO, so don't move read ptr
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(vidPIO, sm_pixels, false));

    dma_channel_configure(
        dma_chan,                           // Channel to be configured
        &c,                                 // The configuration we just created
        IMAGE_DATA,                         // The initial write address
        &vidPIO->rxf[sm_pixels],            // The initial read address
        IMAGE_SIZE_WORDS,                   // Number of transfers.
        false                               // Do not start immediately.
    );

    frameReady = false;

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (1) {
        processNextFrame();
        gpio_put(LED_PIN, frameReady);
        busy_wait_at_least_cycles(133);
    }

    return 0;
}

static void processNextFrame(void) {
    if (!frameReady) {
        // tud_cdc_write_clear();
        pio_sm_clear_fifos(vidPIO,sm_pixels);
        
        // point the DMA dest to IMAGE_DATA, set xfer size, start the DMA.
        dma_channel_set_write_addr(dma_chan, &IMAGE_DATA[0], true);
        dma_channel_start(dma_chan);

        // then
        // clear vsync flag IRQ4 to indicate  we're ready for a frame
        vidPIO->irq = 0b00010000;

        dma_channel_wait_for_finish_blocking(dma_chan);

        frameReady = true;
        return;
    }

    pipeImage(IMAGE_DATA);
}

static void pipeImage(void* _unsafe_image_ptr_plz) {
    // omg the FTDI version is so much simpler lol
    uint8_t *bytePtr = (uint8_t*)_unsafe_image_ptr_plz;
    for (uint i=0; i<IMAGE_SIZE_BYTES; i++) {
        write_byte(bytePtr[i]);
    }

    for (uint i=0; i<4; i++) {
        write_byte(0b00000011);
    }

    frameReady = false;
}


static inline void write_byte(uint8_t b) {
    // based on 133MHz clock or
    // 133 MHz = 7.518ns per cycle
    // let's try actually using TXE as a read
    gpio_put(WR, true);
    while (gpio_get(TXE)) { // TXE hi means you gotta wait.
        busy_wait_at_least_cycles(1);
    }
    busy_wait_at_least_cycles(5);
    gpio_put_masked(BUSMASK, b);
    busy_wait_at_least_cycles(1);  // DATA to WR# active setup time: 5ns
    gpio_put(WR, false); // WR active is LOW.
    busy_wait_at_least_cycles(5); // WR# active pulse width: 30ns
    gpio_put(WR, true);
    // busy_wait_at_least_cycles(7); // TXE# inactive after WR# cycle: 49 ns
    // actually this last wait cycle is probably irrelevant, since we're waiting for TXE to go low
}
