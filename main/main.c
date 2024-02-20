
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
//#include "freertos/ringbuf.h"
#include "nvs.h"
#include "nvs_flash.h"
//#include "driver/i2s_std.h" // <<-- ESP-IDF 5.x
#include "driver/i2s.h"		// <<-- ESP-IDF 4.x
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_check.h"
#include "sdkconfig.h"


// NOTE: for ESP-IDF 5.x
//#include "esp_rom_gpio.h"


static const char *TAG = "nyantronics_pcm_recorder";

//#define portTickType int


// === SD CARD ===

#define CONFIG_EXAMPLE_SPI_MOSI_GPIO 23
#define CONFIG_EXAMPLE_SPI_MISO_GPIO 19
#define CONFIG_EXAMPLE_SPI_SCLK_GPIO 18
#define CONFIG_EXAMPLE_SPI_CS_GPIO 5

#define CONFIG_EXAMPLE_SAMPLE_RATE 44100
#define CONFIG_EXAMPLE_BIT_SAMPLE 16
#define SAMPLE_SIZE         (CONFIG_EXAMPLE_BIT_SAMPLE * 1024)
#define BYTE_RATE           (CONFIG_EXAMPLE_SAMPLE_RATE * (CONFIG_EXAMPLE_BIT_SAMPLE / 8)) * NUM_CHANNELS

#define EXAMPLE_BUFF_SIZE	2048
#define SAMPLE_BUFFER_SIZE 	1024

#define SPI_DMA_CHAN        SPI_DMA_CH_AUTO
#define NUM_CHANNELS        (1) // For mono recording only!
#define SD_MOUNT_POINT      "/sdcard"

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
sdmmc_card_t *card;
i2s_chan_handle_t rx_handle = NULL;

//static int16_t i2s_readraw_buff[SAMPLE_SIZE];
//size_t bytes_read;
const int WAVE_HEADER_SIZE = 44;

void mount_sdcard(void) {
    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
	
    ESP_LOGI(TAG, "Initializing SD card");

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CONFIG_EXAMPLE_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_EXAMPLE_SPI_MISO_GPIO,
        .sclk_io_num = CONFIG_EXAMPLE_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
	
	// FIXME: reduce SPI bus freq to 15 MHz. Should not be necessary.
	//host.max_freq_khz = 15000;
	host.max_freq_khz = 2000; // 2 MHz
	
    //ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_EXAMPLE_SPI_CS_GPIO;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        } 
		else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
		
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}

void generate_wav_header(char *wav_header, uint32_t wav_size, uint32_t sample_rate) {
    // See this for reference: http://soundfile.sapp.org/doc/WaveFormat/
    uint32_t file_size = wav_size + WAVE_HEADER_SIZE - 8;
    uint32_t byte_rate = BYTE_RATE;

    const char set_wav_header[] = {
        'R', 'I', 'F', 'F', // ChunkID
        file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
        'W', 'A', 'V', 'E', // Format
        'f', 'm', 't', ' ', // Subchunk1ID
        0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
        0x01, 0x00, // AudioFormat (1 for PCM)
        0x01, 0x00, // NumChannels (1 channel)
        sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
        byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
        0x02, 0x00, // BlockAlign
        0x10, 0x00, // BitsPerSample (16 bits)
        'd', 'a', 't', 'a', // Subchunk2ID
        wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
    };

    memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}


volatile bool read_active = false;

static void i2s_read_task(void *args) {
	// Use POSIX and C standard library functions to work with files.
    int flash_wr_size = 0;
    ESP_LOGI(TAG, "Checking file...");

	// FIXME: writing raw PCM data for now since sample size is unknown.
    /* char wav_header_fmt[WAVE_HEADER_SIZE];
    uint32_t flash_rec_time = BYTE_RATE * rec_time;
    generate_wav_header(wav_header_fmt, flash_rec_time, CONFIG_EXAMPLE_SAMPLE_RATE); */

    // First check if file exists before creating a new file.
    struct stat st;
    if (stat(SD_MOUNT_POINT"/record.pcm", &st) == 0) {
        // Delete it if it exists
		ESP_LOGI(TAG, "Deleting existing file");
        unlink(SD_MOUNT_POINT"/record.pcm");
    }
	
	ESP_LOGI(TAG, "Opening file...");

    // Create new WAV file if it doesn't exist yet.
    FILE *f = fopen(SD_MOUNT_POINT"/record.pcm", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
	
	ESP_LOGI(TAG, "Start recording to file");

    // Write the header to the WAV file
	// FIXME: writing raw PCM data for now since sample size is unknown.
    //fwrite(wav_header_fmt, 1, WAVE_HEADER_SIZE, f);
	
	//uint8_t* r_buf = (uint8_t*) calloc(1, EXAMPLE_BUFF_SIZE);
	//assert(r_buf); // Check if r_buf allocation success
	size_t r_bytes = 0;
	
	int32_t raw_samples[SAMPLE_BUFFER_SIZE];
	int16_t samples16[SAMPLE_BUFFER_SIZE];
	
	/* ATTENTION: The print and delay in the read task only for monitoring the data by human,
	 * Normally there shouldn't be any delays to ensure a short polling time,
	 * Otherwise the dma buffer will overflow and lead to the data lost */
	read_active = true;
	while (read_active) {
		// read from the I2S device
		size_t bytes_read = 0;
		i2s_read(I2S_NUM_0, raw_samples, sizeof(int32_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
		int samples_read = bytes_read / sizeof(int32_t);
		
		for (int i = 0; i < samples_read; i++) {
			samples16[i] = raw_samples[i] >> 16;
		}
		
		// Debug
		printf("Raw: %ld\n", raw_samples[0]);
		printf("16: %d\n", samples16[0]);
		
		// Write samples to SD card & to Bluetooth (depending on configuration).
		//for (int i = 0; i < samples_read; i++) {
			//Serial.printf("%ld\n", raw_samples[i]);
			//printf("Raw: %ld\n", raw_samples[i]);
			//printf("16: %d\n", samples16[i]);
			
			// TODO: Write to Bluetooth.
			//write_ringbuf((const uint8_t*) samples16, samples_read * 2);
			
			// Write samples to the open file on the SD card.
			// TODO: Ensure we aren't running out of space.
			// TODO: write to ring buffer dedicated to the SD card instead.
			fwrite(samples16, 1, bytes_read, f);
		//}
		
		
		/* Read i2s data */
		// i2s_read(I2S_NUM_0, (void*) &buffer32, sizeof(buffer32), &bytesRead, 1000);
		//if (i2s_channel_read(rx_chan, r_buf, EXAMPLE_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) {
		/* if (i2s_read(0, (char*) r_buf, EXAMPLE_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) {
			//printf("Read Task: i2s read %d bytes\n-----------------------------------\n", r_bytes);
			//printf("[0] %x [1] %x [2] %x [3] %x\n[4] %x [5] %x [6] %x [7] %x\n\n",
			  //	 r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4], r_buf[5], r_buf[6], r_buf[7]);
			  
			// Print to serial out.
			uint16_t samples = r_bytes / 2;
			uint16_t* sample = (uint16_t*) r_buf;
			for (int i = 0; i < samples; ++i) {
				printf("%d\n", sample[i]); // newline after each sample for serial plotter.
			}
		} 
		else {
			printf("Read Task: i2s read failed\n");
		} */
		//vTaskDelay(pdMS_TO_TICKS(200));
		
		vTaskDelay(1);
	}
	
	//free(r_buf);
	vTaskDelete(NULL);
}


#define CONFIG_I2S_LRCK_PIN 25
#define CONFIG_I2S_DATA_PIN 27
#define CONFIG_I2S_BCK_PIN 26


// --- INIT MICROPHONE ---
void init_microphone(void) {
	// Set the I2S configuration as PDM and 16bits per sample
		//.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
	i2s_config_t i2s_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_RX,
		.sample_rate = 44100,
		.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
		.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
		.communication_format = I2S_COMM_FORMAT_STAND_I2S,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
		.dma_buf_count = 8,
		.dma_buf_len = 64,
		.use_apll = 0,
		.tx_desc_auto_clear = false,
		.fixed_mclk = 0,
	};

	// Set the pinout configuration (set using menuconfig)
	i2s_pin_config_t pin_config = {
		.mck_io_num = I2S_PIN_NO_CHANGE,
		.bck_io_num = CONFIG_I2S_BCK_PIN,
		.ws_io_num = CONFIG_I2S_LRCK_PIN,
		.data_out_num = I2S_PIN_NO_CHANGE,
		.data_in_num = CONFIG_I2S_DATA_PIN,
	};

	// Call driver installation function before any I2S R/W operation.
	ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
	ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config));
	//ESP_ERROR_CHECK(i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO));
	ESP_ERROR_CHECK(i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO));
}


#define INPUT_PIN1 0
#define INPUT_PIN2 4
#define LED_PIN 2


volatile bool rec_active = false;
static void IRAM_ATTR gpio_interrupt_handler(void *args) {
    int pinNumber = (int) args;
    if (pinNumber == INPUT_PIN1 || pinNumber == INPUT_PIN2) {
		rec_active = !rec_active;
	}
}


// --- DEBUG ONLY ---
#define EXAMPLE_MAX_CHAR_SIZE    64
static esp_err_t s_example_write_file(const char *path, char *data) {
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
	
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}
//--- 


void app_main(void) {
    /* initialize NVS — it is used to store PHY calibration data */
    /* esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
	
    ESP_ERROR_CHECK(ret); */

    /*
     * This application only uses the functions of Classical Bluetooth.
     * Release the controller memory for Bluetooth Low Energy.
     */
   /*  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize controller failed\n", __func__);
        return;
    }
    if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable controller failed\n", __func__);
        return;
    }
    if (esp_bluedroid_init() != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed\n", __func__);
        return;
    }
    if (esp_bluedroid_enable() != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed\n", __func__);
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    // set default parameters for Secure Simple Pairing
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    // Set default parameters for Legacy Pairing
    // Use variable pin, input pin code when pairing
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code); */
	
	// Set up LED GPIO things.
	// DEBUG: Disabling for testing with /* */ block.
	//gpio_pad_select_gpio(LED_PIN);
	gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_PIN, 0);
	
	// Set up record button ISR.
    //gpio_pad_select_gpio(INPUT_PIN1);
	gpio_reset_pin(INPUT_PIN1);
    //gpio_pad_select_gpio(INPUT_PIN2);
	gpio_reset_pin(INPUT_PIN2);
    gpio_set_direction(INPUT_PIN1, GPIO_MODE_INPUT);
    gpio_set_direction(INPUT_PIN2, GPIO_MODE_INPUT);
    //gpio_pulldown_en(INPUT_PIN1);
    gpio_pulldown_en(INPUT_PIN2);
    gpio_pullup_dis(INPUT_PIN1);
    gpio_pullup_dis(INPUT_PIN2);
    gpio_set_intr_type(INPUT_PIN1, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(INPUT_PIN2, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT_PIN1, gpio_interrupt_handler, (void*) INPUT_PIN1);
    gpio_isr_handler_add(INPUT_PIN2, gpio_interrupt_handler, (void*) INPUT_PIN2);
	
	// Enable pull-ups for SPI lines for the SD card.
	// FIXME: doesn't seem to make a difference.
	gpio_set_pull_mode(CONFIG_EXAMPLE_SPI_MOSI_GPIO, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(CONFIG_EXAMPLE_SPI_MISO_GPIO, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(CONFIG_EXAMPLE_SPI_SCLK_GPIO, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(CONFIG_EXAMPLE_SPI_CS_GPIO, GPIO_PULLUP_ONLY);
	
	// Mount the SD card via SPI.
    mount_sdcard();
	
	// Init the microphone.
	// DEBUG: Disable for testing.
	init_microphone();
	
	// --- DEBUG SECTION ---
	// DISABLE AFTER TESTING
	// First create a file.
    const char *file_hello = SD_MOUNT_POINT"/hello.txt";
    char data[EXAMPLE_MAX_CHAR_SIZE];
	esp_err_t ret;
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello", card->cid.name);
    ret = s_example_write_file(file_hello, data);
    if (ret != ESP_OK) {
        return;
    }
	
	//return; // Done with testing.
	// ---
	
	// Wait for record button to be pushed, then record until stopped again.
	rec_active = false;
	bool recording = false;
	while (1) {
		if (rec_active && !recording) {
			/* Step 3: Enable the tx and rx channels before writing or reading data */
			//ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
			
			// Set up ringbuffer.
			/* if ((s_ringbuf_a2dp = xRingbufferCreate(8 * 1024, RINGBUF_TYPE_BYTEBUF)) == NULL) {
				ESP_LOGE(BT_AV_TAG, "%s ringbuffer create failed\n", __func__);
				return;
			} */

			//bt_app_task_start_up();
			
			/* Bluetooth device name, connection mode and profile set up */
			//bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_STACK_UP_EVT, NULL, 0, NULL);

			/* Step 4: Create reading task */
			xTaskCreate(i2s_read_task, "i2s_read_task", 30720, NULL, 5, NULL); // 30 kB stack.
			recording = true;
			gpio_set_level(LED_PIN, 1);
		}
		else if (!rec_active && recording) {
			// Stop current recording.
			read_active = false;
			recording = false;
			gpio_set_level(LED_PIN, 0);
		}
		
		vTaskDelay(1);
	}
}