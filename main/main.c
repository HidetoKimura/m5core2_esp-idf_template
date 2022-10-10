// SPDX: MIT
// Copyright 2021 Brian Starkey <stark3y@gmail.com>
// Portions from lvgl example: https://github.com/lvgl/lv_port_esp32/blob/master/main/main.c
//
// 

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_log.h>

#include "i2c_manager.h"
#include "m5core2_axp192.h"

#include "lvgl.h"
#include "lvgl_helpers.h"

#define LV_TICK_PERIOD_MS 1

#include "btstack_port_esp32.h"
#include "btstack_run_loop.h"
#include "hci_dump.h"
#include "hci_dump_embedded_stdout.h"

static uint64_t current_tick = 0;

static void gui_timer_tick(void *arg)
{
	// Unused
	(void) arg;

	lv_tick_inc(LV_TICK_PERIOD_MS);

	current_tick++;

}

static void disp_init(void)
{
	static lv_color_t bufs[DISP_BUF_SIZE];
	static lv_disp_draw_buf_t  disp_buf;
	uint32_t size_in_px = DISP_BUF_SIZE;

	// Set up the frame buffers
	lv_disp_draw_buf_init(&disp_buf, bufs, NULL, size_in_px);

	// Set up the display driver
	static lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);
	disp_drv.flush_cb = disp_driver_flush;
	disp_drv.draw_buf = &disp_buf;
	disp_drv.hor_res = LV_HOR_RES_MAX;
	disp_drv.ver_res = LV_VER_RES_MAX;
	lv_disp_drv_register(&disp_drv);

	// Register the touch screen. All of the properties of it
	// are set via the build config
	static lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);
	indev_drv.read_cb = touch_driver_read;
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	lv_indev_drv_register(&indev_drv);
}

#define GUI_WID_BODY_ROW_MAX 	14

typedef struct {
	lv_obj_t* title_label;
	lv_obj_t* body_label[GUI_WID_BODY_ROW_MAX];
} gui_wid_obj_t;

static gui_wid_obj_t gui_wid;

static void widgets_create(void)
{
    gui_wid.title_label = lv_label_create(lv_scr_act());
    lv_label_set_text(gui_wid.title_label, "Hello World");
	lv_obj_align(gui_wid.title_label, LV_ALIGN_TOP_LEFT, 0, 0);

	for (int i = 0; i < GUI_WID_BODY_ROW_MAX; i++) 
	{
		gui_wid.body_label[i] = lv_label_create(lv_scr_act());
		lv_label_set_text_fmt(gui_wid.body_label[i],"%d:", i);
		lv_obj_align(gui_wid.body_label[i], LV_ALIGN_TOP_LEFT, 0, 16 * (i + 1));
	}

}

static void widgets_update(void)
{
	lv_label_set_text_fmt(gui_wid.body_label[GUI_WID_BODY_ROW_MAX - 1], "Current Tick: %lld", current_tick);
}

static void gui_thread(void *pvParameter)
{
	(void) pvParameter;

	disp_init();

	// Timer to drive the main lvgl tick
	const esp_timer_create_args_t periodic_timer_args = {
		.callback = &gui_timer_tick,
		.name = "periodic_gui"
	};
	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

	widgets_create();

	while (1) {
		vTaskDelay(10 / portTICK_PERIOD_MS);

		widgets_update();
		lv_task_handler();
	}

	// Never returns
}

extern int btstack_main(int argc, const char * argv[]);

void app_main(void)
{
	/* Print chip information */
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
			CONFIG_IDF_TARGET,
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	printf("silicon revision %d, ", chip_info.revision);

	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	printf("Free heap: %d\n", esp_get_free_heap_size());
	
	m5core2_init();

	lvgl_i2c_locking(i2c_manager_locking());

	lv_init();
	lvgl_driver_init();

	// Needs to be pinned to a core
	xTaskCreatePinnedToCore(gui_thread, "gui", 4096*2, NULL, 0, NULL, 1);

	printf("Running...\n");
	fflush(stdout);

    // optional: enable packet logger
    // hci_dump_init(hci_dump_embedded_stdout_get_instance());

    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // Setup example
    btstack_main(0, NULL);

    // Enter run loop (forever)
    btstack_run_loop_execute();

	for ( ; ; ) {
		vTaskDelay(portMAX_DELAY);
	}

	printf("Restarting now.\n");
	esp_restart();
}
