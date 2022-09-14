#ifndef SDCARD_MOUNT_H
#define SDCARD_MOUNT_H

void sd_card_init();
void sd_card_unmount();
int read_config_file(char *ssid, char *ssid_pass);
void write_to_config_file(char *ssid, char *ssid_pass);
void write_to_log_file(uint8_t class_id, uint8_t score);
void write_photo_to_sd( uint8_t *_jpg_buf, size_t _jpg_buf_len );
#endif