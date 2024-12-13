#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>

#define UART_PORT "/dev/ttyAMA4"

typedef struct {
	uint8_t pkid;
	uint8_t cmd;
	uint8_t len;
	uint8_t data[16];
} tSerialPacket;

typedef struct PacketListItem{
	tSerialPacket packet;
	struct PacketListItem* next;
} tPacketListItem;

typedef struct {
	pthread_mutex_t list_mutex;
	tPacketListItem* list_head;
	tPacketListItem* list_tail;
} tPacketList;

typedef struct {
	tPacketList* packet_list;
	int running;
	int serial_fd;
} tThreadData;

int packet_list_push(tPacketList* list, tSerialPacket* packet) {
	if(packet == NULL)
		return -1;
	tPacketListItem* item = calloc(1, sizeof(tPacketListItem));
	if(item == NULL)
		return -1;
	memcpy(&item->packet, packet, sizeof(tSerialPacket));
	pthread_mutex_lock(&list->list_mutex);
	// add item to end of list
	if(list->list_head == NULL) { // list empty
		list->list_head = item;
		list->list_tail = item;
	} else { // add to end
		list->list_tail->next = item;
		list->list_tail = item;
	}
    pthread_mutex_unlock(&list->list_mutex);
	return 0;
}

int packet_list_pop(tPacketList* list, tSerialPacket* packet) {
	if(packet == NULL)
		return -1;
	int ret = 0;
	pthread_mutex_lock(&list->list_mutex);
	// add item to end of list
	if(list->list_head != NULL) {
		memcpy(packet, &list->list_head->packet, sizeof(tSerialPacket));
		tPacketListItem* next = list->list_head->next;
		free(list->list_head);
		list->list_head = next;
		if(next == NULL)
			list->list_tail = NULL;
		ret = 1;
	}
    pthread_mutex_unlock(&list->list_mutex);
	return ret;
}

int set_interface_attribs(int fd, int speed) {
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;		 /* 8-bit characters */
	tty.c_cflag &= ~PARENB;	 /* no parity bit */
	tty.c_cflag &= ~CSTOPB;	 /* only need 1 stop bit */
	//tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}
uint32_t get_current_time_ms(void) {
	uint32_t ms;
	uint32_t s;
	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	s = spec.tv_sec;
	ms = round(spec.tv_nsec / 1000000);
	if (ms > 999) {
		s++;
		ms = 0;
	}
	ms += s * 1000;
	return ms;
}

int serial_open(void) {
	int fd = open(UART_PORT, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd == -1) {
		perror("open_port: Unable to open " UART_PORT "\n");
	} else {
		set_interface_attribs(fd, B1200);
	}
	return fd;
}

void serial_close(int fd) {
	close(fd);
}

int serial_send(int fd, uint8_t *buffer, size_t length) {
	tcflush(fd, TCIOFLUSH);
	uint8_t ret[16+6] = {0};
	printf("TX[");
	for(int i = 0; i < length; i++) {
		printf("%02X ", buffer[i]);
	}
	printf("]\n");
	int n = write(fd, buffer, length);
	if (n < 0) {
		perror("Write failed\n");
	}
	return n;
}

int serial_read(int fd, uint8_t *buffer, size_t *length) {
	size_t len = 1;
	if (length != NULL)
		len = *length;
	int n = read(fd, buffer, len);
	if (n < 0) {
		perror("Read failed\n");
	} else if (n == 0) {
		printf("No data on port\n");
		if (length != NULL)
			*length = 0;
	} else {
		printf("%i bytes read\n", n);
		if (length != NULL)
			*length = n;
		n = 0;
	}
	return n;
}

int serial_readch(int fd, uint8_t *ch) {
	int r =  read(fd, ch, 1);
	//if(r) printf("{%02X} ",*ch);
	return r;
}

void next_crc16(uint8_t *crc_h, uint8_t *crc_l, uint8_t dat) {
	uint8_t x = *crc_h ^ dat;
	x ^= x >> 4;
	*crc_h = *crc_l ^ (x << 4) ^ (x >> 3);
	*crc_l = (x << 5) ^ x;
}

int send_packet(int fd, tSerialPacket* packet) {
	if (packet->len > 16)
		return -1;
	uint8_t crch = 0xFF;
	uint8_t crcl = 0xFF;
	uint8_t buffer[16 + 7] = {
		0xA5, 0xA5, packet->pkid, packet->cmd, packet->len, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	};
	next_crc16(&crch, &crcl, 0xA5);
	next_crc16(&crch, &crcl, 0xA5);
	next_crc16(&crch, &crcl, packet->pkid);
	next_crc16(&crch, &crcl, packet->cmd);
	next_crc16(&crch, &crcl, packet->len);
	for (int i = 0; i < packet->len; i++) {
		buffer[5 + i] = packet->data[i];
		next_crc16(&crch, &crcl, packet->data[i]);
	}
	buffer[5 + packet->len] = crch;
	buffer[6 + packet->len] = crcl;
	int r = serial_send(fd, buffer, packet->len + 7);
	return r;
}

int flush_serial_rx(int fd) {
	int n = 0;
	uint8_t dummy;
	do {
		n = serial_readch(fd, &dummy);
	} while (n > 0);
	return n;
}
int flush_serial_self_rx(int fd) {
	int n = 0;
	uint8_t dummy;
	uint32_t start;
	do {
		n = serial_readch(fd, &dummy);
	} while (n = 0);
	do {
		start = get_current_time_ms();
		while(get_current_time_ms() - start < 20);
		n = serial_readch(fd, &dummy);
	} while (n > 0);
	return n;
}

uint32_t get_current_time_us(void) {
	uint32_t us;
	uint32_t s;
	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	s = spec.tv_sec;
	us = round(spec.tv_nsec / 1000);
	if (us > 999999) {
		s++;
		us = 0;
	}
	us += s * 1000000;
	return us;
}

int receive_packet(int fd, tSerialPacket *packet, uint32_t timeout) {
	uint32_t now = get_current_time_ms();
	uint8_t ch;
	uint8_t crch = 0xFF;
	uint8_t crcl = 0xFF;
	uint8_t rx_idx;
	uint8_t rx_pkid;
	uint8_t rx_cmd;
	uint8_t rx_len;
	uint8_t rx_buf[16];
	int n;
	static uint8_t header_pos = 0;
	while (1) {
		if (get_current_time_ms() - now > timeout) {
			header_pos = 0;
			return 0;
		}
		n = serial_readch(fd, &ch);
		if (n < 0)
			return n;
		if (n > 0) {
			now = get_current_time_ms();
			switch (header_pos) {
				case 0: // first start char
					if (ch == 0xA5) {
						header_pos++;
						crch = crcl = 0xFF;
						next_crc16(&crch, &crcl, 0xA5);
					}
					break;
				case 1: // second start char
					if (ch == 0xA5) {
						header_pos++;
						next_crc16(&crch, &crcl, 0xA5);
					} else {
						header_pos = 0;
					}
					break;
				case 2: // packet id byte
					rx_pkid = ch;
					next_crc16(&crch, &crcl, ch);
					header_pos++;
					break;
				case 3: // command byte
					rx_cmd = ch;
					next_crc16(&crch, &crcl, ch);
					header_pos++;
					break;
				case 4: // data length byte
					rx_len = ch;
					rx_idx = 0;
					next_crc16(&crch, &crcl, ch);
					header_pos++;
					if (rx_len == 0)
						header_pos++;
					else if (rx_len > 16)
						header_pos = 0;
					break;
				case 5: // data
					rx_buf[rx_idx++] = ch;
					next_crc16(&crch, &crcl, ch);
					if (rx_idx >= rx_len)
						header_pos++;
					break;
				case 6: // crc high byte
					if (ch == crch)
						header_pos++;
					else
						header_pos = 0;
					break;
				case 7: // crc low byte
					if (ch == crcl) {
						packet->pkid = rx_pkid;
						packet->cmd = rx_cmd;
						packet->len = rx_len;
						for (int i = 0; i < rx_len; i++)
							packet->data[i] = rx_buf[i];
						header_pos = 0;
						return 1;
					}
					header_pos = 0;
					break;
				default:
					header_pos = 0;
					return 0;
			}
		}
	}
}



void *serial_rx_thread(void* param) {
	tThreadData* serial_data = (tThreadData*)param;
	int ret;
	tSerialPacket packet;
    while (serial_data->running) {
		ret = receive_packet(serial_data->serial_fd, &packet, 5000);
		if (ret > 0)
			ret = packet_list_push(serial_data->packet_list, &packet);
		if(ret < 0)
			serial_data->running = 0;
    }
	printf("Receiver thread stopped\n");
    return NULL;
}

void* serial_tx_thread(void* param) {
	tThreadData* serial_data = (tThreadData*)param;
	uint8_t count = 0;
	tSerialPacket txpacket = {
		.cmd = 5,
		.pkid = 0,
		.len = 16,
		.data = {
			0x32, 0x11, 0x54, 0x00,
			0x32, 0x11, 0x54, 0x00,
			0x32, 0x11, 0x54, 0x00,
			0x32, 0x11, 0x54, 0x00,
		},
	};
	int ret;
	int state = 0;
	tSerialPacket rxpacket;
	uint32_t send_time = 0;
	while(serial_data->running) {
		switch(state) {
			// send new packet
			case 0:
				send_time = get_current_time_ms();
				while(get_current_time_ms() - send_time < 100);
				txpacket.pkid = count & 0x3F;
				count++;
				ret = send_packet(serial_data->serial_fd, &txpacket);
				send_time = get_current_time_ms();
				state++;
				break;
			// check for reply within timeout
			case 1:
				ret = packet_list_pop(serial_data->packet_list, &rxpacket);
				if(ret < 0)
					break;
				else if(ret > 0)
					state++;
				else if(get_current_time_ms() - send_time > 2000) {
					printf("Timeout\n");
					state = 0;
				}
				break;
			// check if the reply is what we expect
			case 2:
				if(rxpacket.pkid == txpacket.pkid) // this was the transmitted packet
					state = 1; // get another reply
				else if(rxpacket.pkid == (txpacket.pkid | 0x80)) // packet acknowledge
					state++;
				else if(rxpacket.pkid == (txpacket.pkid | 0x40)) // packet neg-acknowledge
					state++;
				else {
					printf("Uknown packet received [%02X]\n", rxpacket.pkid);
					state = 0;
				}
				break;
			// display packet
			case 3:
				printf("RX[A5 A5 %02X %02X %02X ", rxpacket.pkid, rxpacket.cmd, rxpacket.len);
				for(int i = 0; i < rxpacket.len; i++) {
					printf("%02X ", rxpacket.data[i]);
				}
				printf("XX XX ]\n");
				if((rxpacket.pkid & 0x80) != 0) {
					printf("ACK received [%02X]\n", rxpacket.pkid & 0x3F);
				} else {
					printf("NACK received [%02X]\n", rxpacket.pkid & 0x3F);
				}
				state = 0;
				break;
		}
		if(ret < 0) 
			serial_data->running = 0;
	}
	printf("Transmitter thread stopped\n");
	return NULL;
}

int main(int argc, char **argv) {
	pthread_t t1, t2;
	tThreadData serial_rx_data;
	tThreadData serial_tx_data;
	tPacketList packet_list = {.list_head = NULL, .list_tail = NULL, .list_mutex = PTHREAD_MUTEX_INITIALIZER};
	setbuf(stdout, NULL);

	// Open serial port
	int fd = serial_open();
	if (fd < 0)
		return fd;

	// Create serial receive thread
	serial_rx_data.packet_list = &packet_list;
	serial_rx_data.running = 1;
	serial_rx_data.serial_fd = fd;
	pthread_create(&t1, NULL, &serial_rx_thread, &serial_rx_data);

	// Create serial transmit thread
	serial_tx_data.packet_list = &packet_list;
	serial_tx_data.running = 1;
	serial_tx_data.serial_fd = fd;
	pthread_create(&t2, NULL, &serial_tx_thread, &serial_tx_data);

	// do nothing until both threads stop
	pthread_join(t1, NULL);
	pthread_join(t2, NULL);

	// Don't forget to clean up
	close(fd);
	return 0;
}