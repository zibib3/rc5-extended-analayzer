#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

typedef struct
{
	char phase;
	char anti_phase;
} bit_t;

// old rc-5:
typedef struct
{
	bit_t start[2];
	bit_t toggle[1];
	bit_t address[5];
	bit_t command[6];
} rc_5_packet;

#define IR_SENSOR_PIN (3)
#define INPUT_BUFFER_SIZE (sizeof(rc_5_packet))
#define MESSAGE_TIMEOUT (889 * (INPUT_BUFFER_SIZE+1))

void receive_callback(uint gpio, uint32_t event_mask);
int64_t timeout_callback(alarm_id_t id, void *user_data);
int bit_array_to_number(bit_t array[], size_t  size);
void parse_message();

static char input_buffer[INPUT_BUFFER_SIZE] = {0};
static absolute_time_t time_buffer[INPUT_BUFFER_SIZE] = {0};
static size_t offset = 0;
static bool full_message_receive = false;

int main()
{
	stdio_init_all();

	gpio_init(IR_SENSOR_PIN);
	gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
	gpio_pull_up(IR_SENSOR_PIN);

    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, &receive_callback);

    while (true)
	{
		if (full_message_receive)
			parse_message();
		else
	    	__wfe();
	}
}

void receive_callback(uint gpio, uint32_t event_mask)
{
	if (offset >= INPUT_BUFFER_SIZE)
		offset = 0;
	time_buffer[offset] = get_absolute_time();
	if (event_mask == GPIO_IRQ_EDGE_FALL)
		input_buffer[offset] = 1;
	else
		input_buffer[offset] = 0;
	if (offset == 0)
	{
		// When timeout, print the packet:
		add_alarm_in_us(MESSAGE_TIMEOUT, timeout_callback, NULL, true);
		gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
	}
	offset++;
}

/*
 * Print the packet:
 */
int64_t timeout_callback(alarm_id_t id, void *user_data)
{
	full_message_receive = true;
	return 0;
}

void parse_message()
{
	gpio_set_irq_enabled(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false);

	char bits[INPUT_BUFFER_SIZE * 2] = {0};
	rc_5_packet *input = (rc_5_packet *)bits;
	size_t size_of_bits = 1;

	printf("Message received at: %lu.%lu\n", to_ms_since_boot(time_buffer[0]) / 1000, to_ms_since_boot(time_buffer[0]) % 1000);
	printf("intervals:");
	for (int i = 0; i < offset; ++i)
	{
		absolute_time_t cur = time_buffer[i];
		absolute_time_t next;
		if (i+1 < offset)
			next = time_buffer[i+1];
		else
			next = time_buffer[i];

		int64_t diff = absolute_time_diff_us(cur, next);
		printf("%lld ", diff);

		if (diff < 1000)
		{
			bits[size_of_bits++] = input_buffer[i];
		}
		else
		{
			bits[size_of_bits++] = input_buffer[i];
			bits[size_of_bits++] = input_buffer[i];
		}
	}
	printf("\n");

	printf("%d bits:\n", size_of_bits);
	for (int i = 0; i < (size_of_bits / 2)+1; ++i)
	{
		if ((i*2) % sizeof(rc_5_packet) / 2)
			printf(" ");
		else
			printf("_");
	}
	printf("\n");
	for (int i = 0; i < size_of_bits / 2; ++i)
	{
		printf("%d", bits[i*2]);
	}
	printf("\n");
	for (int i = 0; i < size_of_bits / 2; ++i)
	{
		printf("%d", bits[i*2+1]);
	}
	printf("\n");

	int start = bit_array_to_number(input->start, sizeof(input->start) / sizeof(input->start[0]));
	int toggle = bit_array_to_number(input->toggle, sizeof(input->toggle) / sizeof(input->toggle[0]));
	int address = bit_array_to_number(input->address, sizeof(input->address) / sizeof(input->address[0]));
	int command = bit_array_to_number(input->command, sizeof(input->command) / sizeof(input->command[0]));
	int extended_command = command + (((~start) & 0b1) << 6);
	printf("Start: %#x(%d)\n", start, start);
	printf("Toggle: %#x(%d)\n", toggle, toggle);
	printf("Address: %#x(%d)\n", address, address);
	printf("E-Command: %#x(%d)\n", extended_command, extended_command);

	offset = 0;
	full_message_receive = false;
	gpio_set_irq_enabled(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true);
}

int bit_array_to_number(bit_t array[], size_t  size)
{
	for (size_t i = 0; i < size; ++i)
	{
		if (array[i].phase == array[i].anti_phase)
			return -1;
	}
	int retval = 0;
	for (size_t i = 0; i < size; ++i)
		retval += (array[i].anti_phase << (size - i - 1));
	return retval;
}
