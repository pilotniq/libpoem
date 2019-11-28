/*
 * Erl's generic GPIO interface
 */

#include <stdbool.h>
#include <poem/error.h>

typedef enum { GPIO_DIRECTION_INPUT, GPIO_DIRECTION_OUTPUT } GPIOdirection;
typedef enum { GPIO_PULL_NONE, GPIO_PULL_DOWN, GPIO_PULL_UP } GPIOpull;

Error gpio_configure( int pin, GPIOdirection direction, GPIOpull pull );
bool gpio_read( int pin );
void gpio_write( int pin, bool state );
