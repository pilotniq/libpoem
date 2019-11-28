#include <poem/gpio.h>

#include "nrf_gpio.h"

Error gpio_configure( int pin, GPIOdirection direction, GPIOpull pull )
{
    nrf_gpio_pin_dir_t dir;
    nrf_gpio_pin_pull_t nrfPull;

    dir = (direction == GPIO_DIRECTION_INPUT) ? NRF_GPIO_PIN_DIR_INPUT : NRF_GPIO_PIN_DIR_OUTPUT;

    switch( pull )
    {
        case GPIO_PULL_NONE:
            nrfPull = NRF_GPIO_PIN_NOPULL;
            break;

        case GPIO_PULL_DOWN:
            nrfPull = NRF_GPIO_PIN_PULLDOWN;
            break;

        case GPIO_PULL_UP:
            nrfPull = NRF_GPIO_PIN_PULLUP;
            break;

        default:
            // todo: return error;
            nrfPull = NRF_GPIO_PIN_NOPULL;
            break;
    }

    nrf_gpio_cfg( pin, dir, NRF_GPIO_PIN_INPUT_CONNECT, nrfPull, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE );

    return NULL;
}

bool gpio_read( int pin )
{
    return nrf_gpio_pin_read( pin ) != 0;
}

void gpio_write( int pin, bool state )
{
    nrf_gpio_pin_write( pin, state ? 1 : 0 );
}
