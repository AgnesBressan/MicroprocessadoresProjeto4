#include <Arduino.h>
#include "driver/ledc.h"

// PWM configuration definitions
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_RESOLUTION LEDC_TIMER_8_BIT // 8-bit resolution
#define PWM_FREQUENCY 5000              // 5 kHz frequency

// PWM channel and GPIO pin definitions
#define PWM_CHANNEL_RED LEDC_CHANNEL_0
#define PWM_CHANNEL_GREEN LEDC_CHANNEL_1
#define PWM_CHANNEL_BLUE LEDC_CHANNEL_2

#define PIN_RED 12
#define PIN_GREEN 27
#define PIN_BLUE 25

// Function to configure the PWM timer
void configure_pwm_timer()
{
    // Set up the PWM timer with the desired parameters
    ledc_timer_config_t pwm_timer = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&pwm_timer);
}

// Function to configure PWM channels for each RGB color
void configure_pwm_channels()
{
    // Define the configuration for each channel
    ledc_channel_config_t pwm_channels[] = {
        {.gpio_num = PIN_RED, // Red channel GPIO pin
         .speed_mode = PWM_MODE,
         .channel = PWM_CHANNEL_RED,
         .timer_sel = PWM_TIMER,
         .duty = 0,
         .hpoint = 0},
        {.gpio_num = PIN_GREEN, // Green channel GPIO pin
         .speed_mode = PWM_MODE,
         .channel = PWM_CHANNEL_GREEN,
         .timer_sel = PWM_TIMER,
         .duty = 0,
         .hpoint = 0},
        {.gpio_num = PIN_BLUE, // Blue channel GPIO pin
         .speed_mode = PWM_MODE,
         .channel = PWM_CHANNEL_BLUE,
         .timer_sel = PWM_TIMER,
         .duty = 0,
         .hpoint = 0},
    };

    // Apply configuration for each channel
    for (int i = 0; i < 3; i++)
    {
        ledc_channel_config(&pwm_channels[i]);
    }
}

// Function to generate the RGB fade effect using PWM
void execute_fade_effect()
{
    int step = 5; // Increment value for duty cycle
    int duty_red = 0, duty_green = 0, duty_blue = 0;

    while (true)
    {
        // Increment duty cycles for each color with different factors
        duty_red += step * 2;
        duty_green += step;
        duty_blue += step * 3;

        // Reset duty cycle values when they exceed 255
        if (duty_red > 255)
            duty_red = 0;
        if (duty_green > 255)
            duty_green = 0;
        if (duty_blue > 255)
            duty_blue = 0;

        // Set and update duty cycle values for each PWM channel
        ledc_set_duty(PWM_MODE, PWM_CHANNEL_RED, duty_red);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL_RED);

        ledc_set_duty(PWM_MODE, PWM_CHANNEL_GREEN, duty_green);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL_GREEN);

        ledc_set_duty(PWM_MODE, PWM_CHANNEL_BLUE, duty_blue);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL_BLUE);

        // Print the current increment and duty cycle values to the Serial Monitor
        Serial.println("-----------------------------------");
        Serial.printf("Increment: %d\n", step);
        Serial.printf("Duty Cycle - R: %d, G: %d, B: %d\n", duty_red, duty_green, duty_blue);

        // Delay for smooth color transitions
        delay(100);
    }
}

// Initial setup function
void setup()
{
    Serial.begin(115200);    // Start Serial communication
    configure_pwm_timer();   // Set up the PWM timer
    configure_pwm_channels(); // Configure the PWM channels
}

// Main loop function
void loop()
{
    execute_fade_effect(); // Run the RGB fade effect continuously
}
