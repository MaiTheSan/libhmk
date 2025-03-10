
#include "analog.h"
#include "hardware/hardware.h"

#include "board_def.h"

// GPIO ports for each ADC channel
static gpio_type *channel_ports[] = {
    GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOA,
    GPIOB, GPIOB, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC,
};

_Static_assert(M_ARRAY_SIZE(channel_ports) == ADC_NUM_CHANNELS,
               "Invalid number of ADC channels");

// GPIO pins for each ADC channel
static const uint16_t channel_pins[] = {
    GPIO_PINS_0, GPIO_PINS_1, GPIO_PINS_2, GPIO_PINS_3, GPIO_PINS_4, GPIO_PINS_5, GPIO_PINS_6, GPIO_PINS_7,
    GPIO_PINS_0, GPIO_PINS_1, GPIO_PINS_0, GPIO_PINS_1, GPIO_PINS_2, GPIO_PINS_3, GPIO_PINS_4, GPIO_PINS_5,
};

_Static_assert(M_ARRAY_SIZE(channel_pins) == ADC_NUM_CHANNELS,
               "Invalid number of ADC channels");

#if ADC_NUM_MUX_INPUTS > 0
// ADC channels connected to each multiplexer input
static const uint8_t mux_input_channels[] = ADC_MUX_INPUT_CHANNELS;

_Static_assert(M_ARRAY_SIZE(mux_input_channels) == ADC_NUM_MUX_INPUTS,
               "Invalid number of ADC multiplexer inputs");

// GPIO ports for each multiplexer select pin
static gpio_type *mux_select_ports[] = ADC_MUX_SELECT_PORTS;

_Static_assert(M_ARRAY_SIZE(mux_select_ports) == ADC_NUM_MUX_SELECT_PINS,
               "Invalid number of multiplexer select pins");

// GPIO pins for each multiplexer select pin
static const uint16_t mux_select_pins[] = ADC_MUX_SELECT_PINS;

_Static_assert(M_ARRAY_SIZE(mux_select_pins) == ADC_NUM_MUX_SELECT_PINS,
               "Invalid number of multiplexer select pins");

// Matrix containing the key index for each multiplexer input channel and each
// ADC channel. If the value is at least `NUM_KEYS`, the corresponding key is
// not connected.
static const uint16_t mux_input_matrix[][ADC_NUM_MUX_INPUTS] =
    ADC_MUX_INPUT_MATRIX;

_Static_assert(M_ARRAY_SIZE(mux_input_matrix) == (1 << ADC_NUM_MUX_SELECT_PINS),
               "Invalid number of multiplexer select pins");
#endif

#if ADC_NUM_RAW_INPUTS > 0
// ADC channels connected to each raw input
static const uint8_t raw_input_channels[] = ADC_RAW_INPUT_CHANNELS;

_Static_assert(M_ARRAY_SIZE(raw_input_channels) == ADC_NUM_RAW_INPUTS,
               "Invalid number of ADC raw inputs");

// Vector containing the key index for each raw input channel. If the value is
// at least `NUM_KEYS`, the corresponding key is not connected.
static const uint16_t raw_input_vector[] = ADC_RAW_INPUT_VECTOR;

_Static_assert(M_ARRAY_SIZE(raw_input_vector) == ADC_NUM_RAW_INPUTS,
               "Invalid number of ADC raw inputs");
#endif

// Buffer for DMA transfer
__attribute__((aligned(8))) static volatile uint16_t
    adc_buffer[ADC_NUM_MUX_INPUTS + ADC_NUM_RAW_INPUTS];
// ADC values for each key
static volatile uint16_t adc_values[NUM_KEYS];
// Set to true when `adc_values` is filled for the first time
static volatile bool adc_initialized = false;

#if ADC_NUM_MUX_INPUTS > 0
// Current multiplexer channel being processed
static volatile uint8_t current_mux_channel = 0;
#endif

static void gpio_configuration(void);
static void dma_configuration(void);
static void adc_configuration(void);
#if ADC_NUM_MUX_INPUTS > 0
static void tmr_configuration(void);
#endif

void analog_init(void) {
  // Enable peripheral clocks
  gpio_configuration();
  dma_configuration();
  adc_configuration();
#if ADC_NUM_MUX_INPUTS > 0
  tmr_configuration();
#endif

  // Start the conversion loop
  adc_ordinary_software_trigger_enable(ADC1, TRUE);

  // Wait for the ADC values to be initialized
  while (!adc_initialized)
    ;
}

/**
  * @brief  gpio configuration.
  * @param  none
  * @retval none
  */
static void gpio_configuration(void)
{
  gpio_init_type gpio_init_struct;

  // Enable GPIO peripheral clocks
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

#if ADC_NUM_MUX_INPUTS > 0
  // Initialize the multiplexer input channels
  for (uint32_t i = 0; i < ADC_NUM_MUX_INPUTS; i++) {
    uint8_t channel = mux_input_channels[i];

    // Configure GPIO for analog mode
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = channel_pins[channel];
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(channel_ports[channel], &gpio_init_struct);
  }

  // Initialize multiplexer select pins
  for (uint32_t i = 0; i < ADC_NUM_MUX_SELECT_PINS; i++) {
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = mux_select_pins[i];
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(mux_select_ports[i], &gpio_init_struct);

    gpio_bits_reset(mux_select_ports[i], mux_select_pins[i]);
  }
#endif

#if ADC_NUM_RAW_INPUTS > 0
  // Initialize the raw input channels
  for (uint32_t i = 0; i < ADC_NUM_RAW_INPUTS; i++) {
    uint8_t channel = raw_input_channels[i];

    // Configure GPIO for analog mode
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = channel_pins[channel];
    gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(channel_ports[channel], &gpio_init_struct);
  }
#endif
}

/**
  * @brief  dma configuration.
  * @param  none
  * @retval none
  */
static void dma_configuration(void)
{
  dma_init_type dma_init_struct;

  // Enable DMA clock
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

  // Configure DMA interrupt
  nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0);

  // Initialize DMA
  dma_reset(DMA1_CHANNEL1);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = ADC_NUM_MUX_INPUTS + ADC_NUM_RAW_INPUTS;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)adc_buffer;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = TRUE;
  dma_init(DMA1_CHANNEL1, &dma_init_struct);

  // Configure DMAMUX
  dmamux_enable(DMA1, TRUE);
  dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_ADC1);

  // Enable DMA transfer complete interrupt
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

/**
  * @brief  adc configuration.
  * @param  none
  * @retval none
  */
static void adc_configuration(void)
{
  adc_base_config_type adc_base_struct;

  // Enable ADC clock and configure ADC divider
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  adc_clock_div_set(ADC_DIV_16); // Set ADC clock divider

  // Configure ADC interrupt
  nvic_irq_enable(ADC1_IRQn, 0, 0);

  // Initialize ADC base configuration
  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.repeat_mode = FALSE;
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_struct.ordinary_channel_length = ADC_NUM_MUX_INPUTS + ADC_NUM_RAW_INPUTS;
  adc_base_config(ADC1, &adc_base_struct);

#if ADC_NUM_MUX_INPUTS > 0
  // Configure multiplexer input channels
  for (uint32_t i = 0; i < ADC_NUM_MUX_INPUTS; i++) {
    adc_ordinary_channel_set(ADC1, mux_input_channels[i], i + 1, ADC_SAMPLETIME_239_5);
  }
#endif

#if ADC_NUM_RAW_INPUTS > 0
  // Configure raw input channels
  for (uint32_t i = 0; i < ADC_NUM_RAW_INPUTS; i++) {
    adc_ordinary_channel_set(ADC1, raw_input_channels[i], ADC_NUM_MUX_INPUTS + i + 1, ADC_SAMPLETIME_239_5);
  }
#endif

  // Configure ADC trigger source
  adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);

  // Configure DMA mode
  adc_dma_mode_enable(ADC1, TRUE);

  // Enable ADC
  adc_enable(ADC1, TRUE);

  // Calibrate ADC
  adc_calibration_init(ADC1);
  while(adc_calibration_init_status_get(ADC1));
  adc_calibration_start(ADC1);
  while(adc_calibration_status_get(ADC1));
}

#if ADC_NUM_MUX_INPUTS > 0
/**
  * @brief  timer configuration for multiplexer delay.
  * @param  none
  * @retval none
  */
static void tmr_configuration(void)
{
  tmr_base_init_type tmr_base_init_struct;

  // Enable timer clock
  crm_periph_clock_enable(CRM_TMR10_PERIPH_CLOCK, TRUE);

  // Configure timer interrupt
  nvic_irq_enable(TMR10_GLOBAL_IRQn, 0, 0);

  // Initialize timer base configuration
  tmr_base_default_para_init(&tmr_base_init_struct);
  tmr_base_init_struct.period = (system_core_clock / 2000000) * ADC_SAMPLE_DELAY - 1;
  tmr_base_init_struct.div = 0;
  tmr_base_init_struct.clock_division = TMR_CLOCK_DIV1;
  tmr_base_init_struct.repetition_counter = 0;
  tmr_base_init_struct.count_mode = TMR_COUNT_UP;
  tmr_base_init(TMR10, &tmr_base_init_struct);

  // Enable timer overflow interrupt
  tmr_interrupt_enable(TMR10, TMR_OVF_INT, TRUE);
}
#endif

void analog_task(void) {
  // Nothing to do here, all processing is done in ISRs
}

uint16_t analog_read(uint8_t key) {
  return adc_values[key];
}

//--------------------------------------------------------------------+
// IRQ Handlers
//--------------------------------------------------------------------+

void ADC1_IRQHandler(void) {
  if(adc_flag_get(ADC1, ADC_OCCE_FLAG)) {
    adc_flag_clear(ADC1, ADC_OCCE_FLAG);
  }
}

void DMA1_Channel1_IRQHandler(void) {
  if(dma_flag_get(DMA1_FDT1_FLAG)) {
    dma_flag_clear(DMA1_FDT1_FLAG);

#if ADC_NUM_MUX_INPUTS > 0
    for (uint32_t i = 0; i < ADC_NUM_MUX_INPUTS; i++) {
      const uint16_t key = mux_input_matrix[current_mux_channel][i];
      if (key < NUM_KEYS)
        adc_values[key] = adc_buffer[i];
    }
#endif

#if ADC_NUM_RAW_INPUTS > 0
    for (uint32_t i = 0; i < ADC_NUM_RAW_INPUTS; i++) {
      const uint16_t key = raw_input_vector[i];
      if (key < NUM_KEYS)
        adc_values[key] = adc_buffer[ADC_NUM_MUX_INPUTS + i];
    }
#endif

#if ADC_NUM_MUX_INPUTS > 0
    current_mux_channel =
        (current_mux_channel + 1) & ((1 << ADC_NUM_MUX_SELECT_PINS) - 1);
    // We initialize all the ADC values when we have gone through all the
    // multiplexer input channels.
    adc_initialized |= (current_mux_channel == 0);

    // Set the multiplexer select pins
    for (uint32_t i = 0; i < ADC_NUM_MUX_SELECT_PINS; i++) {
      if((current_mux_channel >> i) & 1)
        gpio_bits_set(mux_select_ports[i], mux_select_pins[i]);
      else
        gpio_bits_reset(mux_select_ports[i], mux_select_pins[i]);
    }

    // Delay to allow the multiplexer outputs to settle
    tmr_counter_value_set(TMR10, 0);
    tmr_counter_enable(TMR10, TRUE);
#else
    // We initialize all the ADC values when we have read all the raw input.
    adc_initialized = true;
    // Immediately start the next conversion
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
#endif
  }
}

#if ADC_NUM_MUX_INPUTS > 0
void TMR10_GLOBAL_IRQHandler(void) {
  if(tmr_flag_get(TMR10, TMR_OVF_FLAG)) {
    tmr_flag_clear(TMR10, TMR_OVF_FLAG);

    // Stop the timer to prevent this handler from being called again while the
    // ADC is still converting
    tmr_counter_enable(TMR10, FALSE);

    // Start the next conversion
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
  }
}
#endif