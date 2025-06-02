# Trigger-Based-Environment-Sensor

-> Project Description

In this project, an intelligent proximity detection and alert system was designed based on motion detection and sensor fusion. The system begins operating when motion is detected by a PIR sensor. Once triggered, it gathers distance measurements using both an IR sensor and an HC-SR04 ultrasonic sensor. These values are then processed with noise filtering and sensor fusion techniques to obtain a more accurate and reliable distance estimation.

The resulting data is used as follows:

* Displayed on an LCD screen in real time.

* A passive buzzer emits beeping sounds, with frequency increasing as the detected object gets closer. The buzzer is controlled using PWM (Pulse Width Modulation) to achieve dynamic tone variation.

* When a very close distance is detected, a LED starts blinking rapidly as a visual alert.

* At this critical proximity, both the buzzer and the LED are activated simultaneously, and a timestamped warning message is sent via UART, using data retrieved from the RTC module.

* The system leverages hardware interrupts to ensure responsive and real-time processing, enhancing both performance and efficiency.

-> Compenents Used

STM32F0DISC

HC-SR04 Ultrasonic Distance Sensor

IR Distance Sensor

PIR Motion Sensor

LED

Resistance (10k)

Passive Buzzer

I2C LCD Screen

UART Module

DS3231 RTC Module

Jumper Cables

Figure 1 : System Overview 

<img src="https://github.com/ssenanb/Trigger-Based-Environment-Sensor/blob/main/system2.jpeg" alt="System Overwiew" width="500"/>

<img src="https://github.com/ssenanb/Trigger-Based-Environment-Sensor/blob/main/system3.jpeg" alt="System Overwiew" width="500"/>

For the system a video : https://vimeo.com/1089594473/f5fc20e37a?share=copy

Figure 2 : The LCD Screen

<img src="https://github.com/ssenanb/Trigger-Based-Environment-Sensor/blob/main/lcd1.jpeg" alt="System Overwiew" width="500"/>

Figure 3 : UART Output (Termite)

<img src="https://github.com/ssenanb/Trigger-Based-Environment-Sensor/blob/main/termite.png" alt="System Overwiew" width="500"/>

-> Pin configuration 

<img src="https://github.com/ssenanb/Trigger-Based-Environment-Sensor/blob/main/configuration.png" alt="System Overwiew" width="500"/>

PA0 -> GPIO_Output -> LED

PA1 -> ADC_IN1 -> IR Distance Sensor

PA8 -> TIM1_CH1 -> Passive Buzzer

PA9 -> USART1_RX

PA10 -> USART1_TX 

PB3 -> TIM2_CH2 -> HC-SR04 - Echo Pin (with Input Capture PWM Mode and Interrupt)

PB4 -> GPIO_EXTI4 -> PIR Sensor

PB5 -> GPIO_Output -> HC-SR04 - Trig Pin

PB6 -> I2C1_SCL -> DS3231 RTC SCL

PB7 -> I2C1_SDA -> DS3231 RTC SDA

PB10 -> I2C2_SCL -> LCD SCL

PB11 -> I2C_SDA -> LCD SDA

STM32F0DISC -> 5V -> Board

STM32F0DISC -> GND -> Board

All the GNDs are connected.

I used this library for the I2C LCD -> https://github.com/alixahedi/i2c-lcd-stm32
