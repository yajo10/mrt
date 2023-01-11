/*****************************************************************************
* | File      	:   LCD_1in14_test.c
* | Function    :   test Demo 
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2021-03-16
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
******************************************************************************/
#include "LCD_Test.h"   //Examples
#include "LCD_1in14_V2.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <string.h>
#include "pico/binary_info.h"
#include "hardware/spi.h"


//Pins definieren
#define RX 4 //MISO, SDO
#define CS 5 //Chip Select
#define SCK 6 // Clock
#define TX 7 //MOSI, SDA, SDI

/* set address */
bool reserved_addr(uint8_t addr) {
return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

//PRE-Kram vom Sensor
//crazy kram definieren
#define READ_BIT 0x80

int32_t t_fine;

uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int8_t dig_H6;
int16_t dig_H2, dig_H4, dig_H5;

//kompensationsfunktion um lesbare werte aus sensorwerten zu bekommen

int32_t compensate_temp(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3))
            >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t compensate_pressure(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t) t_fine) >> 1) - (int32_t) 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) dig_P6);
    var2 = var2 + ((var1 * ((int32_t) dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t) dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t) dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t) dig_P1)) >> 15);
    if (var1 == 0)
        return 0;

    p = (((uint32_t) (((int32_t) 1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t) var1);
    else
        p = (p / (uint32_t) var1) * 2;

    var1 = (((int32_t) dig_P9) * ((int32_t) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t) (p >> 2)) * ((int32_t) dig_P8)) >> 13;
    p = (uint32_t) ((int32_t) p + ((var1 + var2 + dig_P7) >> 4));

    return p;
}

uint32_t compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t) 76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t) dig_H4) << 20) - (((int32_t) dig_H5) * v_x1_u32r)) +
                   ((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) dig_H6)) >> 10) * (((v_x1_u32r *
                                                                                                  ((int32_t) dig_H3))
            >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
                                                 ((int32_t) dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t) (v_x1_u32r >> 12);
}

//irgendwas mit dem CS einstellen
#ifdef CS
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(CS, 1);
    asm volatile("nop \n nop \n nop");
}
#endif


//wieder SPI initialisieren
#if defined(spi_default) && defined(CS)
static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg & 0x7f;  // remove read bit as this is a write
    buf[1] = data;
    cs_select();
    spi_write_blocking(spi_default, buf, 2);
    cs_deselect();
    sleep_ms(10);
}

static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.
    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(spi_default, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(spi_default, 0, buf, len);
    cs_deselect();
    sleep_ms(10);
}

/* This function reads the manufacturing assigned compensation parameters from the device */
void read_compensation_parameters() {
    uint8_t buffer[26];

    read_registers(0x88, buffer, 24);

    dig_T1 = buffer[0] | (buffer[1] << 8);
    dig_T2 = buffer[2] | (buffer[3] << 8);
    dig_T3 = buffer[4] | (buffer[5] << 8);

    dig_P1 = buffer[6] | (buffer[7] << 8);
    dig_P2 = buffer[8] | (buffer[9] << 8);
    dig_P3 = buffer[10] | (buffer[11] << 8);
    dig_P4 = buffer[12] | (buffer[13] << 8);
    dig_P5 = buffer[14] | (buffer[15] << 8);
    dig_P6 = buffer[16] | (buffer[17] << 8);
    dig_P7 = buffer[18] | (buffer[19] << 8);
    dig_P8 = buffer[20] | (buffer[21] << 8);
    dig_P9 = buffer[22] | (buffer[23] << 8);

    dig_H1 = buffer[25];

    read_registers(0xE1, buffer, 8);

    dig_H2 = buffer[0] | (buffer[1] << 8);
    dig_H3 = (int8_t) buffer[2];
    dig_H4 = buffer[3] << 4 | (buffer[4] & 0xf);
    dig_H5 = (buffer[5] >> 4) | (buffer[6] << 4);
    dig_H6 = (int8_t) buffer[7];
}
//mit dieser Funktion findet der SPI zugriff statt
static void bme280_read_raw(int32_t *humidity, int32_t *pressure, int32_t *temperature) {
    uint8_t buffer[8];

    read_registers(0xF7, buffer, 8);
    *pressure = ((uint32_t) buffer[0] << 12) | ((uint32_t) buffer[1] << 4) | (buffer[2] >> 4);
    *temperature = ((uint32_t) buffer[3] << 12) | ((uint32_t) buffer[4] << 4) | (buffer[5] >> 4);
    *humidity = (uint32_t) buffer[6] << 8 | buffer[7];
}
#endif


void Anzeigen(int a )
{
    int32_t humidity, pressure, temperature;

    //die 3 Werte werden in einem Array gespeichert und zurückgegeben
    bme280_read_raw(&humidity, &pressure, &temperature);

    // These are the raw numbers from the chip, so we need to run through the
    // compensations to get human understandable numbers
    pressure = compensate_pressure(pressure);
    temperature = compensate_temp(temperature);
    humidity = compensate_humidity(humidity);

    //LCD_SetBacklight(1023);
    UDOUBLE Imagesize = LCD_1IN14_V2_HEIGHT*LCD_1IN14_V2_WIDTH*2;
    UWORD *BlackImage;
    if((BlackImage = (UWORD *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        exit(0);
    }
    
    // /*1.Create a new image cache named IMAGE_RGB and fill it with white*/
    Paint_NewImage((UBYTE *)BlackImage,LCD_1IN14_V2.WIDTH,LCD_1IN14_V2.HEIGHT, 0, WHITE);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);
    Paint_Clear(WHITE);
    
    // /* GUI */
    printf("drawing...\r\n");
    // /*2.Drawing on the image*/
#if 1
    // /*3.Refresh the picture in RAM to LCD*/
    LCD_1IN14_V2_Display(BlackImage);
    DEV_Delay_ms(1000);
    DEV_SET_PWM(10);
#endif


    switch(a) //die Print befehle müssen wir jetzt noch durch Übergaben an den Bildschirm ersetzen
    {
        //pressure
        case 1:
            Paint_Clear(WHITE);
            Paint_DrawString_EN(10,20, "Pressure:", &Font16, BLACK, WHITE); 
            Paint_DrawNum (120, 20 ,pressure, &Font16,3,  RED,  WHITE);
            Paint_DrawString_EN(200,10, "bar", &Font16, RED, WHITE);
            LCD_1IN14_V2_Display(BlackImage);
            printf("case1\n");
            DEV_Delay_ms(1000);
            break;
        //Temperature
        case 2:
            Paint_Clear(WHITE);
            Paint_DrawString_EN(10, 20, "Temperature:", &Font16, BLACK, WHITE); 
            Paint_DrawNum (50, 40 ,temperature, &Font16,3,  RED,  WHITE);
            LCD_1IN14_V2_Display(BlackImage);
            printf("case2\n");
            DEV_Delay_ms(1000);
            break;
        //humidity
        case 3:
            Paint_Clear(WHITE);
            Paint_DrawString_EN(10, 20, "Humidity:", &Font16, BLACK, WHITE); 
            Paint_DrawNum (50, 40 ,humidity, &Font16,3,  RED,  WHITE);
            LCD_1IN14_V2_Display(BlackImage);
            printf("case3\n");
            DEV_Delay_ms(1000);
            break;
        //gesamtanzeige
        case 5:
            Paint_Clear(WHITE);
            Paint_DrawString_EN(5, 10, "Pressure:", &Font16, BLACK, WHITE); 
            Paint_DrawNum (100, 20 ,pressure ,&Font16,3,  RED,  WHITE);
            Paint_DrawString_EN(5, 60, "Temperature:", &Font16, BLACK, WHITE); 
            Paint_DrawNum (100, 60 ,temperature/100, &Font16,3,  RED,  WHITE);
            Paint_DrawString_EN(5, 110, "Humidity:", &Font16, BLACK, WHITE); 
            Paint_DrawNum (100, 110 ,humidity/1024, &Font16,3,  RED,  WHITE);
            LCD_1IN14_V2_Display(BlackImage);
            printf("case5\n");
            DEV_Delay_ms(1000);
            break;
    }
    free(BlackImage);
    //BlackImage = NULL;
    
    //DEV_Module_Exit();  //aufpassen vielleicht muss der auch wieder rein?
}




int main(void) //in main umbenennen?
{   //Initilisieruzng Display
    
     //SPI Sensor
    //alles initialisieren
    stdio_init_all();
 
    DEV_Delay_ms(100);
    printf("LCD_1in14_test Demo\r\n");
    if(DEV_Module_Init()!=0){
        return -1;
    }
    DEV_SET_PWM(50);
    /* LCD Init */
    printf("1.14inch LCD demo...\r\n");
    LCD_1IN14_V2_Init(HORIZONTAL);
    LCD_1IN14_V2_Clear(WHITE);
    

   
    //SPI-Verbindung aufbauen
    #if !defined(spi_default) || !defined(SCK) || !defined(TX) || !defined(RX) || !defined(CS)
    #warning spi/bme280_spi example requires a board with SPI pins
        puts("Default SPI pins were not defined");
    #else

        printf("Hello, bme280! Reading raw data from registers via SPI...\n");

        // This example will use SPI0 at 0.5MHz.
        spi_init(spi_default, 500 * 1000);
        gpio_set_function(RX, GPIO_FUNC_SPI);
        gpio_set_function(SCK, GPIO_FUNC_SPI);
        gpio_set_function(TX, GPIO_FUNC_SPI);
        // Make the SPI pins available to picotool
        bi_decl(bi_3pins_with_func(RX, TX, SCK, GPIO_FUNC_SPI));

        // Chip select is active-low, so we'll initialise it to a driven-high state
        gpio_init(CS);
        gpio_set_dir(CS, GPIO_OUT);
        gpio_put(CS, 1);
        // Make the CS pin available to picotool
        bi_decl(bi_1pin_with_name(CS, "SPI CS"));

        // See if SPI is working - interrograte the device for its I2C ID number, should be 0x60
        uint8_t id;
        read_registers(0xD0, &id, 1);
        printf("Chip ID is 0x%x\n", id);

        read_compensation_parameters();

        write_register(0xF2, 0x1); // Humidity oversampling register - going for x1
        write_register(0xF4, 0x27);// Set rest of oversampling modes and run mode to normal

    #endif

 
    uint8_t keyA = 15; 
    uint8_t keyB = 17; 

    uint8_t up = 2;
	uint8_t dowm = 18;
	uint8_t left = 16;
	uint8_t right = 20;
	uint8_t ctrl = 3;
   

    SET_Infrared_PIN(keyA);    
    SET_Infrared_PIN(keyB);
		 
	SET_Infrared_PIN(up);
    SET_Infrared_PIN(dowm);
    SET_Infrared_PIN(left);
    SET_Infrared_PIN(right);
    SET_Infrared_PIN(ctrl);


   int i = 3;
    //Große While Schleife
     //Schleife, die das Programm am Laufen hält
    while (1)
        //if-Bedingungen, die die Modi durchschalten 
       {
        if (i>5)
        {
            i=1;
        }
        if (DEV_Digital_Read(keyA ) == 0)
        {
            i = i+1;
        }
        if (i==4)
        {
            i=1;
        }
        if (DEV_Digital_Read(keyB ) == 0)
        {
            i=5;
        }
        Anzeigen(i);
       
        /**for(int i=3; i<=3; i++){
             Anzeigen(i);   
             if(i==3){
                i=0;
             }
        }**/
        
        }
       

       
       
       /*for(int i = 1; i >= 0; i++ )
         

            //Schleife, die so lange anzeigt, bis zum nächsten Mal der Button gedrückt wird
            
            {
                
                //die Anzeigenfunktion soll abhängig von 1 mit Casestruktur einen 
                //der 3 Werte auslesen und dann an den Bildschirm schicken
                // 1-> pressure, 2-> temperature, 3-> humidity, 5-> Gesamtanzeige
                Anzeigen(5);
                //zur nächsten Iteration von der forschleifen springen, wenn Button 1 gedrückt wird
               f(DEV_Digital_Read(keyA ) == 0)
                {
                    printf("Taste\n");
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
                    break;
                }
                // zur Anzeige aller Werte springen, wenn Button 2 gedrückt wird
                if(DEV_Digital_Read(keyB ) == 0)
                {
                    i = 4;
                    break;
                }
                sleep_ms(500);
            }
            //wenn die Schleife zu weit gewachsen ist, soll sie zurückgesetzt werden -> durchklicken
            if (i =3)
            {
                i = 0;
            }
            // wenn man aus Gesamtanzeige rauskommmt, soll es wieder von vorn in dem Einzelanzeigenkreis losgehen
            if (i>=5)
            {
                i = 0;
            }*/
        
    
    

   
    

    /* Module Exit */
    
    return 0;
}
