#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LTC68042.h"


#define TOTAL_IC  1            // Number of ICs in the isoSPI network LTC6804-2 ICs must be addressed in ascending order starting at 0.

const float MIN_CELL_V = 2.20;                  // Minimum allowable cell voltage. Depends on battery chemistry.
const float MAX_CELL_V = 3.60;                  // Maximum allowable cell voltage. Depends on battery chemistry.
const float CELL_BALANCE_THRESHOLD_V = 3.3;     // Cell balancing occurrs when voltage is above this value

const int CELL_NUM = 4,
          temp_pins[2] = {A2,A3},
          currentPin = A0,
          chargeRelayPin = 8,
          dischargeRelayPin = 9;
          
int error = 0,
    i = 0;

int temp[2],cellMin_index,cellMax_index;

float MAX_TEMP,cellMin_V,cellMax_V;

bool overtemp_state = LOW,
     overcharge_state = LOW,
     undercharge_state = LOW,
     overcurrent_state = LOW,
     dischargeRelay_state = LOW,
     chargeRelay_state = LOW;
     
uint16_t cell_codes[TOTAL_IC][12];
/*!<
  The cell codes will be stored in the cell_codes[][12] array in the following format:

  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
****/

uint16_t aux_codes[TOTAL_IC][6];
/*!<
  The GPIO codes will be stored in the aux_codes[][6] array in the following format:

  |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
  |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
  |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
*/

uint8_t tx_cfg[TOTAL_IC][6];
/*!<
  The tx_cfg[][6] stores the LTC6804 configuration data that is going to be written
  to the LTC6804 ICs on the daisy chain. The LTC6804 configuration data that will be
  written should be stored in blocks of 6 bytes. The array should have the following format:

  |  tx_cfg[0][0]| tx_cfg[0][1] |  tx_cfg[0][2]|  tx_cfg[0][3]|  tx_cfg[0][4]|  tx_cfg[0][5]| tx_cfg[1][0] |  tx_cfg[1][1]|  tx_cfg[1][2]|  .....    |
  |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
  |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

*/

uint8_t rx_cfg[TOTAL_IC][8];
/*!<
  the rx_cfg[][8] array stores the data that is read back from a LTC6804-1 daisy chain.
  The configuration data for each IC  is stored in blocks of 8 bytes. Below is an table illustrating the array organization:

  |rx_config[0][0]|rx_config[0][1]|rx_config[0][2]|rx_config[0][3]|rx_config[0][4]|rx_config[0][5]|rx_config[0][6]  |rx_config[0][7] |rx_config[1][0]|rx_config[1][1]|  .....    |
  |---------------|---------------|---------------|---------------|---------------|---------------|-----------------|----------------|---------------|---------------|-----------|
  |IC1 CFGR0      |IC1 CFGR1      |IC1 CFGR2      |IC1 CFGR3      |IC1 CFGR4      |IC1 CFGR5      |IC1 PEC High     |IC1 PEC Low     |IC2 CFGR0      |IC2 CFGR1      |  .....    |
*/

void setup(){
 pinMode(chargeRelayPin, OUTPUT);
 pinMode(dischargeRelayPin, OUTPUT);
 pinMode(currentPin, INPUT);

 digitalWrite(dischargeRelayPin, LOW); // turn off relays during setup
 digitalWrite(chargeRelayPin, LOW);    // turn off relays during setup

 overcharge_state = HIGH;
 undercharge_state = HIGH;
 
 Serial.begin(9600); 
 LTC6804_initialize();  //Initialize LTC6804 hardware
 init_cfg();            //initialize the 6804 configuration array to be written
 delay(100);
}

void loop(){
  wakeup_sleep();
  Serial.print("ready to measure");
  // read cells:
  wakeup_idle();
  LTC6804_adcv(); // do cell AD conversion and fill cell registers
  delay(10);
  wakeup_idle();
  error = LTC6804_rdcv(0, TOTAL_IC, cell_codes); // read cell voltages from registers
  if (error == -1)
  {
    Serial.println("A PEC error was detected in the received data");
  }

  // print to serial outputs:
  print_cells();
  // measure tempreture
  temp_measurement();

  cellMin_V = 100; //
  cellMax_V = 0;   //     reference values to calcultate min and max cell voltage
  cellMin_index = -1; //  and find their corresponding index
  cellMax_index = -1;  //

   for (int i = 0; i < CELL_NUM ; i++) // finding cell with minimun and max charge
  {
    float V = cell_codes[0][i] * 0.0001;
    if (V < cellMin_V) {
      cellMin_V = V;
      cellMin_index = i;
      Serial.println(cellMin_V , cellMin_index);
    }
    if (V > cellMax_V) {
      cellMax_V = V;
      cellMax_index = i;
      Serial.println(cellMax_V , cellMax_index);
    }
    }

  overcharge_state = HIGH;
  undercharge_state = HIGH;
  overcurrent_state = HIGH;

  if(cellMax_V > MAX_CELL_V){
    overcharge_state = LOW;
  }
  if(cellMin_V < MIN_CELL_V){
    undercharge_state = LOW;
  }

   dischargeRelay_state = overcharge_state && undercharge_state && overtemp_state && overcurrent_state;
   chargeRelay_state = overcharge_state && overtemp_state && overcurrent_state;
  
 digitalWrite(dischargeRelayPin, dischargeRelay_state ); // turn off relays during setup
 digitalWrite(chargeRelayPin, chargeRelay_state );    // turn off relays during setup
  
  
  if (cellMax_V >= CELL_BALANCE_THRESHOLD_V) // cell balanicing 
  {
    balance_cfg(0, cellMax_index);
    Serial.print("Balance ");
    Serial.println(cellMax_index);
  }
  else {
    balance_cfg(0, -1);
  }

  // write tx_cfg to LTC6804. This sets the LTC6804 DCCx registers which control the S pins for balancing:
  LTC6804_wrcfg( TOTAL_IC, tx_cfg);
}

/*!***********************************
  \brief Initializes the configuration array
 **************************************/
void init_cfg(){
  for(int i = 0; i<TOTAL_IC;i++){
    tx_cfg[i][0] = 0x06;
    tx_cfg[i][1] = 0x4E1; // 2v // to set this value check data sheet for calculating
    tx_cfg[i][2] = 0x8CA; // 3.6v //
    tx_cfg[i][3] = 0x00;
    tx_cfg[i][4] = 0x00 ; // discharge switches  0->off  1-> on.  S0 = 0x01, S1 = 0x02, S2 = 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
    tx_cfg[i][5] = 0x20 ; // sets the software timer to 1 minute // check datasheet for setting this value
  }
}

/*!************************************************************
  \brief Prints Cell Voltage Codes to the serial port
 *************************************************************/
void print_cells()
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(" IC ");
    Serial.print(current_ic+1,DEC);
    for(int i=0; i<CELL_NUM; i++)
    {
      Serial.print(" C");
      Serial.print(i+1,DEC);
      Serial.print(":");
      Serial.print(cell_codes[current_ic][i]*.0001, 4);
      Serial.print(",");
    }
     Serial.println();
  }
}

// calculate battery pack tempreture using sensors connected to the arduino

void temp_measurement(){
  for(i=0;i<sizeof(temp_pins);i++){
  float val =analogRead(temp_pins[i]);
  temp[i] = ((val/1024) * 5) ;
  if(temp[i] > MAX_TEMP){
    overtemp_state = LOW;
    Serial.println(" OVER TEMP DETECETED");
  }
  else{
    overtemp_state = HIGH;
  }
}
}

/*!***********************************
  \brief sets  the configuration array for cell balancing
  uses CFGR4 and lowest 4 bits of CGFR5
 **************************************/
void balance_cfg(int ic, int cell)
{
  tx_cfg[ic][4] = 0x00; // clears S1-8
  tx_cfg[ic][5] = tx_cfg[ic][5]  & 0xF0; // clears S9-12 and sets software timer to 1 min // bitwise anding creates value as 00100000
  //Serial.println(tx_cfg[ic][5] & 0xF0,BIN);
  if (cell >= 0 and cell <= 7) {
    tx_cfg[ic][4] = tx_cfg[ic][4] | 1 << cell;
  }
  if ( cell > 7) {
    tx_cfg[ic][5] = tx_cfg[ic][5] | ( 1 << (cell - 8));
  }
}
