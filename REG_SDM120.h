#include <Arduino.h>

#define ID_meter_EASTRON  0x06
#define ID_meter_Mitsu 0x48
#define Total_of_EASTRON  6
#define Total_of_Reg_Mitsu  5
#define Number_of_Reg 10


#define ADDR_NUMBER  16       // จำนวน register ที่ต้องการทั้งหมด

#define Reg_Volt                0x0000      //  0.
#define Reg_Current             0x0006      //  1.
#define Reg_ActivePower         0x000C      //  2.
#define Reg_PowerFactor         0x001E      //  3.
#define Reg_Frequency           0x0046      //  4.
#define Reg_TotalActiveEnergy   0x0156      //  5.
//add
#define Export_active_energy   0x004A      //  5.


// #define ADDR_START   0x0064   // ตำแหน่ง address ของ register ที่ต้องการอ่านค่าเริ่มต้นตั้งแต่ 100 ขึ้นไป
#define ADDR_START   0x0064

#define Reg_Volt_Mitsu                0x0066      //  0.
#define Reg_Current_Mitsu             0x0070      //  1.
#define Reg_Frequency_Mitsu           0x0069      //  3.
#define Reg_ActivePower_Mitsu         0x0073
#define Reg_TotalActiveEnergy_Mitsu   0x006E


uint16_t const Reg_addr_EASTRON[6] = {
  Reg_Volt,
  Reg_Current,
  Reg_ActivePower,
  Reg_PowerFactor,
  Reg_Frequency,
  Reg_TotalActiveEnergy
};

uint16_t const Reg_addr_Mitsu[5] = {
  Reg_Volt_Mitsu,
  Reg_Current_Mitsu,
  Reg_ActivePower_Mitsu,
  Reg_Frequency_Mitsu,
  Reg_TotalActiveEnergy_Mitsu
};

float DATA_METER [Number_of_Reg] ;
float DATA_METER_EASTRON [Total_of_EASTRON];
float DATA_METER_Mitsu [Total_of_Reg_Mitsu] ;
