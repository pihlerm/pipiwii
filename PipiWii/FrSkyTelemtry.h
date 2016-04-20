#ifndef FrSkyTelemtry_H_
#define FrSkyTelemtry_H_

   void telemetry_frsky();
   void init_telemetry();
   void write_FrSky8(uint8_t Data);
   void write_FrSky16(uint16_t Data);
   void check_FrSky_stuffing(uint8_t Data);
   static void sendDataHead(uint8_t Data_id);
   static void sendDataTail(void);
   void send_GPS_altitude(void);
   void send_Temperature1(void);
   void send_RPM(void);
   void send_Fuel_level(void);
   void send_Temperature2(void);
   void send_Cell_volt(void);
   void send_Altitude(void);
   void send_GPS_speed(void);
   void send_GPS_position(void);
   void send_Course(void);
   void send_Time(void);
   void send_Accel(void);
   void send_Voltage_ampere(void);
   void send_Vertical_Speed(void);
   void  send_Airspeed(void);


#endif /* FrSkyTelemtry_H_ */




 



