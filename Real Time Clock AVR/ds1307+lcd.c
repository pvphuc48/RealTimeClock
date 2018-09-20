
#include <mega16.h>  
#asm
   .equ __lcd_port=0x12 ;PORTD
#endasm
#include <lcd.h>
#include <delay.h>

// I2C Bus functions 
#asm
   .equ __i2c_port=0x15 ;PORTC
   .equ __sda_bit=1
   .equ __scl_bit=0
#endasm
#include <i2c.h>

// DS1307 Real Time Clock functions
#include <ds1307.h>    
                      
#define mode1    PINA.0
#define mode    PINA.3
#define up    PINA.1
#define down    PINA.2

#define role    PORTA.4
 
#define ACK      1
#define NO_ACK   0  


// khai bao bien   
int c_g,dv_g,c_p,dv_p,c_h,dv_h,c_w,dv_w,c_y,dv_y,c_n,dv_n,c_t,dv_t;
unsigned char Hour,Min,Sec,DayofWeek,Day,Month,Year,gio=0x05,phut=0x00,giay=0x00;
unsigned char     data[7]; //mang chua gia tri thoi gian doc tu ds1307
     
     
//-------------------------------
// Read RTC
//-------------------------------
void ReadRTC(unsigned char * buff)
{
   i2c_start();
   i2c_write(0xD0);
   i2c_write(0x00);

   i2c_start();
   i2c_write(0xD1);
   *(buff+0)=i2c_read(ACK);   // Second
   *(buff+1)=i2c_read(ACK);   // Minute
   *(buff+2)=i2c_read(ACK);   // hour
   *(buff+3)=i2c_read(ACK);   // Day
   *(buff+4)=i2c_read(ACK);   // date
   *(buff+5)=i2c_read(ACK);   // month
   *(buff+6)=i2c_read(NO_ACK);   // year
   i2c_stop();
}
//-------------------------------
// Write RTC
//-------------------------------
void WriteRTC(unsigned char *buff)
{

   i2c_start();
   i2c_write(0xD0);
   i2c_write(0x00);
   i2c_write(*(buff+0));
   i2c_write(*(buff+1));
   i2c_write(*(buff+2));
   i2c_write(*(buff+3));
   i2c_write(*(buff+4));
   i2c_write(*(buff+5));
   i2c_write(*(buff+6));
   i2c_stop();
}
void hienthi(int m)
{	 
	 unsigned char chuc,donvi;
	 lcd_gotoxy(0,0);
if(m!=1){ 
	chuc = Hour/10; donvi = Hour%10;
	lcd_putchar(chuc+0x30); lcd_putchar(donvi+0x30);lcd_putsf(":");
	} else lcd_putsf("??:");
if(m!=2){       
        chuc = Min/10; donvi = Min%10;
	lcd_putchar(chuc+0x30); lcd_putchar(donvi+0x30);lcd_putsf(":");
	} else lcd_putsf("??:");
if(m!=3){         
        chuc = Sec/10; donvi = Sec%10;
	lcd_putchar(chuc+0x30); lcd_putchar(donvi+0x30);lcd_putsf("  Thu ");                     
	} else lcd_putsf("??  Thu ");
if(m!=4){         
        if(DayofWeek==1) {  lcd_putsf("CN "); } 
        else { lcd_putchar(DayofWeek+0x30); lcd_putsf(" "); }
	}  else lcd_putsf("?? ");
	lcd_gotoxy(3,1); 
if(m!=5){         
        chuc = Day/10; donvi = Day%10;
	lcd_putchar(chuc+0x30); lcd_putchar(donvi+0x30);lcd_putsf("/");
	} else lcd_putsf("??/");
if(m!=6){         
        chuc = Month/10; donvi = Month%10;
	lcd_putchar(chuc+0x30); lcd_putchar(donvi+0x30);lcd_putsf("/20");
	} else lcd_putsf("??/20"); 
if(m!=7){         
        chuc = Year/10; donvi = Year%10;
	lcd_putchar(chuc+0x30); lcd_putchar(donvi+0x30);lcd_putsf(" ");
	} else lcd_putsf("?? ");
} 
void chinhgio()
{
   int i;
   Hour = (data[2] & 15) + ((data[2]&240) >> 4) * 10; 
   Min = (data[1] & 15) + ((data[1]&240) >> 4) * 10;
   Sec = (data[0] & 15) + ((data[0]&240) >> 4) * 10;
   DayofWeek = data[3];
   Day = (data[4] & 15) + ((data[4]&240) >> 4) * 10;
   Month = (data[5] & 15) + ((data[5]&240) >> 4) * 10;
   Year = (data[6] & 15) + ((data[6]&240) >> 4) * 10;
//s_gio:
   while(1)
   {
      if (mode == 0) {while(mode==0){hienthi(0);}; goto s_phut;}
      if(i<150) hienthi(0); else {if(i<300) {hienthi(1);} else i=0; }  i++;
      if (up == 0) {while(up==0){hienthi(0);}; Hour++; if(Hour==24) Hour=0;}
      if (down == 0) {while(down==0){hienthi(0);}; Hour--; if(Hour<1) Hour=23;}
      
   }
s_phut:
   hienthi(0);
   while(1)
   {
      if (mode == 0) {while(mode==0){hienthi(0);}; goto s_giay;}
      if(i<150) hienthi(0); else {if(i<300) {hienthi(2);} else i=0; }  i++;
      if (up == 0) {while(up==0){hienthi(0);}; Min++; if(Min==60) Min=0;}
      if (down == 0) {while(down==0){hienthi(0);}; Min--; if(Min<1) Min=59;}
   }
s_giay:
   hienthi(0);
   while(1)
   {
      if (mode == 0) {while(mode==0){hienthi(0);}; goto s_thu;}
      if(i<150) hienthi(0); else {if(i<300) {hienthi(3);} else i=0; }  i++;
      if (up == 0) {while(up==0){hienthi(0);}; Sec++; if(Sec==60) Sec=0;}
      if (down == 0) {while(down==0){hienthi(0);}; Sec--; if(Sec<1) Sec=59;}
   }
   
s_thu:
   hienthi(0);
   while(1)
   {
      if (mode == 0) {while(mode==0){hienthi(0);}; goto s_ngay;}
      if(i<150) hienthi(0); else {if(i<300) {hienthi(4);} else i=0; }  i++;
      if (up == 0) {while(up==0){hienthi(0);}; DayofWeek++; if(DayofWeek==8) DayofWeek=1;}
      if (down == 0) {while(down==0){hienthi(0);}; DayofWeek--; if(DayofWeek<1) DayofWeek=7;}
   }
s_ngay:
   hienthi(0);
   while(1)
   {
      if (mode == 0) {while(mode==0){hienthi(0);}; goto s_thang;}
      if(i<150) hienthi(0); else {if(i<300) {hienthi(5);} else i=0; }  i++;
      if (up == 0) {while(up==0){hienthi(0);}; Day++; if(Day==32) Day=1;}
      if (down == 0) {while(down==0){hienthi(0);}; Day--; if(Day<1) Day=31;}
   }
s_thang:
   hienthi(0);
   while(1)
   {
      if (mode == 0) {while(mode==0){hienthi(0);};goto s_nam;}
      if(i<150) hienthi(0); else {if(i<300) {hienthi(6);} else i=0; }  i++;
      if (up == 0) {while(up==0){hienthi(0);}; Month++; if(Month==13) Month=1;}
      if (down == 0) {while(down==0){hienthi(0);}; Month--; if(Month<1) Month=12;}
   }
s_nam:
   hienthi(0);
   while(1)
   {
      if (mode == 0) {while(mode==0){hienthi(0);}; break;}
      if(i<150) hienthi(0); else {if(i<300) {hienthi(7);} else i=0; }  i++;
      if (up == 0) {while(up==0){hienthi(0);}; Year++; if(Year==100) Year=0;}
      if (down == 0) {while(down==0){hienthi(0);}; Year--; if(Year<1) Year=99;}
   }
	   
// ghi lai cac gia tri cai dat
        data[0] = Sec%10 + ((Sec/10)<<4); data[1] = Min%10 + ((Min/10)<<4); 
        data[2] = Hour%10 + ((Hour/10)<<4); data[3] = DayofWeek%10 + ((DayofWeek/10)<<4); 
        data[4] = Day%10 + ((Day/10)<<4); data[5] = Month%10 + ((Month/10)<<4); 
        data[6] = Year%10 + ((Year/10)<<4); 
        WriteRTC(&data[0]);
} 
void hienthi1(int m)
{	 
	 unsigned char chuc,donvi;
	 lcd_gotoxy(0,0);
if(m!=1){ 
	chuc = Hour/10; donvi = Hour%10;
	lcd_putchar(chuc+0x30); lcd_putchar(donvi+0x30);lcd_putsf(":");
	} else lcd_putsf("??:");
if(m!=2){       
        chuc = Min/10; donvi = Min%10;
	lcd_putchar(chuc+0x30); lcd_putchar(donvi+0x30);lcd_putsf(":");
	} else lcd_putsf("??:");
if(m!=3){         
        chuc = Sec/10; donvi = Sec%10;
	lcd_putchar(chuc+0x30); lcd_putchar(donvi+0x30);lcd_putsf("        ");                     
	} else lcd_putsf("??        ");
} 
void chinhbaothuc()
{     
    int i;
        lcd_gotoxy(0,1);
        lcd_putsf(" CHINH BAO THUC ");
       
        lcd_gotoxy(0,0); 
   Hour = (gio & 15) + ((gio&240) >> 4) * 10; 
   Min = (phut & 15) + ((phut&240) >> 4) * 10;
   Sec = (giay & 15) + ((giay&240) >> 4) * 10;
//s_gio:
   while(1)
   {
      if (mode1 == 0) {while(mode1==0){hienthi1(0);}; goto s_phut;}
      if(i<150) hienthi1(0); else {if(i<300) {hienthi1(1);} else i=0; }  i++;
      if (up == 0) {while(up==0){hienthi1(0);}; Hour++; if(Hour==24) Hour=0;}
      if (down == 0) {while(down==0){hienthi1(0);}; Hour--; if(Hour<1) Hour=23;}
      
   }
s_phut:
   hienthi1(0);
   while(1)
   {
      if (mode1 == 0) {while(mode1==0){hienthi1(0);}; goto s_giay;}
      if(i<150) hienthi1(0); else {if(i<300) {hienthi1(2);} else i=0; }  i++;
      if (up == 0) {while(up==0){hienthi1(0);}; Min++; if(Min==60) Min=0;}
      if (down == 0) {while(down==0){hienthi1(0);}; if(Min>0) Min--;  else Min=59;}
   }
s_giay:
   hienthi1(0);
   while(1)
   {
      if (mode1 == 0) {while(mode1==0){hienthi1(0);}; break;}
      if(i<150) hienthi1(0); else {if(i<300) {hienthi1(3);} else i=0; }  i++;
      if (up == 0) {while(up==0){hienthi1(0);}; Sec++; if(Sec==60) Sec=0;}
      if (down == 0) {while(down==0){hienthi1(0);};  if(Sec>0)Sec--; else Sec=59;}
   }     
   giay = Sec%10 + ((Sec/10)<<4); phut = Min%10 + ((Min/10)<<4); 
        gio = Hour%10 + ((Hour/10)<<4);    
} 
void convert_bcd_lcd(int g,int p, int h,int w,int n,int t,int y)
  {
   c_g=(g&240)>>4;   //chia lay phan du, so hang don vi
   dv_g=g&15;   //tach hang tram va hang chuc
   c_g = c_g + 0x30;
   dv_g = dv_g + 0x30;
   c_p=(p%240)>>4;  //chia lay phan du, so hang don vi
   dv_p=p&15;   //tach hang tram va hang chuc
   c_p = c_p + 0x30;
   dv_p = dv_p + 0x30;
   c_h=(h&240)>>4;  //chia lay phan du, so hang don vi
   dv_h=h&15;   //tach hang tram va hang chuc
   c_h = c_h + 0x30;
   dv_h = dv_h + 0x30;
   c_w=(w&240)>>4;  //chia lay phan du, so hang don vi
   dv_w=w&15;   //tach hang tram va hang chuc
   c_w = c_w + 0x30;
   dv_w = dv_w + 0x30;
   c_n=(n&240)>>4;  //chia lay phan du, so hang don vi
   dv_n=n&15;   //tach hang tram va hang chuc
   c_n = c_n + 0x30;
   dv_n = dv_n + 0x30;
   c_t=(t&240)>>4;  //chia lay phan du, so hang don vi
   dv_t=t&15;   //tach hang tram va hang chuc
   c_t = c_t + 0x30;
   dv_t = dv_t + 0x30;
   c_y=(y&240)>>4;  //chia lay phan du, so hang don vi
   dv_y=y&15;   //tach hang tram va hang chuc
   c_y = c_y + 0x30;
   dv_y = dv_y + 0x30;
   
  }
void main(void)
{
PORTA=0x00;
DDRA=0xF0;
PORTB=0x00;
DDRB=0xFF;
PORTC=0x00;
DDRC=0xFF;
PORTD=0x00;
DDRD=0xFF;

lcd_init(16);
// I2C Bus initialization
i2c_init();

// DS1307 Real Time Clock initialization
rtc_init(0,0,0);
lcd_clear();
role = 1;
while (1)
      {  
        ReadRTC(&data[0]); 
        convert_bcd_lcd(data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
            lcd_gotoxy(0,0);
            lcd_putchar(c_h); lcd_putchar(dv_h);lcd_putsf(":");
            lcd_putchar(c_p); lcd_putchar(dv_p);lcd_putsf(":");
            lcd_putchar(c_g); lcd_putchar(dv_g);lcd_putsf("  Thu ");  
            if(data[3]==1)  lcd_putsf("CN"); else {  lcd_putchar(dv_w); lcd_putsf(" ");  }
            lcd_gotoxy(0,1);  lcd_putsf("   ");
            lcd_putchar(c_n); lcd_putchar(dv_n);lcd_putsf("/");
            lcd_putchar(c_t); lcd_putchar(dv_t);lcd_putsf("/20");
            lcd_putchar(c_y); lcd_putchar(dv_y);lcd_putsf("   ");

         if(mode==0) { while(mode==0){} chinhgio(); }
         if(mode1==0) { while(mode1==0){} chinhbaothuc(); }
         
         if(gio==data[2] && phut==data[1] && giay==data[0]) role = 0;
      };
}
