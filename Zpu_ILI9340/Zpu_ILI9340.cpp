#include "Zpu_ILI9340.h"

// Constructor when using software SPI.  All output pins are configurable.
Zpu_ILI9340::Zpu_ILI9340(uint8_t slot, uint8_t cs, uint8_t dc, uint8_t mosi, 
                         uint8_t sclk, uint8_t rst, uint8_t miso)
{
  _slot = slot;
  _cs   = cs;
  _dc   = dc;
  _mosi  = mosi;
  _miso = miso;
  _sclk = sclk;
  _rst  = rst;
}

void Zpu_ILI9340::spiwrite(uint8_t c)
{
//  SPI.transfer(c);
  USPIDATA = c;
}


void Zpu_ILI9340::writecommand(uint8_t c)
{
  digitalWrite(_dc, LOW);
  digitalWrite(_sclk, LOW);
  digitalWrite(_cs, LOW);

  spiwrite(c);

  digitalWrite(_cs, HIGH);
}


void Zpu_ILI9340::writedata(uint8_t c) {
  digitalWrite(_dc, HIGH);
  digitalWrite(_sclk, LOW);
  digitalWrite(_cs, LOW);
  
  spiwrite(c);

  digitalWrite(_cs, HIGH);
} 


void Zpu_ILI9340::begin(void) 
{
//  SPI.begin(WISHBONESLOT(_slot));
//  SPI.begin(MOSI(_mosi), MISO(_miso), SCK(_sclk));
//  SPI.setClockDivider(SPI_CLOCK_DIV4); // 8 MHz (full! speed!)
//  SPI.setBitOrder(MSBFIRST);
//  SPI.setDataMode(SPI_MODE0);

  USPICTL=BIT(SPICP1)|BIT(SPICPOL)|BIT(SPISRE)|BIT(SPIEN)|BIT(SPIBLOCK);

  pinModePPS(_mosi,HIGH);
  pinMode(_mosi, OUTPUT);
  outputPinForFunction(_mosi, IOPIN_USPI_MOSI );

  pinModePPS(_sclk,HIGH);
  pinMode(_sclk, OUTPUT);
  outputPinForFunction(_sclk, IOPIN_USPI_SCK);

  pinMode(_miso, INPUT);
  inputPinForFunction(_miso, IOPIN_USPI_MISO );

  pinModePPS(_cs, LOW);
  pinMode(_cs, OUTPUT);

  pinModePPS(_dc, LOW);
  pinMode(_dc, OUTPUT);

  pinModePPS(_rst, LOW);
  pinMode(_rst, OUTPUT);

  digitalWrite(_rst, LOW);
  digitalWrite(_sclk, LOW);
  digitalWrite(_mosi, LOW);

  // toggle RST low to reset
  digitalWrite(_rst, HIGH);
  delay(5);
  digitalWrite(_rst, LOW);
  delay(20);
  digitalWrite(_rst, HIGH);
  delay(150);

  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);

  writecommand(0xCF);  
  writedata(0x00); 
  writedata(0xC1); 
  writedata(0x30); 

  writecommand(0xED);  
  writedata(0x64); 
  writedata(0x03); 
  writedata(0x12); 
  writedata(0x81); 
 
  writecommand(0xE8);  
  writedata(0x85); 
  writedata(0x00); 
  writedata(0x78); 

  writecommand(0xCB);  
  writedata(0x39); 
  writedata(0x2C); 
  writedata(0x00); 
  writedata(0x34); 
  writedata(0x02); 
 
  writecommand(0xF7);  
  writedata(0x20); 

  writecommand(0xEA);  
  writedata(0x00); 
  writedata(0x00); 
 
  writecommand(ILI9340_PWCTR1);    //Power control 
  writedata(0x23);   //VRH[5:0] 
 
  writecommand(ILI9340_PWCTR2);    //Power control 
  writedata(0x10);   //SAP[2:0];BT[3:0] 
 
  writecommand(ILI9340_VMCTR1);    //VCM control 
  writedata(0x3e); //�Աȶȵ���
  writedata(0x28); 
  
  writecommand(ILI9340_VMCTR2);    //VCM control2 
  writedata(0x86);  //--
 
  writecommand(ILI9340_MADCTL);    // Memory Access Control 
  writedata(ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);

  writecommand(ILI9340_PIXFMT);    
  writedata(0x55); 
  
  writecommand(ILI9340_FRMCTR1);    
  writedata(0x00);  
  writedata(0x18); 
 
  writecommand(ILI9340_DFUNCTR);    // Display Function Control 
  writedata(0x08); 
  writedata(0x82);
  writedata(0x27);  
 
  writecommand(0xF2);    // 3Gamma Function Disable 
  writedata(0x00); 
 
  writecommand(ILI9340_GAMMASET);    //Gamma curve selected 
  writedata(0x01); 
 
  writecommand(ILI9340_GMCTRP1);    //Set Gamma 
  writedata(0x0F); 
  writedata(0x31); 
  writedata(0x2B); 
  writedata(0x0C); 
  writedata(0x0E); 
  writedata(0x08); 
  writedata(0x4E); 
  writedata(0xF1); 
  writedata(0x37); 
  writedata(0x07); 
  writedata(0x10); 
  writedata(0x03); 
  writedata(0x0E); 
  writedata(0x09); 
  writedata(0x00); 
  
  writecommand(ILI9340_GMCTRN1);    //Set Gamma 
  writedata(0x00); 
  writedata(0x0E); 
  writedata(0x14); 
  writedata(0x03); 
  writedata(0x11); 
  writedata(0x07); 
  writedata(0x31); 
  writedata(0xC1); 
  writedata(0x48); 
  writedata(0x08); 
  writedata(0x0F); 
  writedata(0x0C); 
  writedata(0x31); 
  writedata(0x36); 
  writedata(0x0F); 

  writecommand(ILI9340_SLPOUT);    //Exit Sleep 
  delay(120); 
  writecommand(ILI9340_DISPON);    //Display on 
}


void Zpu_ILI9340::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  writecommand(ILI9340_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART 
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9340_PASET); // Row addr set
  writedata(y0>>8);
  writedata(y0);     // YSTART
  writedata(y1>>8);
  writedata(y1);     // YEND

  writecommand(ILI9340_RAMWR); // write to RAM
}


void Zpu_ILI9340::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if((x < 0) ||(x >= ILI9340_TFTWIDTH) ||
     (y < 0) ||(y >= ILI9340_TFTHEIGHT))
  {
    return;
  }

  setAddrWindow(x,y,x+1,y+1);

  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);

  spiwrite(color >> 8);
  spiwrite(color);

  digitalWrite(_cs, HIGH);
}


void Zpu_ILI9340::fillScreen(uint16_t color)
{
  fillRect(0, 0,  ILI9340_TFTWIDTH, ILI9340_TFTHEIGHT, color);
}

// fill a rectangle
void Zpu_ILI9340::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= ILI9340_TFTWIDTH) || (y >= ILI9340_TFTHEIGHT)) return;
  if((x + w - 1) >= ILI9340_TFTWIDTH)  w = ILI9340_TFTWIDTH  - x;
  if((y + h - 1) >= ILI9340_TFTHEIGHT) h = ILI9340_TFTHEIGHT - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }

  digitalWrite(_cs, HIGH);
}

