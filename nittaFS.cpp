/**
 * @file   nittaFS.cpp
 * @brief  Nitta force moment sensor class
 * @author
 * @date   2011/02/16
 * @note
 */
/*
 * ResMgr and Message Server Process
 *
 * This program is for JR3/Nitta Force moment sensor.
 * Copyright(C) by Waseda University, Nitta Coropration. 2002.
 *  modified by n-yamanobe 2011/02/16
 */

#include "nittaFS.h"

#ifdef SIMULATION

nittaFS::nittaFS()
{
}

nittaFS::~nittaFS()
{
}

#else

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/time.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/mman.h>
#include <hw/pci.h>
#include "jr3pci3.idm"

#define VENDORID        0x1762  /* Vendor ID of JR3 */
// #define DEVICEID     0x3114  /* for 4ch board */
// #define DEVICEID     0x3113  /* for 3ch board */
#define DEVICEID        0x3112  /* for 2ch board */
// #define DEVICEID     0x1111  /* for 1ch board */

#define Jr3ResetAddr    0x18000
#define Jr3NoAddrMask   0x40000
#define Jr3DmAddrMask   0x6000
#define PAGE_SIZE       0x1000

#define SENSOR0         0
#define SENSOR1         0x80000
#define SENSOR2         0x100000
#define SENSOR3         0x180000

/* Convert a DSP bus address to a PCI bus address */
#define ToJr3PciAddrH(addr)	(((int)(addr) << 2) + Jr3BaseAddressH)
#define ToJr3PciAddrL(addr)	(((int)(addr) << 2) + Jr3BaseAddressL)

/* Read and Write from the program memory */
#define ReadJr3Pm(addr) 	(*(uint16_t volatile *)(ToJr3PciAddrH((addr))) << 8 |\
		 *(uint8_t  volatile *)(ToJr3PciAddrL((addr))))

#define WriteJr3Pm2(addr,data,data2)       (*(int volatile *)ToJr3PciAddrH((addr)) = (int)(data));\
				   (*(int volatile *)ToJr3PciAddrL((addr)) = (int)(data2))

#define WriteJr3Pm(addr,data)              WriteJr3Pm2((addr),(data) >> 8, (data))

/* Read and Write from the data memory using bus relative addressing */
#define ReadJr3Dm(addr)	(*(uint16_t volatile *)(ToJr3PciAddrH((addr))))
#define WriteJr3Dm(addr,data)	*(int volatile *)(ToJr3PciAddrH((addr))) = (int)(data)

/* Read and Write from the data memory using 0 relative addressing */
#define ReadJr3(addr) (ReadJr3Dm((addr) | Jr3DmAddrMask))
#define WriteJr3(addr,data) WriteJr3Dm((addr) | Jr3DmAddrMask,data)

nittaFS::nittaFS()
{
  for(int i=0; i<2; i++){
	for(int j=0; j<6; j++){
	  m_offsets[i][j] = 0x0000;
	}
  }
}
nittaFS::~nittaFS()
{
}

bool nittaFS::init()
{
  unsigned bus, function, index = 0;
  unsigned devid, venid;
  int result;
  unsigned lastbus, version, hardware;
  unsigned int address0;
  volatile uint32_t Jr3BaseAddressH;
  //volatile uint32_t Jr3BaseAddressL;
  volatile uint32_t Jr3BaseAddress0H;
  volatile uint32_t Jr3BaseAddress0L;
  volatile uint32_t Jr3BaseAddress1H;
  volatile uint32_t Jr3BaseAddress1L;
  char bufptr[32];

  pci_attach(0);
  result = pci_present(&lastbus, &version, &hardware);
  if (result == -1){
	printf("PCI BIOS not present!\n");
	return false;
  }
  devid = DEVICEID;
  venid = VENDORID;

  result = pci_find_device(devid, venid, index, &bus, &function);
  pci_read_config_bus(bus, function, 0x10, 1, sizeof(unsigned int), (void *)&bufptr);
  address0 = *(unsigned int *)bufptr;

  Jr3BaseAddress0H = (volatile uint32_t)mmap_device_memory(NULL, 0x100000, PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, address0);
  Jr3BaseAddress0L = (volatile uint32_t)mmap_device_memory(NULL, 0x100000, PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, address0 + Jr3NoAddrMask);
  Jr3BaseAddressH = (volatile uint32_t)mmap_device_memory(NULL, 0x100000, PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, address0);  //  <- WriteJr3Dmのために必要
  WriteJr3Dm(Jr3ResetAddr, 0); // Reset DSP
  usleep(3000000);

  download(Jr3BaseAddress0H, Jr3BaseAddress0L);
  usleep(3000000);

  MappedAddress0 = Jr3BaseAddress0H;

  Jr3BaseAddress1H = (volatile uint32_t)mmap_device_memory(NULL, 0x100000, PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, address0 + 0x80000);
  Jr3BaseAddress1L = (volatile uint32_t)mmap_device_memory(NULL, 0x100000, PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, address0 + 0x80000 + Jr3NoAddrMask);

  download(Jr3BaseAddress1H, Jr3BaseAddress1L);
  usleep(3000000);

  MappedAddress1 = Jr3BaseAddress1H;

  // initialize force sensor;
  // for sensor 0
  set_offset(0, m_offsets[0]);
  m_unit_no[0] = get_units(0, m_units[0]);

  // for sensor 1
  set_offset(1, m_offsets[1]);
  m_unit_no[1] = get_units(1, m_units[1]);

  //std::cout << "unit_no: " << m_unit_no << std::endl;

  // for fs deviation test
  //~ FILE *fp_nitta;
  //~ std::string f_name1 = "/home/hrpuser/yamanobe/data/fs_nitta_init.dat";
  //~ if((fp_nitta=fopen(f_name1.c_str(),"w"))==NULL){
	//~ std::cerr<< "file open error!!" << std::endl;
  //~ }
  //~ for(int i=0; i<2000; i++){

	//~ double fout[6];
	//~ get_forces(fout);
	//~ fprintf(fp_nitta,"%f %f %f %f %f %f\n",fout[0],fout[1],fout[2],fout[0],fout[1],fout[2]);

  //~ }
  return true;
}

int nittaFS::get_units(int No, double units[3]){

  unsigned long MappedAddress;
  if(No == 0)			MappedAddress = MappedAddress0;
  else if(No == 1)		MappedAddress = MappedAddress1;
  else{
	std::cerr << "nittaFS ERROR: inputted No is more than 1." << std::endl;
	MappedAddress = MappedAddress0;
  }

  int unit_no = IFS_Read(MappedAddress, 0xfc);
  switch (unit_no){
  case 0:
	// pound, inch*pound, inch*1000
	units[0] = 0.0;
	units[1] = 0.0;
	units[2] = 0.0;
	std::cout << "unit_no is 0 (pound, inch*pound, inch*1000)" << std::endl;
	break;
  case 1:
	// N, Nm*10, mm*10
	//printf("force unit 1\n");
	units[0] = 1.0;  // [N]
	units[1] = 0.1;  // [Nm]
	units[2] = 0.01; // [m]
	std::cout << "unit_no is 1 (N, Nm*10, mm*10)" << std::endl;
	break;
  case 2:
	// kgf*10, kgf*cm, mm*10
	//printf("force unit 2\n");
	units[0] = 9.8*0.1;  // [N]
	units[1] = 9.8*0.01; // [Nm]
	units[2] = 0.01;     // [m]
	std::cout << "unit_no is 2 (kgf*10, kgf*cm, mm*10)" << std::endl;
	break;
  case 3:
	// kilopound, kiloinch*pound, inch*1000
	units[0] = 0.0;
	units[1] = 0.0;
	units[2] = 0.0;
	std::cout << "unit_no is 3 (kilopound, kiloinch*pound, inch*1000)" << std::endl;
	break;
  default:
	std::cout << "force unit error !\n" << std::endl;
	units[0] = 0.0;
	units[1] = 0.0;
	units[2] = 0.0;
	break;
  }
  return unit_no;
}

void nittaFS::set_offset(int No, int offsets[6])
{
  unsigned long MappedAddress;
  if(No == 0)			MappedAddress = MappedAddress0;
  else if(No == 1)		MappedAddress = MappedAddress1;
  else{
	std::cerr << "nittaFS ERROR: inputted No is more than 1." << std::endl;
	MappedAddress = MappedAddress0;
  }

  for (int i=0;i<6;i++) IFS_Write(MappedAddress, 0x88+i, offsets[i]);
  IFS_Write(MappedAddress, 0x00e7, 0x0700);
  usleep(1000);

}
void nittaFS::reset_offset(int No)
{
  unsigned long MappedAddress;
  if(No == 0)			MappedAddress = MappedAddress0;
  else if(No == 1)		MappedAddress = MappedAddress1;
  else{
	std::cerr << "nittaFS ERROR: inputted No is more than 1." << std::endl;
	MappedAddress = MappedAddress0;
  }
  //for (int i=0;i<6;i++) IFS_Write(MappedAddress, 0x88+i, offsets[i]);
  IFS_Write(MappedAddress, 0x00e7, 0x0800);
  usleep(1000);

}

void nittaFS::get_offset(int No, int offsets[6])
{
  unsigned long MappedAddress;
  if(No == 0)			MappedAddress = MappedAddress0;
  else if(No == 1)		MappedAddress = MappedAddress1;
  else{
	std::cerr << "nittaFS ERROR: inputted No is more than 1." << std::endl;
	MappedAddress = MappedAddress0;
  }
  for (int i=0;i<6;i++) offsets[i] = IFS_Read(MappedAddress, 0x88+i);
}

void nittaFS::get_forces(int No, double forces[6])
{
  unsigned long MappedAddress;
  int num = No;
  if(num == 0)			MappedAddress = MappedAddress0;
  else if(num == 1)	MappedAddress = MappedAddress1;
  else{
	std::cerr << "nittaFS ERROR: inputted No is more than 1." << std::endl;
	num = 0;
	MappedAddress = MappedAddress0;
  }

  short max_val[6],real_val[6],max_addr[6],real_addr[6];

  for(int k=0;k<6;++k){
	max_addr[k] = (0x80+k);
	real_addr[k] = (0xa0+k);
	max_val[k] = IFS_Read(MappedAddress, max_addr[k]);      //Maximum Default V$
	real_val[k] = IFS_Read(MappedAddress, real_addr[k]);    //Measured Force $
  }

  for(int k=0; k<3; ++k){
	forces[k]   = (double)max_val[k] / 16384.0 * (double)real_val[k] * m_units[num][0];     //Fx,Fy,Fz
	forces[k+3] = (double)max_val[k+3] / 16384.0 * (double)real_val[k+3] * m_units[num][1]; //Mx,My,Mz
  }

  #ifdef DEBUG_PLUGIN2
  //#if 1
  std::cout << "nitta fs: " << forces[0] <<" " << forces[1] <<" " << forces[2] <<" " << forces[3] <<" " << forces[4] <<" " << forces[5] << std::endl;
  #endif
}

short nittaFS::IFS_Read(unsigned long MappedAddress, unsigned int addr)
{
	  short data = 0;
	  unsigned long address = 0;

	  address = MappedAddress + ((addr | Jr3DmAddrMask) << 2);
	  data = *(short *)address;
	  return data;
}

void nittaFS::IFS_Write(unsigned long MappedAddress, unsigned int addr, int data)
{
	  unsigned long address = 0;

	 address = MappedAddress + ((addr | Jr3DmAddrMask) << 2);
	  *(int *)address = data;
}

int nittaFS::download(unsigned int base0, unsigned int base1)
{
   int count;
   int index = 0;
   unsigned int Jr3BaseAddressH;
   unsigned int Jr3BaseAddressL;

   Jr3BaseAddressH = base0;
   Jr3BaseAddressL = base1;

   /* The first line is a line count */
   count = dsp[index++];

   /* Read in file while the count is no 0xffff */
   while (count != 0xffff){
	 int addr;
	 /* After the count is the address */
	 addr = dsp[index++];
	 /* loop count times and write the data to the dsp memory */
	 while (count > 0){
	   /* Check to see if this is data memory or program memory */
	   if (addr & 0x4000){
	 int data = 0;
	 /* Data memory is 16 bits and is on one line */
	 data = dsp[index++];
	 WriteJr3Dm(addr, data);
	 count--;
	 if (data != ReadJr3Dm(addr)){
	   printf("data addr: %4.4x out: %4.4x in: %4.4x\n", addr, data, ReadJr3Dm(addr));
	 }
	   }
	   else{
	 int data, data2;
	 int data3;
	 /* Program memory is 24 bits and is on two lines */
	 data = dsp[index++];
	 data2 = dsp[index++];
	 WriteJr3Pm2(addr, data, data2);
	 count -= 2;

	 /* Verify the write */
	 if (((data << 8) | (data2 & 0xff)) != (data3 = ReadJr3Pm(addr))){
	   //printf("pro addr: %4.4x out: %6.6x in: %6.6x\n", addr, ((data << 8)|(data2 & 0xff)), /* ReadJr3Pm(addr) */ data3);
	 }
	   }
	   addr++;
	 }
	 count = dsp[index++];
   }

   return 0;
}


#endif
