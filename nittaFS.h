/**
 * @file   nittaFS.h
 * @brief  Nitta force moment sensor class
 * @author
 * @date   2011/02/16
 * @note
 */

/**
 * ResMgr and Message Server Process
 *
 * This program is for JR3/Nitta Force moment sensor.
 * Copyright(C) by Waseda University, Nitta Coropration. 2002.
 *  modified by n-yamanobe 2011/02/16
 */

#ifndef NITTA_FS_H
#define NITTA_FS_H

#include <iostream>

class nittaFS
{
	public:
		nittaFS();
		virtual ~nittaFS();
#ifndef SIMULATION
		bool init();
		int get_units(int No, double units[3]);
		void set_offset(int No, int offsets[6]);
		void get_offset(int No, int offsets[6]);
		void reset_offset(int No);

		void get_forces(int No, double forces[6]);

	protected:
		//int m_No;
		unsigned long MappedAddress0, MappedAddress1;
		int m_offsets[2][6];
		int m_unit_no[2];
		double m_units[2][6];

		short IFS_Read(unsigned long MappedAddress, unsigned int addr);
		void IFS_Write(unsigned long MappedAddress, unsigned int addr, int data);
		int download(unsigned int base0, unsigned int base1);
#endif
};

#endif
