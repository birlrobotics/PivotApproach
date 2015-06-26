#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>
using namespace std;

#define GRAV_COMP_FILE "gravCompParams.dat"
typedef vector< vector<double> > vec2; 

bool extract_SingleItem(ifstream& in, vector<double>& first, vector<double>& second, int size)
{
  // Local variables
  string title;
  string data;
  double temp;
  int len = 0; // to fill in array of incoming pointer

  // Verify size: expecting 2 or 6. 
  if((size > 1) || (size < 7))
    {
      if(size==2)
	len=1;
      else if(size==6)
	len=3;
      else
	return false; 
    }
    
  // Extract average values
  // Average
  in >>  title; // This first extraction is for the title. It is discarded.

  // Now extract actual values.
  for (int i=0; i<size; i++)
    {
      in >>  data; 

      // check for -nan
      if(strcmp(data.c_str(),"nan")==0 || strcmp(data.c_str(),"-nan")==0)
	data="0\0"; // Set the value to zero and end with null

      if(i<len)
	{
	  temp=strtod(data.c_str(),NULL);	   
	  first[i]=temp;
	}
      else
	{
	  temp=strtod(data.c_str(),NULL);
	  second[i-len]=temp;
	}
    }
  
   return true;
}

// Same but for arrays
bool extract_SingleItem(ifstream& in, double* first, double* second, int size)
{
  // Local variables
  string title;
  string data;
  int len = 0; // to fill in array of incoming pointer

  // Verify size: expecting 2 or 6. 
  if((size > 1) || (size < 7))
    {
      if(size==2)
	len=1;
      else if(size==6)
	len=3;
      else
	return false; 
    }
    
  // Extract average values
  // Average
  in >> title; // This first extraction is for the title. It is discarded.

  // Now extract actual values.
  for (int i=0; i<size; i++)
    {
      in >>  data; 

      // check for -nan
      if(strcmp(data.c_str(),"nan")==0 || strcmp(data.c_str(),"-nan")==0)
	data="0\0"; // Set the value to zero and end with null

      if(i<len)
	  first[i]=strtod(data.c_str(),NULL);
      else
	  second[i]=strtod(data.c_str(),NULL);
    }
  
   return true;
}

int main(void)
{
  int ret=0;
  int index=0;
  char command[10];
  vec2 F_vec(2, vector<double>(3));
  vec2 M_vec(2, vector<double>(3));
  double F_ave[2][3]={{0,0,0},{0,0,0}};
  double M_ave[2][3]={{0,0,0},{0,0,0}};
  double F_sd[2][3]={{0,0,0},{0,0,0}};
  double M_sd[2][3]={{0,0,0},{0,0,0}};

  // Create input stream
  ifstream istr_gravCompParam(GRAV_COMP_FILE,ios::in);
  if(!istr_gravCompParam) return false;

  // Extract Arm Index on first line.
  istr_gravCompParam.getline(command,10,'\n');

  // Check for first loop: Left Sensor
  if (!strcmp(command, "sensor 0:"))
    index=0; // Left Arm
  else
    index=1; // Right arm

  // Extract data for four pairs: F_ave/M_ave, F_sd/M_sd, F_error_max/M_error_max; 
  while(!istr_gravCompParam.eof())
    {
      ret= extract_SingleItem(istr_gravCompParam, F_vec[index], M_vec[index], 6); 
      if(ret)
	ret= extract_SingleItem(istr_gravCompParam, F_sd[index], M_sd[index], 6); 
      
    }
  return 0;
}
