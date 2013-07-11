#include <iostream>
#include <fstream>

#include "nittaFS.h"

extern "C" {
#include <unistd.h>
#include <stdio.h>
}

int main(){

  std::cout << "fs_test" << std::endl;

  FILE *fp;
  if((fp=fopen("/home/hrpuser/forceSensorPlugin/server/test.dat","w"))==NULL){
    std::cout<< "file open error!!" << std::endl;
    return 0;
  }

  nittaFS *fs1;
  //int offsets[6] = {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
  //int unit_no;
  //double units[3];

  fs1 = new nittaFS();
  fs1->init();
  //fs1->set_offset(offsets);
  //std::cout << "unit_no: " << fs1->get_units() << std::endl;
  //fs1->reset_offset();

  for(int n=0; n<2; n++){
      double fs_data[6];
      double fs_tmp[6] = {0.0,};
      double fs_offset[6] = {0.0,};

    for(int i=0; i<1000; i++){
      fs1->get_forces(n, fs_data);

      //std::cout << "fs_data [" << i << "]";
      for (int j=0; j<6; j++) {
        //std::cout << " " << fs_data[j];
        fs_tmp[j] += fs_data[j];
      }

      fprintf(fp,"[%d] %f %f %f %f %f %f\n",i, fs_data[0],fs_data[1],fs_data[2],fs_data[3],fs_data[4],fs_data[5]);
      //std::cout << "fs_tmp: " << i << " " << fs_tmp[0] << " " << fs_tmp[1] << " " << fs_tmp[2] << " " << fs_tmp[3] << " " << fs_tmp[4] << " " << fs_tmp[5] << std::endl;
      //usleep(1000);
      //std::cout << std::endl;
    }
    std::cout << "fs_offset:";
    for(int j=0; j<6; j++) {
      //std::cout << " " << fs_data[j];
      fs_offset[j] = fs_tmp[j]/1000;
      fs_tmp[j] = 0.0;
      std::cout << " " << fs_offset[j];
    }
    std::cout << std::endl;

    for(int i=0; i<10000; i++){
      fs1->get_forces(n, fs_data);
      //if((i%100) == 0)
        //std::cout << "[]"<< " " << fs_data[0] << " " << fs_data[1] << " " << fs_data[2] << " " << fs_data[3] << " " << fs_data[4] << " " << fs_data[5] << std::endl;

      //std::cout << "fs_data [" << i << "]";
      for (int j=0; j<6; j++) {
        //std::cout << " " << fs_data[j];
        fs_data[j] = fs_data[j] - fs_offset[j];
        if (fs_data[j] < 0 ) fs_tmp[j] += -fs_data[j];
        else fs_tmp[j] += fs_data[j];
      }

      fprintf(fp,"[%d] %f %f %f %f %f %f\n",i+1000, fs_data[0],fs_data[1],fs_data[2],fs_data[3],fs_data[4],fs_data[5]);
      //printf("[%d] %f %f %f %f %f %f\n",i+1000, fs_data[0],fs_data[1],fs_data[2],fs_data[3],fs_data[4],fs_data[5]);
      if((i%100) == 0)
        printf("[%d] %f %f %f %f %f %f\n",i+1000, fs_data[0],fs_data[1],fs_data[2],fs_data[3],fs_data[4],fs_data[5]);
        //std::cout << i+ 1000 << " " << fs_data[0] << " " << fs_data[1] << " " << fs_data[2] << " " << fs_data[3] << " " << fs_data[4] << " " << fs_data[5] << std::endl;
      usleep(1000);
      //std::cout << std::endl;
    }

    for(int j=0; j<6; j++) {
      fs_tmp[j] = fs_tmp[j]/10000;
    }
    //fprintf(fp,"[###] %f %f %f %f %f %f\n\n\n\n", fs_tmp[0],fs_tmp[1],fs_tmp[2],fs_tmp[3],fs_tmp[4],fs_tmp[5]);
    printf("sensor 1\n");
  }

  fclose(fp);
  return 1;

}
