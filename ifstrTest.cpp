#include <fstream>
#include <iostream>

using namespace std;

int main()
{
  cout << "Read waypoint file." << std::endl;
  ifstream fp("pivotApproachState1.dat");

  if(fp.is_open() )
    cout << "File opened successfully." << std::endl;

  double x;
  while(!fp.eof())
    {
      fp >> x;
      cout << x << "\t";
    }

  cin.ignore();
 
  return 0;
}
