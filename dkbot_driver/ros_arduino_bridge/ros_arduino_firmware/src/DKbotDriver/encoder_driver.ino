#include <Encoder.h>


//Encoder leftEncoder(18,19);
//Encoder rightEncoder(21,20); Encoder for 1:30 motor (old)

Encoder leftEncoder(19,18);
Encoder rightEncoder(20,21);




long leftPosition;
long rightPosition;


long readEncoder(int i)
{
  if (i==LEFT)
  {
   
    return leftEncoder.read()-leftPosition;
  }
  else
  {
     return rightEncoder.read()-rightPosition;
  }
  
  
}
void resetEncoder(int i)
{
  if (i==LEFT)
  {
    leftPosition=leftEncoder.read();
    
  }else
  {
    rightPosition=rightEncoder.read();
  }
}
void resetEncoders()
{
  
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
  
  
}




