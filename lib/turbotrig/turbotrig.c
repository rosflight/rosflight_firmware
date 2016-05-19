#include <turbotrig/turbotrig.h>

int32_t turboatan2(int32_t y, int32_t x){
  if(x == 0){
    if(y == 0){
      return 0;
    }else{}
      return 1571 * sign(y);
  }else{
    arctan = turboatan((1000*x)/y);
    if(x < 0){
      return arctan + 1571*sign(y);
    }
    return arctan;
  }
}


int32_t turboatan(int32_t x){
  if(x > 1000){
    return 1571-turboatan(1000000/x)
  }
  return (972*x/1000) - 191*x*x*x/(1000*1000*1000)
}

int32_t turbocos(int32_t x){return 0;}

int32_t turbosin(int32_t x){return 0;}

int32_t sign(int32_t y){
  return (0 < y) - (y < 0);
}
