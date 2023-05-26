#ifndef ZS05_H
#define ZS05_H

#include "main.h"

typedef struct
{
	int8_t temperature;
	uint8_t humidity;
}ZS05_Data;


//public function
uint8_t ZS05_Get_Temperature(ZS05_Data *zs05_data);

#endif