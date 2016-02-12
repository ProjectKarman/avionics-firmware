/*
 * hmc5883l.c
 *
 * Created: 2/11/2016 9:52:19 PM
 *  Author: Andrew Kaster
 */ 

 #include "hmc5883l.h"
 #include <stdbool.h>
 #include <stdint.h>
 #include <string.h>

 #include "asf.h"
 #include "FreeRTOS.h"
 #include "queue.h"
 #include "semphr.h"
 #include "task.h"

