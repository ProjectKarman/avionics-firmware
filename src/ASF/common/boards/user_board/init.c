/**
 * \file
 *
 * \brief User board initialization template
 *
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>
#include "nrf24l01p.h"
#include "ms5607_02ba_dev.h"

void board_init(void)
{
  /* This function is meant to contain board-specific initialization code
	 * for, e.g., the I/O pins. The initialization can rely on application-
	 * specific board configuration, found in conf_board.h.
	 */
  pmic_init();
  sysclk_init();
  ioport_init();
  dma_enable();
  nrf24l01p_init();
  // ms5607_02ba_init();
}
