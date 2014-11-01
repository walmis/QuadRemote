/*
 * LiPo_Bat.cpp
 *
 *  Created on: Nov 1, 2014
 *      Author: walmis
 */

#include "LiPo_Bat.hpp"

const uint16_t LiPo_Bat::volt_table[] = {
		3500, 3600, 3683, 3747, 3812, 3839, 3883, 3936, 3999, 4085, 4200
};

int LiPo_Bat::getPercent(uint16_t bat_mv) {
	if (bat_mv <= volt_table[0]) {
		return 0;
	} else if (bat_mv >= volt_table[10]) {
		return 100;
	} else {
		/* search nearest value */
		int i = 0;
		while (i < 10 && volt_table[i + 1] < bat_mv)
			i++;
		/* interpolate linear between the smaller and greater value */
		/* Tens digit, 10% per entry, ones digit: interpolated */
		return i * 10 + (bat_mv - volt_table[i]) * 10 / (volt_table[i + 1] - volt_table[i]);
	}
}
