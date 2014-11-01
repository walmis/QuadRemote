/*
 * LiPo_Bat.hpp
 *
 *  Created on: Nov 1, 2014
 *      Author: walmis
 */

#ifndef LIPO_BAT_HPP_
#define LIPO_BAT_HPP_


class LiPo_Bat {
	const uint16_t volt_table[] = {
			3500, 3600, 3683, 3747, 3812, 3839, 3883, 3936, 3999, 4085, 4200
	};

	static uint8_t getPercent(uint8_t bat_mv) {
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
};


#endif /* LIPO_BAT_HPP_ */
