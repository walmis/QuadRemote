/*
 * LiPo_Bat.hpp
 *
 *  Created on: Nov 1, 2014
 *      Author: walmis
 */

#ifndef LIPO_BAT_HPP_
#define LIPO_BAT_HPP_

#include <stdint.h>

class LiPo_Bat {
public:
	static const uint16_t volt_table[];

	static int getPercent(uint16_t bat_mv);
};


#endif /* LIPO_BAT_HPP_ */
