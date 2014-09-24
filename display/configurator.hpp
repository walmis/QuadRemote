/*
 * configurator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#ifndef CONFIGURATOR_HPP_
#define CONFIGURATOR_HPP_

class Configurator {
public:
	virtual void init(uint8_t index) {};
	virtual void populate(LCDLine* lines, uint8_t nLines) = 0;
	virtual void nextValue() = 0;
	virtual void prevValue() = 0;
};

#endif /* CONFIGURATOR_HPP_ */
