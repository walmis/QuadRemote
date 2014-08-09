/*
 * MMA8451Q.hpp
 *
 *  Created on: Apr 5, 2014
 *      Author: walmis
 */

#ifndef MMA8451Q_HPP_
#define MMA8451Q_HPP_

#include <xpcc/architecture/peripheral/i2c_adapter.hpp>
#include <xpcc/math/geometry.hpp>

#define MMA8451_REG_STATUSR             0x00
#define MMA8451_REG_OUT_X_MSB           0x01
#define MMA8451_REG_OUT_X_MSB           0x01
#define MMA8451_REG_OUT_X_LSB           0x02
#define MMA8451_REG_OUT_Y_MSB           0x03
#define MMA8451_REG_OUT_Y_LSB           0x04
#define MMA8451_REG_OUT_Z_MSB           0x05
#define MMA8451_REG_OUT_Z_LSB           0x06
//Reserved                              0x07
//Reserved                              0x08
#define MMA8451_REG_F_SETUP             0x09
#define MMA8451_REG_TRIG_CFG            0x0A
#define MMA8451_REG_SYSMOD              0x0B
#define MMA8451_REG_INT_SOURCE          0x0C
#define MMA8451_REG_WHO_AM_I            0x0D
#define MMA8451_REG_XYZ_DATA_CFG        0x0E
#define MMA8451_REG_HP_FILTER_CUTOFF    0x0F
#define MMA8451_REG_PL_STATUS           0x10
#define MMA8451_REG_PL_CFG              0x11
#define MMA8451_REG_PL_COUNT            0x12
#define MMA8451_REG_PL_BF_ZCOMP         0x13
#define MMA8451_REG_PL_THS_REG          0x14
#define MMA8451_REG_FF_MT_CFG           0x15
#define MMA8451_REG_FF_MT_SRC           0x16
#define MMA8451_REG_FF_MT_THS           0x17
#define MMA8451_REG_FF_MT_COUNT         0x18
//Reserved                              0x19
//Reserved                              0x1A
//Reserved                              0x1B
//Reserved                              0x1C
#define MMA8451_REG_TRANSIENT_CFG       0x1D
#define MMA8451_REG_TRANSIENT_SRC       0x1E
#define MMA8451_REG_TRANSIENT_THS       0x1F
#define MMA8451_REG_TRANSIENT_COUNT     0x20
#define MMA8451_REG_PULSE_CFG           0x21
#define MMA8451_REG_PULSE_SRC           0x22
#define MMA8451_REG_PULSE_THSX          0x23
#define MMA8451_REG_PULSE_THSY          0x24
#define MMA8451_REG_PULSE_THSZ          0x25
#define MMA8451_REG_PULSE_TMLT          0x26
#define MMA8451_REG_PULSE_LTCY          0x27
#define MMA8451_REG_PULSE_WIND          0x28
#define MMA8451_REG_ASLP_COUNT          0x29
#define MMA8451_REG_CTRL_REG1           0x2A
#define MMA8451_REG_CTRL_REG2           0x2B
#define MMA8451_REG_CTRL_REG3           0x2C
#define MMA8451_REG_CTRL_REG4           0x2D
#define MMA8451_REG_CTRL_REG5           0x2E
#define MMA8451_REG_OFF_X               0x2F
#define MMA8451_REG_OFF_Y               0x30
#define MMA8451_REG_OFF_Z               0x31
//Reserved                              0x40
//                                      ....
//Reserved                              0x7F


//ZYXOW is set whenever a new acceleration data is produced before completing the
//retrieval of the previous set. This event occurs when the content of at least
//one acceleration data register (i.e., OUT_X, OUT_Y, OUT_Z) has been.overwritten.
//ZYXOW is cleared when the high-bytes of the acceleration data (OUT_X_MSB,
//OUT_Y_MSB, OUT_Z_MSB) of all the active channels are read.
#define STATUSR_ZYXOW           (1<<7)

//ZOW is set whenever a new acceleration sample related to the Z-axis is
//generated before the retrieval of the previous sample.  When this occurs the
//previous sample is overwritten. ZOW is cleared anytime OUT_Z_MSB register is
//read.
#define STATUSR_ZOW             (1<<6)

//YOW is set whenever a new acceleration sample related to the Y-axis is
//generated before the retrieval of the previous sample.  When this occurs the
//previous sample is overwritten. YOW is cleared anytime OUT_Y_MSB register is
//read.
#define STATUSR_YOW             (1<<5)

//XOW is set whenever a new acceleration sample related to the X-axis is
//generated before the retrieval of the previous sample.  When this occurs the
//previous sample is overwritten. XOW is cleared anytime OUT_X_MSB register is
//read.
#define STATUSR_XOW             (1<<4)

//ZYXDR signals that a new sample for any of the enabled channels is available.
//ZYXDR is cleared when the high-bytes of the acceleration data (OUT_X_MSB,
//OUT_Y_MSB, OUT_Z_MSB) of all the enabled channels are read.
#define STATUSR_ZYXDR           (1<<3)

//ZDR is set whenever a new acceleration sample related to the Z-axis is
//generated. ZDR is cleared anytime OUT_Z_MSB register is read.
#define STATUSR_ZDR             (1<<2)

//YDR is set whenever a new acceleration sample related to the Y-axis is
//generated. YDR is cleared anytime OUT_Y_MSB register is read.
#define STATUSR_YDR             (1<<1)

//XDR is set whenever a new acceleration sample related to the X-axis is
//generated. XDR is cleared anytime OUT_X_MSB register is read.
#define STATUSR_XDR             (1<<0)

//0 No FIFO overflow events detected.
//1 FIFO event detected; FIFO has overflowed.
#define STATUSR_F_OVF           (1<<7)

//0 No FIFO watermark events detected.
//1 FIFO Watermark event detected. FIFO sample count is greater than watermark value.
//If F_MODE = 11, Trigger Event detected
#define STATUSR_F_WMRK_FLAG     (1<<6)

//Indicate the number of acceleration samples currently stored in the FIFO
//buffer. Count 000000 indicates that the FIFO is empty.
#define STATUSR_F_CNT_M         ((1<<5) | (1<<4) | (1<<3) | \
                                 (1<<2) | (1<<1) | (1<<0) )



/////////////////////////REG_F_SETUP////////////////////////////////////////////
//FIFO buffer overflow mode mask.
#define F_SETUP_MODE_M          ((1<<7)|(1<<6))

//FIFO Event Sample Count Watermark mask.
#define F_SETUP_WMRK_M          ((1<<5) | (1<<4) | (1<<3) | \
                                (1<<2) | (1<<1) | (1<<0) )

/////////////////////////Trigger Configuration Register////////////////////////
//Transient interrupt trigger bit. Default value: 0
#define TRIG_CFG_TRANS          (1<<5)

//Trig_LNDPRT Landscape/Portrait Orientation interrupt trigger bit. Default value: 0
#define TRIG_CFG_LNDPRT         (1<<4)

//Trig_PULSE Pulse interrupt trigger bit. Default value: 0
#define TRIG_CFG_PULSE          (1<<3)

//Trig_FF_MT Freefall/Motion trigger bit. Default value: 0
#define TRIG_CFG_FF_MT          (1<<2)

//FIFO Gate Error. Default value: 0.
//0: No FIFO Gate Error detected.
//1: FIFO Gate Error was detected.
//Emptying the FIFO buffer clears the FGERR bit in the SYS_MOD register.
//See section 0x2C: CTRL_REG3 Interrupt Control Register for more information
//on configuring the FIFO Gate function.
#define SYSMOD_FGERR            (1<<7)

//FGT[4:0] Number of ODR time units since FGERR was asserted. Reset when FGERR
//Cleared. Default value: 0_0000
#define SYSMOD_FGT_M            ( (1<<6) | (1<<5) | (1<<4) | (1<<3) | (1<<2) )

//System Mode. Default value: 00.
//00: STANDBY mode
//01: WAKE mode
//10: SLEEP mode
#define SYSMOD_SYSMOD_M         ( (1<<1) | (1<<0) )


/////////////////////////System Interrupt Status Register////////////////////////
//Auto-SLEEP/WAKE interrupt status bit.
#define INT_SRC_ASLP            (1<<7)

//FIFO interrupt status bit.
#define INT_SRC_FIFO            (1<<6)

//Transient interrupt status bit.
#define INT_SRC_TRANS           (1<<5)

//Landscape/Portrait Orientation interrupt status bit.
#define INT_SRC_LNDPRT          (1<<4)

//Pulse interrupt status bit.
#define INT_SRC_PULSE           (1<<3)

//Freefall/Motion interrupt status bit.
#define INT_SRC_FF_MT           (1<<2)

//Data Ready Interrupt bit status.
#define INT_SRC_DRDY            (1<<0)

/////////////////////////XYZ_DATA_CFG Register////////////////////////////////
//Enable High-pass output data
#define XYZ_DATA_CFG_HPF_OUT    (1<<4)

//Output buffer data format full scale mask.
#define XYZ_DATA_CFG_FS_M       ((1<<1) | (1<<0))

/////////////////////////High-Pass Filter Register/////////////////////////////
//High-Pass Filter (HPF) for Pulse Processing Function.
#define HP_FILTER_CUTOFF_HPF_BYP (1<<5)

//Low-Pass Filter (LPF) for Pulse Processing Function.
#define HP_FILTER_CUTOFF_LPF_EN  (1<<4)

//Cutoff frequency selection
#define HP_FILTER_CUTOFF_SEL_M   ((1<<1) | (1<<0))

//////////////////////Portrait/Landscape Status Register//////////////////////
//Landscape/Portrait status change flag
#define PL_STATUS_NEWLP      (1<<7)

//Z-Tilt Angle Lockout
#define PL_STATUS_LO         (1<<6)

//Landscape/Portrait orientation mask.
#define PL_STATUS_LAPO_M     ((1<<2) | (1<<1))

//Back or Front orientation
#define PL_STATUS_BAFRO      (1<<0)

//////////////////Portrait/Landscape Configuration Register ///////////////////
//Debounce counter mode selection.
#define PL_CFG_DBCNTM           (1<<7)

//Portrait/Landscape Detection Enable.
#define PL_CFG_PL_EN            (1<<6)

//////////////////Back/Front and Z Compensation Register//////////////////////
//Back/Front Trip Angle Threshold
#define PL_BF_ZCOMP_BKFR        ((1<<7) | (1<<6))

//Z-Lock Angle Threshold.
#define PL_BF_ZCOMP_ZLCOCK      ((1<<2) | (1<<1) |  (1<<0))

///////////Portrait/Landscape Threshold and Hysteresis Registe/////////////////
//Portrait/Landscape trip threshold angle from 15¡ã to 75¡ã.
#define P_L_THS_THS_M           ((1<<7) | (1<<6) |  (1<<5) | (1<<4) | (1<<3))

//This angle is added to the threshold angle for a smoother transition from
//Portrait to Landscape and Landscape to Portrait.
#define P_L_THS_HYS_M           ((1<<2) | (1<<1) |  (1<<0))

//Event Latch Enable: Event flags are latched into FF_MT_SRC register.
//Reading of the FF_MT_SRC register clears the event flag EA and all FF_MT_SRC bits.
//Default value: 0.
//0: Event flag latch disabled
//1: event flag latch enabled
#define FF_MT_CFG_ELE           (1<<7)

//OAE bit allows the selection between Motion (logical OR combination) and Freefall
//(logical AND combination) detection.  Default value: 0. (Freefall Flag)
//0: Freefall Flag (Logical AND combination)
//1: Motion Flag (Logical OR combination)
#define FF_MT_CFG_OAE           (1<<6)

//Event flag enable on Z Default value: 0.
//0: event detection disabled;
//1: raise event flag on measured acceleration value beyond preset threshold
#define FF_MT_CFG_ZEFE          (1<<5)

//Event flag enable on Y Default value: 0.
//0: event detection disabled;
//1: raise event flag on measured acceleration value beyond preset threshold
#define FF_MT_CFG_YEFE          (1<<4)

//Event flag enable on X Default value: 0.
//0: event detection disabled;
//1: raise event flag on measured acceleration value beyond preset threshold
#define FF_MT_CFG_XEFE          (1<<3)

//Event Active Flag. Default value: 0.
//0: No event flag has been asserted; 1: one or more event flag has been asserted.
#define FF_MT_SRC_EA            (1<<7)

//Z Motion Flag. Default value: 0.
//0: No Z Motion event detected, 1: Z Motion has been detected
//This bit reads always zero if the ZEFE control bit is set to zero
#define FF_MT_SRC_ZHE           (1<<5)

//Z Motion Polarity Flag. Default value: 0.
//0: Z event was Positive g, 1: Z event was Negative g
//This bit read always zero if the ZEFE control bit is set to zero
#define FF_MT_SRC_ZHP           (1<<4)

//Y Motion Flag. Default value: 0.
//0: No Y Motion event detected, 1: Y Motion has been detected
//This bit read always zero if the YEFE control bit is set to zero
#define FF_MT_SRC_YHE           (1<<3)

//Y Motion Polarity Flag. Default value: 0
//0: Y event detected was Positive g, 1: Y event was Negative g
//This bit reads always zero if the YEFE control bit is set to zero
#define FF_MT_SRC_YHP           (1<<2)

//X Motion Flag. Default value: 0
//0: No X Motion event detected, 1: X Motion has been detected
//This bit reads always zero if the XEFE control bit is set to zero
#define FF_MT_SRC_XHE           (1<<1)

//X Motion Polarity Flag. Default value: 0
//0: X event was Positive g, 1: X event was Negative g
//This bit reads always zero if the XEFE control bit is set to zero
#define FF_MT_SRC_XHP           (1<<0)

//Debounce counter mode selection. Default value: 0.
//0: increments or decrements debounce, 1: increments or clears counter.
#define FF_MT_THS_DBCNTM        (1<<7)

//Freefall /Motion Threshold: Default value: 000_0000.
#define FF_MT_THS_THS_M         ((1<<6) | (1<<5) | (1<<4) | (1<<3) | \
                                 (1<<2) | (1<<1) | (1<<0) )
//Transient event flags are latched into the TRANSIENT_SRC register.
//Reading of the TRANSIENT_SRC register clears the event
//flag. Default value: 0.
//0: Event flag latch disabled; 1: Event flag latch enabled
#define TRANSIENT_CFG_ELE     (1<<4)

//Event flag enable on Z transient acceleration greater than transient threshold
//event. Default value: 0.
//0: Event detection disabled;
//1: Raise event flag on measured acceleration delta value greater than transient
//   threshold.
#define TRANSIENT_CFG_ZTEFE   (1<<3)

//Event flag enable on Y transient acceleration greater than transient threshold
//event.  Default value: 0.
//0: Event detection disabled;
//1: Raise event flag on measured acceleration delta value greater than transient
//   threshold.
#define TRANSIENT_CFG_YTEFE   (1<<2)

//Event flag enable on X transient acceleration greater than transient threshold event.
//    Default value: 0.
//0: Event detection disabled;
//1: Raise event flag on measured acceleration delta value greater than transient
//   threshold.
#define TRANSIENT_CFG_XTEFE   (1<<1)

//Bypass High-Pass filter
//Default value: 0.
//0: Data to transient acceleration detection block is through HPF
//1: Data to transient acceleration detection block is NOT through
//HPF (similar to motion detection function)
#define TRANSIENT_CFG_HPF_BYP (1<<0)



//Event Active Flag. Default value: 0.
//0: no event flag has been asserted; 1: one or more event flag has been asserted.
#define TRANSIENT_SCR_EA            (1<<6)

//Z transient event. Default value: 0.
//0: no interrupt
//1: Z Transient acceleration greater than the value of TRANSIENT_THS event has occurred
#define TRANSIENT_SCR_ZTRANSE       (1<<5)

//Polarity of Z Transient Event that triggered interrupt. Default value: 0.
//0: Z event was Positive g
//1: Z event was Negative g
#define TRANSIENT_SCR_Z_TRANS_POL   (1<<4)

//Y transient event. Default value: 0.
//0: no interrupt
//1: Y Transient acceleration greater than the value of TRANSIENT_THS event has occurred
#define TRANSIENT_SCR_YTRANSE       (1<<3)

//Polarity of Y Transient Event that triggered interrupt. Default value: 0.
//0: Y event was Positive g
//1: Y event was Negative g
#define TRANSIENT_SCR_Y_TRANS_POL   (1<<2)

//X transient event. Default value: 0.
//0: no interrupt
//1: X Transient acceleration greater than the value of TRANSIENT_THS event has occurred
#define TRANSIENT_SCR_XTRANSE       (1<<1)

//Polarity of X Transient Event that triggered interrupt. Default value: 0.
//0: X event was Positive g
//1: X event was Negative g
#define TRANSIENT_SCR_X_TRANS_POL   (1<<0)


//Debounce counter mode selection. Default value: 0.
//0: increments or decrements debounce
//1: increments or clears counter.
#define TRANSIENT_THS_DBCNTM        (1<<7)

//Transient Threshold: Default value: 000_0000.
#define TRANSIENT_THS_THS_M         ((1<<6) | (1<<5) | (1<<4) | (1<<3) | \
                                     (1<<2) | (1<<1) | (1<<0) )

//Double Pulse Abort. Default value: 0.
//0: Double Pulse detection is not aborted if the start of a pulse is detected
//during the time period specified by the PULSE_LTCY register.
//1: Setting the DPA bit momentarily suspends the double tap detection if the
//start of a pulse is detected during the time period specified by the PULSE_LTCY
//register and the pulse ends before the end of the time period specified by the
//PULSE_LTCY register.
#define PULSE_CFG_DPA           (1<<7)

//Pulse event flags are latched into the PULSE_SRC register.
//Reading of the PULSE_SRC register clears the event flag.  Default value: 0.
//0: Event flag latch disabled; 1: Event flag latch enabled
#define PULSE_CFG_ELE           (1<<6)

//Event flag enable on double pulse event on Z-axis. Default value: 0.
//0: Event detection disabled; 1: Event detection enabled
#define PULSE_CFG_ZDPEFE        (1<<5)

//Event flag enable on single pulse event on Z-axis. Default value: 0.
//0: Event detection disabled; 1: Event detection enabled
#define PULSE_CFG_ZSPEFE        (1<<4)

//Event flag enable on double pulse event on Y-axis. Default value: 0.
//0: Event detection disabled; 1: Event detection enabled
#define PULSE_CFG_YDPEFE        (1<<3)

//Event flag enable on single pulse event on Y-axis. Default value: 0.
//0: Event detection disabled; 1: Event detection enabled
#define PULSE_CFG_YSPEFE        (1<<2)

//Event flag enable on double pulse event on X-axis. Default value: 0.
//0: Event detection disabled; 1: Event detection enabled
#define PULSE_CFG_XDPEFE        (1<<1)

//Event flag enable on single pulse event on X-axis. Default value: 0.
//0: Event detection disabled; 1: Event detection enabled
#define PULSE_CFG_XSPEFE        (1<<0)

//Event Active Flag. Default value: 0.
//0: No interrupt has been generated
//1: One or more interrupt events have been generated
#define PULSE_SRC_EA            (1<<7)

//Z-axis event. Default value: 0.
//0: No interrupt; 1: Z-axis event has occurred
#define PULSE_SRC_AXZ           (1<<6)

//Y-axis event. Default value: 0.
//0: No interrupt; 1: Y-axis event has occurred
#define PULSE_SRC_AXY           (1<<5)

//X-axis event. Default value: 0.
//0: No interrupt; 1: X-axis event has occurred
#define PULSE_SRC_AXX 10        (1<<4)

//Double pulse on first event. Default value: 0.
//0: Single Pulse Event triggered interrupt
//1: Double Pulse Event triggered interrupt
#define PULSE_SRC_DPE           (1<<3)

//Pulse polarity of Z-axis Event. Default value: 0.
//0: Pulse Event that triggered interrupt was Positive
//1: Pulse Event that triggered interrupt was negative
#define PULSE_SRC_POLZ          (1<<2)

//Pulse polarity of Y-axis Event. Default value: 0.
//0: Pulse Event that triggered interrupt was Positive
//1: Pulse Event that triggered interrupt was negative
#define PULSE_SRC_POLY          (1<<1)

//Pulse polarity of X-axis Event. Default value: 0.
//0: Pulse Event that triggered interrupt was Positive
//1: Pulse Event that triggered interrupt was negative
#define PULSE_SRC_POLX          (1<<0)

//Pulse Threshold on X-axis mask.
#define PULSE_THSX_M            ( (1<<6) | (1<<5) | (1<<4) | (1<<3) | \
                                  (1<<2) | (1<<1) | (1<<0)  )

//Pulse Threshold on Y-axis mask.
#define PULSE_THSY_M            ( (1<<6) | (1<<5) | (1<<4) | (1<<3) | \
                                  (1<<2) | (1<<1) | (1<<0)  )

//Pulse Threshold on Z-axis mask.
#define PULSE_THSZ_M            ( (1<<6) | (1<<5) | (1<<4) | (1<<3) | \
                                  (1<<2) | (1<<1) | (1<<0)  )

//Configures the Auto-WAKE sample frequency when the device is in SLEEP Mode. Default value: 00.
#define CTRL_REG1_ASLP_RATE_M    ((1<<7) | (1<<6) )

//Data rate selection. Default value: 000.
#define CTRL_REG1_DR_M           ((1<<5) | (1<<4) | (1<<3) )

//Reduced noise reduced Maximum range mode. Default value: 0.
//0: Normal mode
//1: Reduced Noise mode
#define CTRL_REG1_LNOISE         (1<<2)

//Fast Read mode: Data format limited to single Byte Default value: 0.
//0: Normal mode
//1: Fast Read Mode
#define CTRL_REG1_F_READ         (1<<1)

//Full Scale selection. Default value: 00.
//0: STANDBY mode
//1: ACTIVE mode
#define CTRL_REG1_ACTIVE         (1<<0)

//Self-Test Enable. Default value: 0.
//0: Self-Test disabled
//1: Self-Test enabled
#define CTRL_REG2_ST             (1<<7)

//Software Reset. Default value: 0.
//0: Device reset disabled
//1: Device reset enabled.
#define CTRL_REG2_RST            (1<<6)

//SLEEP mode power scheme selection.
#define CTRL_REG2_SMODS_M        ((1<<4) | (1<<3))

//Auto-SLEEP enable. Default value: 0.
//0: Auto-SLEEP is not enabled;
//1: Auto-SLEEP is enabled.
#define CTRL_REG2_SLPE           (1<<2)

//ACTIVE mode power scheme selection
#define CTRL_REG2_MODS_M         ((1<<1) | (1<<0) )

//0: FIFO gate is bypassed. FIFO is flushed upon the system mode transitioning
//    from WAKE to SLEEP mode or from SLEEP to WAKE mode. Default value: 0.
//1: The FIFO input buffer is blocked when transitioning from WAKE to SLEEP mode
//or from SLEEP to WAKE mode until the FIFO is flushed. Although the system
//transitions from WAKE to SLEEP or from SLEEP to WAKE the contents of the FIFO
//buffer are preserved, new data samples are ignored until the FIFO is emptied by
//the host application.
//
//If the FIFO_GATE bit is set to logic ¡®1¡¯ and the FIFO buffer is not emptied
//before the arrival of the next sample, then the FGERR bit in the SYS_MOD
//register (0x0B) will be asserted. The FGERR bit remains asserted as long as the
//FIFO buffer remains un-emptied.
//
//Emptying the FIFO buffer clears the FGERR bit in the SYS_MOD register.
#define CTRL_REG3_FIFO_GATE      (1<<7)

//0: Transient function is bypassed in SLEEP mode. Default value: 0.
//1: Transient function interrupt can wake up system
#define CTRL_REG3_WAKE_TRANS     (1<<6)

//0: Orientation function is bypassed in SLEEP mode. Default value: 0.
//1: Orientation function interrupt can wake up system
#define CTRL_REG3_WAKE_LNDPRT    (1<<5)

//0: Pulse function is bypassed in SLEEP mode. Default value: 0.
//1: Pulse function interrupt can wake up system
#define CTRL_REG3_WAKE_PULSE     (1<<4)

//0: Freefall/Motion function is bypassed in SLEEP mode. Default value: 0.
//1: Freefall/Motion function interrupt can wake up
#define CTRL_REG3_WAKE_FF_MT     (1<<3)

//Interrupt polarity ACTIVE high, or ACTIVE low. Default value: 0.
//0: ACTIVE low
//1: ACTIVE high
#define CTRL_REG3_IPOL           (1<<1)

//Push-Pull/Open Drain selection on interrupt pad. Default value: 0.
//0: Push-Pull
//1: Open Drain
#define CTRL_REG3_PP_OD          (1<<0)

//Interrupt Enable. Default value: 0.
//0: Auto-SLEEP/WAKE interrupt disabled
//1: Auto-SLEEP/WAKE interrupt enabled.
#define CTRL_REG4_INT_EN_ASLP   (1<<7)

//Interrupt Enable. Default value: 0.
//0: FIFO interrupt disabled
//1: FIFO interrupt enabled.
#define CTRL_REG4_INT_EN_FIFO   (1<<6)

//Interrupt Enable. Default value: 0.
//0: Transient interrupt disabled
//1: Transient interrupt enabled.
#define CTRL_REG4_INT_EN_TRANS  (1<<5)

//Interrupt Enable. Default value: 0.
//0: Orientation (Landscape/Portrait) interrupt disabled.
//1: Orientation (Landscape/Portrait) interrupt enabled.
#define CTRL_REG4_INT_EN_LNDPR  (1<<4)

//Interrupt Enable. Default value: 0.
//0: Pulse Detection interrupt disabled
//1: Pulse Detection interrupt enabled
#define CTRL_REG4_INT_EN_PULSE  (1<<3)

//Interrupt Enable. Default value: 0.
//0: Freefall/Motion interrupt disabled
//1: Freefall/Motion interrupt enabled
#define CTRL_REG4_INT_EN_FF_MT  (1<<2)

//Interrupt Enable. Default value: 0.
//0: Data Ready interrupt disabled
//1: Data Ready interrupt enabled
#define CTRL_REG4_INT_EN_DRDY   (1<<0)

//INT1/INT2 Configuration. Default value: 0.
//0: Interrupt is routed to INT2 pin
//1: Interrupt is routed to INT1 pin
#define CTRL_REG5_INT_CFG_ASLP   (1<<7)

//INT1/INT2 Configuration. Default value: 0.
//0: Interrupt is routed to INT2 pin
//1: Interrupt is routed to INT1 pin
#define CTRL_REG5_INT_CFG_FIFO   (1<<6)

//INT1/INT2 Configuration. Default value: 0.
//0: Interrupt is routed to INT2 pin
//1: Interrupt is routed to INT1 pin
#define CTRL_REG5_INT_CFG_TRANS  (1<<5)

//INT1/INT2 Configuration. Default value: 0.
//0: Interrupt is routed to INT2 pin
//1: Interrupt is routed to INT1 pin
#define CTRL_REG5_INT_CFG_LNDPRT (1<<4)

//INT1/INT2 Configuration. Default value: 0.
//0: Interrupt is routed to INT2 pin
//1: Interrupt is routed to INT1 pin
#define CTRL_REG5_INT_CFG_PULSE  (1<<3)

//INT1/INT2 Configuration. Default value: 0.
//0: Interrupt is routed to INT2 pin
//1: Interrupt is routed to INT1 pin
#define CTRL_REG5_INT_CFG_FF_MT  (1<<2)

//INT1/INT2 Configuration. Default value: 0.
//0: Interrupt is routed to INT2 pin
//1: Interrupt is routed to INT1 pin
#define CTRL_REG5_INT_CFG_DRDY   (1<<0)


template<typename I2cMaster>
class MMA8451Q {
public:
	void initialize(uint8_t address) {

		adapter.initialize(address, 0,0,0,0);

		//enable low noise mode, activate, DR1=1 (200Hz)
		registerWrite(MMA8451_REG_CTRL_REG1, CTRL_REG1_LNOISE | CTRL_REG1_ACTIVE | (1<<3));

		//set high resolution oversampling
		registerWrite(MMA8451_REG_CTRL_REG2, (1<<1));
		//enable ring buffer fifo mode
		//registerWrite(MMA8451_REG_F_SETUP, 1<<6);
	}

	void registerWrite(uint8_t reg, uint8_t value) {
		while (adapter.getState() == xpcc::I2c::AdapterState::Busy)
			;

		buffer[0] = reg;
		buffer[1] = value;

		adapter.initialize(buffer, 2, 0, 0);
		I2cMaster::startBlocking(&adapter);

	}

	bool read() {
		while (adapter.getState() == xpcc::I2c::AdapterState::Busy)
			;

		uint8_t* buf = buffer;
		buf[0] = MMA8451_REG_OUT_X_MSB;

		adapter.initialize(buf, 1, buf, 6);
		if(I2cMaster::start(&adapter)) {
			readPending = true;
			return true;
		}
		return false;
	}

	bool isDataAvail() {
		return !isBusy() && readPending;
	}


	bool getXYZ(xpcc::Vector3f &vector) {
		if(!isDataAvail()) {
			return false;
		}
		int16_t aux;

		float counts[] = {16384.0, 8192.0, 4096.0};

		for(int i = 0; i < 6; i+=2) {
			aux = 0;
			aux = buffer[i];
			aux <<= 8;
			aux |= buffer[i+1];

			vector[i>>1] = aux / counts[0];
		}
		readPending = false;
		return true;
	}

	bool isBusy() {
		return (adapter.getState() == xpcc::I2c::AdapterState::Busy);
	}

private:
	bool readPending;
	uint8_t buffer[8];
	xpcc::I2cWriteReadAdapter adapter;

};



#endif /* MMA8451Q_HPP_ */
