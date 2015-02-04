//////////////////////////////////////////////////////////////////////////////
/// \ingroup control
//@{
/// \file ais_motorctrl.h
///
/// \brief Header file of the control Loop module
///
/// \author Pascal Langenberg
///
/// \version 0.1
///
/// \date 12.05.2006
///
//////////////////////////////////////////////////////////////////////////////

#ifndef AIS_MOTORCTRL_H
#define AIS_MOTORCTRL_H


#include "ais_typedef.h"
#include "motorctrl/ais_error.h"




/// Structure for Motorencoderticks
struct TICKS {
    /// number of ticks in the last cycle [-20000...20000]
    VMC_INT_16 rel_Ticks;
	/// absolute number of ticks since reset [-2°32...2^32]
    VMC_LONG_32 abs_Ticks;
};

/// Structure for State of one Motor
struct MOTORSTATE {
    /// ID of the Motor /channel [0...4]
	VMC_UCHAR_8 ID;
    /// state of the motor: 	0: acc 1: braking 2: stop
    VMC_UCHAR_8 state;
	/// desired RPM in RPM [-25000...25000]
    VMC_INT_16 ref_RPM;
	/// actual RPM in RPM [-25000...25000]
    VMC_INT_16 act_RPM;
	/// control RPM in RPM [-25000..25000]
    VMC_INT_16 set_RPM;
	/// control PWM in SIGNAL_RANGE [-SIGNAL_RANGE..+SIGNAL_RANGE]
    VMC_INT_16 set_PWM;
	/// actual voltage in mV [0...50000]
    VMC_UINT_16 act_Voltage;
	/// actual current in mA [0...10000]
    VMC_UINT_16 act_Current;
	/// actual temperature in celsius [0...200]
    VMC_UINT_16 act_Temp;
	/// actual power in W [0...250]
    VMC_UINT_16 act_Torque;
	/// absolut Rots of the motor since reset [0...2^32]
    VMC_UINT_16 act_Power;
	/// actual torque in mNm [0...10000]
    TMC_ULONG_32 abs_Rots;

	VMC_INT_16 old_set_rpm;
	VMC_INT_16 setter_diff;

	struct TICKS ticks;
};
/// Structure for Motorencoderticks
struct STATE {
    struct MOTORSTATE motorstate;
};


void init_motorcontrol();
void main_motorcontrol(long last_cycle_time);

VMC_UCHAR_8 get_id(VMC_UCHAR_8 id);
VMC_UCHAR_8 get_state(VMC_UCHAR_8 id);
VMC_INT_16 get_ref_RPM(VMC_UCHAR_8 id);
VMC_INT_16 get_act_RPM(VMC_UCHAR_8 id);
VMC_INT_16 get_set_RPM(VMC_UCHAR_8 id);
VMC_INT_16 get_set_PWM(VMC_UCHAR_8 id);
VMC_UINT_16 get_act_Voltage(VMC_UCHAR_8 id);
VMC_UINT_16 get_act_Current(VMC_UCHAR_8 id);
VMC_UINT_16 get_act_Temp(VMC_UCHAR_8 id);
VMC_UINT_16 get_act_Power(VMC_UCHAR_8 id);
VMC_UINT_16 get_act_Torque(VMC_UCHAR_8 id);
TMC_ULONG_32 get_abs_Rots(VMC_UCHAR_8 id);
//TMC_ULONG_32 clear_abs_Rots(VMC_UCHAR_8 id);		//@@@@@
VMC_UCHAR_8 get_motor_Error(VMC_UCHAR_8 id);
VMC_UCHAR_8 get_timeout_Error();
VMC_UCHAR_8 get_encoder_Error(VMC_UCHAR_8 id);
VMC_UCHAR_8 get_emergencystop_Error();
VMC_INT_16 get_rel_Ticks(VMC_UCHAR_8 id);
VMC_LONG_32 get_abs_Ticks(VMC_UCHAR_8 id);

void set_id(VMC_UCHAR_8 id, VMC_UCHAR_8 ID);
void set_set_RPM(VMC_UCHAR_8 id, VMC_INT_16 rpm);
void set_ref_RPM(VMC_UCHAR_8 id, VMC_INT_16 rpm);
void set_set_PWM(VMC_UCHAR_8 id, VMC_INT_16 pwm);

void set_all_PWM(VMC_INT_16 pwm1, VMC_INT_16 pwm2,VMC_INT_16 pwm3);
void set_all_RPM(VMC_INT_16 rpm1, VMC_INT_16 rpm2, VMC_INT_16 rpm3);
void reset_motor_Error(VMC_UCHAR_8 id);
void reset_timeout_Error();
void reset_encoder_Error(VMC_UCHAR_8 id);
void reset_emergencystop_Error();
void set_state(VMC_UINT_16 motor_ID, VMC_UCHAR_8 state);
void set_act_RPM(VMC_UINT_16 motor_ID, VMC_INT_16 rpm);
void set_act_Voltage(VMC_UINT_16 motor_ID, VMC_UINT_16 current);
void set_act_Current(VMC_UINT_16 motor_ID, VMC_UINT_16 current);
void set_act_Temp(VMC_UINT_16 motor_ID, VMC_UINT_16 temp);
void set_act_Power(VMC_UINT_16 motor_ID, VMC_UINT_16 power);
void set_act_Torque(VMC_UINT_16 motor_ID, VMC_UINT_16 torque);
void set_abs_Rots(VMC_UINT_16 motor_ID, TMC_ULONG_32 rots);
void set_rel_Ticks(VMC_UINT_16 motor_ID, VMC_UINT_16 ticks);
void set_abs_Ticks(VMC_UINT_16 motor_ID, TMC_ULONG_32 ticks);


#endif /* AIS_MOTORCTRL_H */

//@}

