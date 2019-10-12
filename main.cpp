#include "PID.h"
#include"pwm.h"
#include"diff.h"
#include"motor.h"
#include"source.h"

int sc_main(int argc, char* argv[])
{
	sca_tdf::sca_signal<double> iref;	//connects source module and diff module
	sca_tdf::sca_signal<double> error_sig; //connects diff module and pid module
	sca_tdf::sca_signal<double> pid_sig; 		//connects pid module and pwm module
	sca_tdf::sca_signal<double> vdrv;		//connects pwm and motor module
	sca_tdf::sca_signal<double> imeans;		//feedback to diff module


	source* s_tb=new source("tb");		//source module
	diff* diff_tb=new diff("diff_tb");	//module to get the error signal
	pwm* pwm_tb=new pwm("pwm_tb");		//calculates pwm signal
	PID* pid_tb=new PID("pid_tb");		//calculated PID
	motor* motor_tb=new motor("motor_tb"); //motor module


	s_tb->ref_value(iref);

	diff_tb->in_ref(iref);
	diff_tb->in_means(imeans);
	diff_tb->out_sig(error_sig);

	pid_tb->PID_in(error_sig);
	pid_tb->PID_out(pid_sig);

	pwm_tb->in(pid_sig);
	pwm_tb->vdrv(vdrv);

	motor_tb->motor_in(vdrv);
	motor_tb->motor_out(imeans);



	sca_util::sca_trace_file* file = sca_util::sca_create_tabular_trace_file("PWM.dat");
	sca_util::sca_trace(file,error_sig,"error_sig");
	sca_util::sca_trace(file,pid_sig,"pid_sig");
	sca_util::sca_trace(file,vdrv,"vdrv");
	sca_util::sca_trace(file,imeans,"imeans");
	sca_util::sca_trace(file,iref,"iref");

	//sca_util::sca_close_tabular_trace_file("PWM");


	sc_start(1000,SC_MS);
	return 0;
}
