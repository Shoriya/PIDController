#include<systemc-ams.h>
#include<systemc-ams>

SCA_TDF_MODULE(PID){
	sca_tdf::sca_in<double> PID_in;
	sca_tdf::sca_out<double> PID_out;


	SC_CTOR(PID)
	{
		//g0=1;
	}

	void initialize()
	{
		nn(0)=(4*M_PI); //ki
		nn(1)=(1/15.0); //kp
		nn(2)=0.00; 	//kd
		dd(1)=1;
		}

	void set_attributes()
	{
		set_timestep(10,SC_US);
	}

	void processing()
	{
		//input_value=PID_in.read();
		PID_out.write(lt_nd(nn,dd,PID_in.read()));

	}

private:
	double g0;
	sca_util::sca_vector<double> nn;
	sca_util::sca_vector<double> dd;
	sca_tdf::sca_ltf_nd lt_nd;

};

