
#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_
     // ------------------------------------------------
class pid_controller
{
    public:
    	pid_controller():
    		  error(0.0),
			  pre_error(0.0),
			  total_error(0.0),
			  //output(0.0),
              kp_(0.0),
              kd_(0.0),
              ki_(0.0),
              max_output(0.0)
    	{

        };
    	~pid_controller(){

        };
        void reset_error()
        {
            error = 0.0;
            pre_error = 0.0;
            total_error = 0.0;
        };
        void set_k(double kp,double ki,double kd,double max)
        {
            this->kp_ = kp;
            this->ki_ = ki;
            this->kd_ = kd;
            
            this->max_output = max;
        };        
        double calculate(double error)
        {
    	    this->error = error;
            //default ki_ = 0.0
	        double output = kp_*this->error + kd_*(this->error - this->pre_error) + ki_*this->total_error;
            //std::cout << error <<","<<kp_ <<","<<kd_<<","<<ki_<<","<<max_output<<","<<output<<","<<total_error <<std::endl;
            pre_error = this->error;

            if(output >= max_output)
            {
                output = max_output;

            }else if(output < -max_output)
            {
                output = -max_output;
            }else
            {
                total_error = total_error+this->error;
            }

            return output;        
        };

    private:      
		double error;
		double pre_error;
		double total_error;
		//double output;
        double max_output;
        double kp_;
        double ki_;
        double kd_;
};



#endif
