

class RTIMUBBB
{
public:
	RTIMUBBB();
	~RTIMUBBB();

	void calculate();

private:
	RTIMU *imu;
	RTIMU_DATA oldData;


};
