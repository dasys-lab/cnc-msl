/*
 * Monitoring.h
 *
 *  Created on: 24 Mar 2016
 *      Author: Stephan Opfer
 */

#ifndef SRC_MONITORING_H_
#define SRC_MONITORING_H_


namespace std{
	class thread;
}

namespace msl
{
	class MSLWorldModel;
	class Monitoring
	{
	public:
		Monitoring(MSLWorldModel* wm);
		virtual ~Monitoring();
		void stop();

	protected:
		void run();
		void monitorMotion();
		void monitorSimulator();
		void setMaySendMessages(bool maySend);

	private:
		MSLWorldModel* wm;
		std::thread* monitorThread;
		bool running;
		bool isUsingSimulator;

	};

} /* namespace msl */

#endif /* SRC_MONITORING_H_ */
