#ifndef RFIDTAG_H_
#define RFIDTAG_H_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "rfidAntenna.h"

namespace gazebo {
  	namespace sensors {

		class RfidTag : public SensorPlugin
		{
			public:
				/// \brief Constructor.
				RfidTag();

				/// \brief Destructor.
				virtual ~RfidTag();

				/// \brief Load the sensor plugin.
				/// \param[in] _sensor Pointer to the sensor that loaded this plugin.
				/// \param[in] _sdf SDF element that describes the plugin.
				virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
			
				std::string Name(){
					return parentSensor->Name();
				}

			private:
				/// \brief Callback that receives the contact sensor's update signal.
				virtual void OnUpdate();

				/// \brief Pointer to the contact sensor
				sensors::RFIDTagPtr parentSensor;

				/// \brief Publisher for RFID pose messages.
				transport::PublisherPtr scanPub;

				/// \brief Connection that maintains a link between the contact sensor's
				/// updated signal and the OnUpdate callback.
				event::ConnectionPtr updateConnection;
		};

  	} 
}
#endif /* RFIDTAG_H_ */
