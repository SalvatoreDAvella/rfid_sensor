#ifndef RFIDAntenna_H_
#define RFIDAntenna_H_

#include <string>
#include <vector>

#include <ros/ros.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>
#include <rfid_sensor/Tag.h>
#include <rfid_sensor/TagArray.h>
#include <random>
#include <regex>
#include <map>


namespace gazebo {
	namespace sensors {

		class RfidTag;
		/// \brief A plugin for RFID Antenna sensor.
		class RfidAntanna: public SensorPlugin {
			public:
				/// \brief Constructor.
				RfidAntanna();

				/// \brief Destructor.
				virtual ~RfidAntanna();

				/// \brief Load the sensor plugin.
				/// \param[in] _sensor Pointer to the sensor that loaded this plugin.
				/// \param[in] _sdf SDF element that describes the plugin.
				virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

			private:
				/// \brief Callback that computes the RSSI and phase signals for the 
				/// tags detected by the reader antenna.
				virtual void OnUpdate();

				/// \brief Pointer to the RFID antenna sensor.
				RFIDSensorPtr _antennaSensor;

				/// \brief Connection that maintains a link between the RFID tag sensor's
				/// updated signal and the OnUpdate callback.
				event::ConnectionPtr _updateConnection;

				/// \brief Parent entity which the sensor is attached to
				boost::weak_ptr<physics::Link> _antennaEntity;

				std::map<std::string, RFIDTag*> _tags_map;

				double _frequency;
				double _noisephi;
				double _noiserssi;
				double _lambda;
				double _meterToPhi;
				double _range;
				double _communication_gain;
				std::string _optional_distribution;
				std::string _optional_distribution_params;
				std::vector<double> _params;

				int _number_of_tags;
								
				std::default_random_engine _generator;
				std::normal_distribution<double> _distribution;
				std::normal_distribution<double> _radius_distribution;

				std::uniform_real_distribution<double> _optional_dist1;
				std::exponential_distribution<double> _optional_dist2;
				std::gamma_distribution<double> _optional_dist3;
				std::weibull_distribution<double> _optional_dist4;
				std::normal_distribution<double> _optional_dist5;
				std::lognormal_distribution<double> _optional_dist6;
				std::chi_squared_distribution<double> _optional_dist7;
				std::cauchy_distribution<double> _optional_dist8;
			protected: 
				gazebo::physics::WorldPtr _world;
				ros::NodeHandle _node_handle;
				ros::Publisher _event_pub;
		};
	} //namespace sensors
} //namespace gazebo

#endif /* RFIDAntenna_H_ */
