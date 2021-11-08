#include "rfidTag.h"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_SENSOR_PLUGIN(RfidTag)

/////////////////////////////////////////////////
RfidTag::RfidTag() : SensorPlugin() {
}

/////////////////////////////////////////////////
RfidTag::~RfidTag() {
}

/////////////////////////////////////////////////
void RfidTag::Load(SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
	// Get the parent sensor.
	this->parentSensor = std::dynamic_pointer_cast<RFIDTag>(_sensor);

	// Make sure the parent sensor is valid.
	if (!this->parentSensor)
	{
		gzerr << "RFIDPlugin requires a RFIDSensor.\n";
		return;
	}

	// Connect to the sensor update event.
	this->updateConnection = this->parentSensor->ConnectUpdated(
		std::bind(&RfidTag::OnUpdate, this));

	// Make sure the parent sensor is active.
	this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void RfidTag::OnUpdate()
{
	msgs::Pose msg;
	msgs::Set(&msg, this->parentSensor->Pose());
}
