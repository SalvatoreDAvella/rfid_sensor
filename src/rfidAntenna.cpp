#include "rfidAntenna.h"

#define lightspeed 299792458

using namespace gazebo;
using namespace sensors;
GZ_REGISTER_SENSOR_PLUGIN(RfidAntanna)

/////////////////////////////////////////////////
RfidAntanna::RfidAntanna() : SensorPlugin() {
}

/////////////////////////////////////////////////
RfidAntanna::~RfidAntanna() {
}

/////////////////////////////////////////////////
void RfidAntanna::Load(SensorPtr _sensor, sdf::ElementPtr _sdf) {
    
    // Get the parent sensor
    _antennaSensor = std::dynamic_pointer_cast<RFIDSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->_antennaSensor) {
        gzerr << "RfidAntenna requires a RFIDSensor.\n";
        return;
    }

    _world = physics::get_world(this->_antennaSensor->WorldName());
    _antennaEntity = boost::dynamic_pointer_cast<physics::Link>(
            this->_world->EntityByName(this->_antennaSensor->ParentName()));

    // Connect to the sensor update event.
    _updateConnection = this->_antennaSensor->ConnectUpdated(
            std::bind(&RfidAntanna::OnUpdate, this));

    // Make sure the parent sensor is active.
    _antennaSensor->SetActive(true);

    // if frequency parameter does NOT exist
    if (!(_sdf->HasElement("frequency"))) {
        std::cout
                << "Missing parameter <frequency> in RfidAntenna, default to standard"
                << std::endl;
        _frequency = 865.7e6;
    }
    else 
        _frequency = _sdf->Get<double>("frequency");

    // if noisephi parameter does NOT exist
    if (!(_sdf->HasElement("noisephi"))) {
        std::cout
                << "Missing parameter <noisephi> in RfidAntenna, default to standard"
                << std::endl;
        _noisephi = 0.1;
    }
    else 
        _noisephi = _sdf->Get<double>("noisephi");

     // if noiserssi parameter does NOT exist
    if (!(_sdf->HasElement("noiserssi"))) {
        std::cout
                << "Missing parameter <noiserssi> in RfidAntenna, default to standard"
                << std::endl;
        _noiserssi = 0.005;
    }
    else 
        _noiserssi = _sdf->Get<double>("noisephi");

    // if range parameter does NOT exist
    if (!(_sdf->HasElement("range"))) {
        std::cout
                << "Missing parameter <range> in RfidAntenna, default to standard"
                << std::endl;
        _range = 5;
    }
    else
        _range = _sdf->Get<double>("range");

    // if communication_gain parameter does NOT exist
    if (!(_sdf->HasElement("communication_gain"))) {
        std::cout
                << "Missing parameter <communication_gain> in RfidAntenna, default to standard"
                << std::endl;
        _communication_gain = 250;
    }
    else
        _communication_gain = _sdf->Get<double>("communication_gain");

    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the antenna name
    node->Init(_antennaSensor->Name());

    _node_handle = ros::NodeHandle("~/"+_antennaSensor->Name());

    _lambda = lightspeed / _frequency;
    _meterToPhi = (4 * M_PI) / _lambda;

    _number_of_tags = 0;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    _generator = std::default_random_engine(seed);
    _distribution = std::normal_distribution<double>(0.0,1.0);
    _radius_distribution = std::normal_distribution<double>(0.0,0.05);

    std::cout << "Antenna " << _antennaSensor->Name() << " properties:" << std::endl <<
        "\tFrequency: " << _frequency << std::endl <<
        "\tNoise phi (rad): " << _noisephi << std::endl <<
        "\tNoise rssi: " << _noiserssi << std::endl <<
        "\tRange: " << _range << std::endl <<
        "\tCommunication Gain: " << _communication_gain << std::endl <<
        "\tLambda " << _lambda << std::endl;

    _event_pub = _node_handle.advertise<rfid_sensor::TagArray>(
        "data", 10, 10);
}

/////////////////////////////////////////////////
void RfidAntanna::OnUpdate() {

    int count = 0;
    Sensor_V sensors = SensorManager::Instance()->GetSensors();
    for (Sensor_V::iterator iter = sensors.begin(); iter != sensors.end(); ++iter) {
        if ((*iter)->Type() == "rfidtag") {
            count++;
        }
    }

    if (_number_of_tags != count)    
    {
        _number_of_tags = 0;
        _tags_vector.clear();
        Sensor_V sensors = SensorManager::Instance()->GetSensors();
        for (Sensor_V::iterator iter = sensors.begin(); iter != sensors.end(); ++iter) {
            if ((*iter)->Type() == "rfidtag") {                    
                RFIDTag *rfidTagPointer = (RFIDTag*) ((*iter).get());
                std::string delimiter = "::";
                std::string name = rfidTagPointer->ParentName().substr(0, rfidTagPointer->ParentName().find(delimiter));
                std::cout << "adding tag: " << name 
                    << " to the antenna: " << _antennaSensor->Name() << std::endl;
                StoreTag(rfidTagPointer);
            }
        }
        std::cout << "Number of tags: " << _number_of_tags << std::endl;
    }

    double actual_range = _range;
    actual_range = actual_range * abs(1 + _radius_distribution(_generator));

    std::vector<rfid_sensor::Tag> msgArray;

    ignition::math::Pose3d world_antenna_position = this->_antennaSensor->Pose() 
            + this->_antennaEntity.lock()->WorldPose();

    std::vector<RFIDTag*>::const_iterator ci;
    for (ci = _tags_vector.begin(); ci != _tags_vector.end(); ci++) {
        physics::EntityPtr entityTag = _world->EntityByName((*ci)->ParentName());
        if(entityTag == NULL)
            continue;

        ignition::math::Pose3d world_tag_position = entityTag->WorldPose();
        double dist = world_antenna_position.Pos().Distance(world_tag_position.Pos());
        if (dist > actual_range)
            continue;

        double phi = remainder(dist * _meterToPhi + _noisephi * _distribution(_generator),
                        2 * M_PI);
        double rssi = round(40 * log10(_communication_gain * (_lambda /
                        (4 * M_PI * dist)) + (_noiserssi/dist * _distribution(_generator))));

        rfid_sensor::Tag msg;
        msg.name = (*ci)->ParentName();
        msg.dist = dist;
        msg.phi = phi;
        msg.rssi = rssi;
        msgArray.push_back(msg);
    }

    rfid_sensor::TagArray msgToSend;
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time::now();
    msgToSend.header = header;
    msgToSend.tags = msgArray;
    msgToSend.ntags = msgArray.size();
    msgToSend.antennaPose.position.x = world_antenna_position.Pos().X();
    msgToSend.antennaPose.position.y = world_antenna_position.Pos().Y();
    msgToSend.antennaPose.position.z = world_antenna_position.Pos().Z();
    msgToSend.antennaPose.orientation.w = world_antenna_position.Rot().W();
    msgToSend.antennaPose.orientation.x = world_antenna_position.Rot().X();
    msgToSend.antennaPose.orientation.y = world_antenna_position.Rot().Y();
    msgToSend.antennaPose.orientation.z = world_antenna_position.Rot().Z();

    _event_pub.publish(msgToSend);
}

void RfidAntanna::StoreTag(RFIDTag *_tag) {
    _tags_vector.push_back(_tag);
    _number_of_tags++;
}