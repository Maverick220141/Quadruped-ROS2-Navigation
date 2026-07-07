#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "yesense_interface/msg/imu_data.hpp"
#include "yesense_interface/msg/euler_only.hpp"
#include "yesense_interface/msg/attitude_min_vru.hpp"
#include "yesense_interface/msg/nav_min.hpp"

using std::placeholders::_1;


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class YESENSE_Subscriber : public rclcpp::Node
{
	public:
	YESENSE_Subscriber()
	: Node("yesense_subscriber")
	{
		sub_imu_data 	= this->create_subscription<yesense_interface::msg::ImuData>("imu_data", 10, std::bind(&YESENSE_Subscriber::topic_callback_imu_data, this, _1));
		sub_euler_only  = this->create_subscription<yesense_interface::msg::EulerOnly>("euler_only", 10, std::bind(&YESENSE_Subscriber::topic_callback_euler_only, this, _1));
		sub_att_min_vru = this->create_subscription<yesense_interface::msg::AttitudeMinVru>("att_min_vru", 10, std::bind(&YESENSE_Subscriber::topic_callback_att_min_vru, this, _1));
		sub_nav_min  	= this->create_subscription<yesense_interface::msg::NavMin>("nav_min", 10, std::bind(&YESENSE_Subscriber::topic_callback_nav_min, this, _1));		
	}

	private:
	void topic_callback_imu_data(const yesense_interface::msg::ImuData::SharedPtr msg) const
	{
		RCLCPP_INFO(this->get_logger(), "I heard imu data: '%u, %f, %f, %f, %f, %f, %f, %f, %u'", (unsigned int)msg->tid.tid, msg->acc.x, msg->acc.y, msg->acc.z, msg->gyro.x, msg->gyro.y, msg->gyro.z, msg->temp.temp, msg->sample_timestamp.timestamp);	
	}

	void topic_callback_euler_only(const yesense_interface::msg::EulerOnly::SharedPtr msg) const
	{
		RCLCPP_INFO(this->get_logger(), "I heard euler data: '%u, %f, %f, %f'", (unsigned int)msg->tid.tid, msg->euler.pitch, msg->euler.roll, msg->euler.yaw);	
	}

	void topic_callback_att_min_vru(const yesense_interface::msg::AttitudeMinVru::SharedPtr msg) const
	{
		RCLCPP_INFO(this->get_logger(), "I heard att min vru data: '%u, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f'", 
		(unsigned int)msg->imu_basic.tid.tid, msg->imu_basic.acc.x, msg->imu_basic.acc.y, msg->imu_basic.acc.z, msg->imu_basic.gyro.x, msg->imu_basic.gyro.y, msg->imu_basic.gyro.z,
		msg->euler.pitch, msg->euler.roll, msg->euler.yaw, msg->imu_basic.temp.temp);
	}

	void topic_callback_nav_min(const yesense_interface::msg::NavMin::SharedPtr msg) const
	{
		RCLCPP_INFO(this->get_logger(), "I heard nav min data: '%u, %.10f, %.10f, %f, %f, %f, %f, %u, %u'", 
				(unsigned int)msg->pos.tid.tid, msg->pos.pos.longitude, msg->pos.pos.latitude, msg->pos.pos.altitude, 
				msg->euler.pitch, msg->euler.roll, msg->euler.yaw, msg->pos.status.fusion_status, msg->pos.status.gnss_status);	
	}		

	rclcpp::Subscription<yesense_interface::msg::ImuData>::SharedPtr 		sub_imu_data;
	rclcpp::Subscription<yesense_interface::msg::EulerOnly>::SharedPtr 		sub_euler_only;	
	rclcpp::Subscription<yesense_interface::msg::AttitudeMinVru>::SharedPtr sub_att_min_vru;	
	rclcpp::Subscription<yesense_interface::msg::NavMin>::SharedPtr 		sub_nav_min;			
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);  
  
	rclcpp::spin(std::make_shared<YESENSE_Subscriber>());
	rclcpp::shutdown();
	
	return 0;
}
