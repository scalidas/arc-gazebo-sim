/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <functional>

#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <champ_msgs/msg/contacts_stamped.hpp>
#include <champ/utils/urdf_loader.h>

class ContactSensor: public rclcpp::Node
{
	bool foot_contacts_[4];
	std::vector<std::string> foot_links_;
	rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_publisher_;
	std::unique_ptr<gz::transport::Node> gz_node_;
	bool gz_subscribed_ {false};

public:
	ContactSensor():
		foot_contacts_ {false,false,false,false},
		Node("contacts_sensor", rclcpp::NodeOptions()
				.allow_undeclared_parameters(true)
				.automatically_declare_parameters_from_overrides(true))
	{
		std::vector<std::string> joint_names;

		joint_names = champ::URDF::getLinkNames(this->get_node_parameters_interface());
		// Keep the same indices as before (assumes URDF layout is unchanged)
		foot_links_.push_back(joint_names[2]);
		foot_links_.push_back(joint_names[6]);
		foot_links_.push_back(joint_names[10]);
		foot_links_.push_back(joint_names[14]);

		contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);

		// Initialize Ignition (gz) transport node
		gz_node_.reset(new gz::transport::Node());

		// Bind member callback into a std::function to match transport v13 API
		std::function<void(const gz::msgs::Contacts&)> cb =
			std::bind(&ContactSensor::gazeboCallback, this, std::placeholders::_1);

		// Subscribe to the contacts topic with the correct Ignition topic name
		gz_subscribed_ = gz_node_->Subscribe("/world/default/physics/contacts", cb);
	}

	~ContactSensor() = default;

	// New callback signature: const reference to gz::msgs::Contacts
	void gazeboCallback(const gz::msgs::Contacts &_msg)
	{
		for(size_t i = 0; i < 4; i++)
		{
			foot_contacts_[i] = false;
		}

		for (int i = 0; i < _msg.contact_size(); ++i)
		{
			std::vector<std::string> results;
			// collision1() is an Entity message in this transport version; use its name()
			std::string collision = _msg.contact(i).collision1().name();
			boost::split(results, collision, [](char c){ return c == ':'; });

			// Make sure we have enough parts before indexing
			if (results.size() >= 3)
			{
				for(size_t j = 0; j < 4; j++)
				{
					if (foot_links_[j] == results[2])
					{
						foot_contacts_[j] = true;
						break;
					}
				}
			}
		}
	}

	void publishContacts()
	{
		champ_msgs::msg::ContactsStamped contacts_msg;
		contacts_msg.header.stamp = this->get_clock()->now();
		contacts_msg.contacts.resize(4);

		for(size_t i = 0; i < 4; i++)
		{
			contacts_msg.contacts[i] = foot_contacts_[i];
		}

		contacts_publisher_->publish(contacts_msg);
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ContactSensor>();
	rclcpp::Rate loop_rate(50);

	while (rclcpp::ok())
	{
		node->publishContacts();
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}