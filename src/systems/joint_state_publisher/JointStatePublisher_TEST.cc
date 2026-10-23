/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "JointStatePublisher.hh"

#include <gtest/gtest.h>

#include <string>

#include <sdf/Model.hh>
#include <sdf/Plugin.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Load a world with a single model, create all entities so the
/// model has a parent world, configure the plugin and check the topic.
/// \param[in] _sdfString The SDF string to load
/// \param[in] _expectedTopic The expected resolved joint state topic
void TestTopicName(const std::string &_sdfString,
  const std::string &_expectedTopic)
{
  sdf::Root root;
  root.LoadSdfString(_sdfString);
  ASSERT_EQ(1u, root.WorldCount());
  const sdf::World *worldSdf = root.WorldByIndex(0);
  ASSERT_NE(nullptr, worldSdf);
  ASSERT_EQ(1u, worldSdf->ModelCount());
  const sdf::Model *modelSdf = worldSdf->ModelByIndex(0);
  ASSERT_NE(nullptr, modelSdf);
  ASSERT_FALSE(modelSdf->Plugins().empty());
  sdf::Plugin pluginSdf = modelSdf->Plugins()[0];

  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);
  Entity worldEntity = creator.CreateEntities(worldSdf);
  ASSERT_NE(kNullEntity, worldEntity);

  Entity modelEntity = ecm.EntityByComponents(
      components::Model(), components::Name(modelSdf->Name()));
  ASSERT_NE(kNullEntity, modelEntity);

  JointStatePublisher plugin;
  plugin.Configure(modelEntity, pluginSdf.Element(), ecm, eventMgr);

  EXPECT_EQ(_expectedTopic, plugin.ResolvedTopicNames().jointStateTopic);
}

TEST(JointStatePublisherTest, AbsoluteTopicName)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='jsp_test_world'>
        <model name='jsp_model' namespace='ns'>
          <link name='link'/>
          <plugin name='gz::sim::systems::JointStatePublisher'
                  filename='gz-sim-joint-state-publisher-system'>
            <topic>/test_joint_state</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TestTopicName(sdfString, "/test_joint_state");
}

TEST(JointStatePublisherTest, RelativeTopicName)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='jsp_test_world'>
        <model name='jsp_model' namespace='ns'>
          <link name='link'/>
          <plugin name='gz::sim::systems::JointStatePublisher'
                  filename='gz-sim-joint-state-publisher-system'>
            <topic>test_joint_state</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TestTopicName(sdfString, "ns/test_joint_state");
}

TEST(JointStatePublisherTest, DefaultTopicName)
{
  // Incomplete sdf, only used to test topic resolution behavior
  const std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='jsp_test_world'>
        <model name='jsp_model' namespace='ns'>
          <link name='link'/>
          <plugin name='gz::sim::systems::JointStatePublisher'
                  filename='gz-sim-joint-state-publisher-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TestTopicName(sdfString, "ns/joint_state");
}

TEST(JointStatePublisherTest, TopicNameWithoutNs)
{
  // A custom topic is unchanged when no namespace is specified.
  // Incomplete sdf, only used to test topic resolution behavior
  std::string sdfString = R"(
    <sdf version='1.10'>
      <world name='jsp_test_world'>
        <model name='jsp_model'>
          <link name='link'/>
          <plugin name='gz::sim::systems::JointStatePublisher'
                  filename='gz-sim-joint-state-publisher-system'>
            <topic>test_joint_state</topic>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TestTopicName(sdfString, "test_joint_state");

  // The original default topic is preserved when no namespace is specified.
  sdfString = R"(
    <sdf version='1.10'>
      <world name='jsp_test_world'>
        <model name='jsp_model'>
          <link name='link'/>
          <plugin name='gz::sim::systems::JointStatePublisher'
                  filename='gz-sim-joint-state-publisher-system'>
          </plugin>
        </model>
      </world>
    </sdf>)";

  TestTopicName(sdfString,
      "/world/jsp_test_world/model/jsp_model/joint_state");
}

