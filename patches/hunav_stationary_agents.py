from pathlib import Path


source = Path('/home/hunav_gz_ws/src/hunav_gazebo_wrapper/src/HuNavPlugin.cpp')
text = source.read_text()

loop = '  for (auto a : _agents.agents)\n  {\n'
assert text.count(loop) == 1, 'unexpected HuNavPlugin pedestrian loop'
stationary = '''  for (auto a : _agents.agents)
  {
    // max_vel=0 is an existing HuNav agent field, but HuNavSim may still
    // return a social-force displacement. Keep pose and reported velocity
    // consistent at the Gazebo boundary for explicitly static agents.
    for (const auto & pedestrian : pedestrians)
    {
      if (a.id == pedestrian.id && pedestrian.desired_velocity <= 0.0)
      {
        a.position = pedestrian.position;
        a.yaw = pedestrian.yaw;
        a.velocity.linear.x = 0.0;
        a.velocity.linear.y = 0.0;
        a.velocity.linear.z = 0.0;
        a.velocity.angular.x = 0.0;
        a.velocity.angular.y = 0.0;
        a.velocity.angular.z = 0.0;
        a.linear_vel = 0.0;
        a.angular_vel = 0.0;
        break;
      }
    }
'''
text = text.replace(loop, stationary, 1)

pose = '    model->SetWorldPose(actorPose);  //, true, true); // false, false);\n'
assert text.count(pose) == 1, 'unexpected HuNavPlugin pedestrian pose update'
text = text.replace(pose, pose + '''    for (const auto & pedestrian : pedestrians)
    {
      if (a.id == pedestrian.id && pedestrian.desired_velocity <= 0.0)
      {
        model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
        model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
        break;
      }
    }
''', 1)

script = '    actor->SetScriptTime(actor->ScriptTime() + (distanceTraveled * animationFactor));\n'
assert text.count(script) == 1, 'unexpected HuNavPlugin animation update'
text = text.replace(script, '''    if (distanceTraveled > 0.0)
      actor->SetScriptTime(actor->ScriptTime() + (distanceTraveled * animationFactor));
''', 1)

source.write_text(text)
