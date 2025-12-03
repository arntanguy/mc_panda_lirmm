#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule_visual.h>
#include <mc_panda_lirmm/panda_lirmm.h>

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    using namespace mc_panda_lirmm;
    ForAllVariants([&names](PandaLIRMMRobots robot, bool pump, bool foot, bool hand, bool fixSensorFrame)
                   { names.push_back(NameFromParams(robot, pump, foot, hand, fixSensorFrame)); });
    names.push_back("Panda2LIRMM::Test");
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("PandaLIRMM")

    static auto variant_factory = []()
    {
      std::map<std::string, std::function<mc_rbdyn::RobotModule *()>> variant_factory;
      using namespace mc_panda_lirmm;
      ForAllVariants(
          [&variant_factory](PandaLIRMMRobots robot, bool pump, bool foot, bool hand, bool fixSensorFrame)
          {
            variant_factory[NameFromParams(robot, pump, foot, hand, fixSensorFrame)] = [=]()
            { return new PandaLIRMM(robot, pump, foot, hand, fixSensorFrame); };
          });
      return variant_factory;
    }();

    if(n == "Panda2LIRMM::Test")
    {
      auto size = Eigen::Vector3d{0.75, 1.0, 0.75};
      auto boxYaml = fmt::format(R"(
name: box
origin:
  translation: [0, 0, {0}]
  rotation: [0, 0, 0]
material:
  color:
    r: 0
    g: 0
    b: 1
    a: 1
geometry:
  box:
    size: [{1}, {2}, {3}]
inertia:
  mass: 10 # not measured, just a reasonable assumption of the table weight
fixed: true
      )",
                                 size.z() / 2, size.x(), size.y(), size.z());
      auto boxConfig = mc_rtc::Configuration::fromYAMLData(boxYaml);
      auto rmV = mc_rbdyn::robotModuleFromVisual("box", boxConfig);
      auto pandaRm = mc_rbdyn::RobotLoader::get_robot_module("PandaDefault");

      auto connect_rm = new mc_rbdyn::RobotModule(rmV->connect(
          *pandaRm, "box", /* robot connection link */
          "world", /* tool connection link */
          "", /* prefix */
          mc_rbdyn::RobotModule::ConnectionParameters{}
              .name("panda2_lirmm_default")
              .X_other_connection(sva::PTransformd(Eigen::Vector3d{-(size.x() / 2 - 0.27), 0, -size.z()}))));
      ;
      return connect_rm;
    }

    auto it = variant_factory.find(n);
    if(it != variant_factory.end())
    {
      return it->second();
    }
    else
    {
      mc_rtc::log::error("PandaLIRMM module Cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
