#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule_visual.h>

#include <mc_panda/panda.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>

namespace mc_panda
{ //  TODO: this should be moved to mc_panda

static std::string pandaVariant(bool pump, bool foot, bool hand)
{
  if(pump && !foot && !hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_pump'");
    return "pump";
  }
  if(!pump && foot && !hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_foot'");
    return "foot";
  }
  if(!pump && !foot && hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_hand'");
    return "hand";
  }
  if(!pump && !foot && !hand)
  {
    mc_rtc::log::info("PandaRobotModule uses the panda variant: 'panda_default'");
    return "default";
  }
  mc_rtc::log::error("PandaRobotModule does not provide this panda variant...");
  return "";
}

static std::string pandaVariantUppercase(bool pump, bool foot, bool hand)
{
  auto v = pandaVariant(pump, foot, hand);
  if(v.size())
  {
    v[0] = std::toupper(v[0]);
  }
  return v;
}

/**
 * Default naming scheme for mc_panda::PandaRobotModule variants
 * These are used here to get the main robot module to be attached to the support
 * on which they are attached
 * TODO: move it to mc_panda and use it there
 */
inline static std::string NameFromParams(bool pump, bool foot, bool hand)
{
  auto name = "Panda" + pandaVariantUppercase(pump, foot, hand);
  return name;
}
} // namespace mc_panda

namespace mc_panda_lirmm
{

enum class PandaLIRMMRobots
{
  Panda2LIRMM,
  Panda6LIRMM,
  Panda7LIRMM
};

static std::string to_string(PandaLIRMMRobots robots)
{
  if(robots == PandaLIRMMRobots::Panda2LIRMM)
  {
    return "Panda2LIRMM";
  }
  else if(robots == PandaLIRMMRobots::Panda6LIRMM)
  {
    return "Panda6LIRMM";
  }
  else
  {
    return "Panda7LIRMM";
  }
}

inline static std::string NameFromParams(PandaLIRMMRobots robot, bool pump, bool foot, bool hand)
{
  return to_string(robot) + mc_panda::pandaVariantUppercase(pump, foot, hand);
}

template<typename Callback>
static void ForAllVariants(Callback cb)
{
  auto endEffectorOptions = std::vector<std::array<bool, 3>>{
      {false, false, false}, {true, false, false}, {false, true, false}, {false, false, true}};

  for(auto robot : {PandaLIRMMRobots::Panda2LIRMM, PandaLIRMMRobots::Panda6LIRMM, PandaLIRMMRobots::Panda7LIRMM})
  {
    for(auto endEffector : endEffectorOptions)
    {
      cb(robot, endEffector[0], endEffector[1], endEffector[2]);
    }
  }
}

} // namespace mc_panda_lirmm

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    using namespace mc_panda_lirmm;
    ForAllVariants([&names](PandaLIRMMRobots robot, bool pump, bool foot, bool hand)
                   { names.push_back(mc_panda_lirmm::NameFromParams(robot, pump, foot, hand)); });
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
          [&variant_factory](PandaLIRMMRobots robot, bool pump, bool foot, bool hand)
          {
            std::string variant_name = mc_panda_lirmm::NameFromParams(robot, pump, foot, hand);
            // Use connect approach for these variants
            Eigen::Vector3d size;
            double mass;
            // get the main panda module name from mc_panda
            auto panda_module = mc_panda::NameFromParams(pump, foot, hand);
            auto robot_name = std::string{};
            auto box_to_robot = sva::PTransformd::Identity();
            if(robot == PandaLIRMMRobots::Panda2LIRMM)
            {
              size = Eigen::Vector3d{0.75, 1.0, 0.75};
              mass = 10;
              robot_name = "panda2_lirmm_default";
              box_to_robot = sva::PTransformd(Eigen::Vector3d{-(size.x() / 2 - 0.27), 0, -size.z()});
            }
            else if(robot == PandaLIRMMRobots::Panda6LIRMM)
            {
              size = Eigen::Vector3d{0.25, 0.27, 0.716};
              mass = 40;
              robot_name = "panda6_lirmm_default";
              box_to_robot = sva::PTransformd(Eigen::Vector3d{-(size.x() / 4), 0, -size.z()});
            }
            else // Panda7LIRMM
            {
              size = Eigen::Vector3d{1.0, 0.75, 0.75};
              mass = 10;
              robot_name = "panda7_lirmm_default";
              box_to_robot = sva::PTransformd(Eigen::Vector3d{size.x() / 2 - 0.19, 0, -size.z()});
            }
            variant_factory[variant_name] = [=]()
            {
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
      mass: {4}
    fixed: true
    )",
                                         size.z() / 2, size.x(), size.y(), size.z(), mass);

              auto boxConfig = mc_rtc::Configuration::fromYAMLData(boxYaml);
              auto rmV = mc_rbdyn::robotModuleFromVisual("box", boxConfig);
              auto pandaRm = mc_rbdyn::RobotLoader::get_robot_module(panda_module);

              return new mc_rbdyn::RobotModule(rmV->connect(
                  *pandaRm, "box", "world", "",
                  mc_rbdyn::RobotModule::ConnectionParameters{}.name(robot_name).X_other_connection(box_to_robot)));
            };
          });
      mc_rtc::log::info("mc_panda_lirmm robot module declares the following variants:\n{}",
                        mc_rtc::io::to_string(variant_factory, [](const auto & v) { return v.first; }));
      return variant_factory;
    }();

    // Load the desired robot variant from the factory
    auto it = variant_factory.find(n);
    if(it != variant_factory.end())
    {
      return it->second();
    }
    else
    {
      mc_rtc::log::error("PandaLIRMM module Cannot create an object of type {}", n);
      mc_rtc::log::info("Available variants are: {}",
                        mc_rtc::io::to_string(variant_factory, [](const auto & v) { return v.first; }));
      return nullptr;
    }
  }
}
