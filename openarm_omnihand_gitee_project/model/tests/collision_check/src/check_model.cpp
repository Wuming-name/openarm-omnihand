#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <iomanip>
std::string read(const std::string& p){std::ifstream f(p);return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char** argv){
 if(argc!=4)return 2;
 auto u=urdf::parseURDF(read(argv[1]));auto sr=std::make_shared<srdf::Model>();if(!u||!sr->initString(*u,read(argv[2])))return 3;
 auto model=std::make_shared<moveit::core::RobotModel>(u,sr);planning_scene::PlanningScene scene(model);
 auto cases=YAML::LoadFile(argv[3]);std::cout<<std::setprecision(17)<<"{";bool first=true;
 for(const auto& x:cases){if(!first)std::cout<<",";first=false;auto name=x.first.as<std::string>();auto& state=scene.getCurrentStateNonConst();state.setToDefaultValues();
  for(const auto& q:x.second)state.setVariablePosition(q.first.as<std::string>(),q.second.as<double>());state.update();
  collision_detection::CollisionRequest req;req.contacts=true;req.max_contacts=1000;req.max_contacts_per_pair=10;req.distance=true;
  collision_detection::CollisionResult res;scene.checkCollision(req,res,state);
  std::cout<<"\""<<name<<"\":{\"bounds_valid\":"<<(state.satisfiesBounds()?"true":"false")<<",\"collision\":"<<(res.collision?"true":"false")<<",\"contacts\":[";
  bool a=true;for(auto& pair:res.contacts){for(auto& c:pair.second){if(!a)std::cout<<",";a=false;std::cout<<"{\"a\":\""<<c.body_name_1<<"\",\"b\":\""<<c.body_name_2<<"\",\"depth\":"<<c.depth<<"}";}}
  std::cout<<"],\"transforms\":{";a=true;for(auto& link:model->getLinkModelNames()){if(!a)std::cout<<",";a=false;auto t=state.getGlobalLinkTransform(link).matrix();std::cout<<"\""<<link<<"\":[";for(int i=0;i<4;i++)for(int j=0;j<4;j++){if(i||j)std::cout<<",";std::cout<<t(i,j);}std::cout<<"]";}std::cout<<"}}";
 }std::cout<<"}\n";return 0;
}
