#include "AdvancedLaneFinder.hpp"

int load_params(int argc, char* argv[], AdvancedLaneFinderParams*, ImagePipelineParams*);

int main(int argc, char* argv[]){
  
  AdvancedLaneFinderParams ap;
  ImagePipelineParams ip;

  int result = load_params(argc, argv, &ap, &ip);
  if( result != 0){
    return result; }

  AdvancedLaneFinder finder{ap, ip};
  
  finder.Run();

  return 0;
}