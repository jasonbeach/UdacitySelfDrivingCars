#include "config.hpp"
#include "LaneFinder.hpp"

int main(int argc, char* argv[]){
  
  Params p = load_params(argc, argv);

  LaneFinder lf{p};
  
  lf.Run();

  return 0;
}