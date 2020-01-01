#include "LaneFinder.hpp"

class LaneFinder::Details{
public:
  Details(const Params& p): p_(p){ }
  void Run(){
    fmt::print(" Finding them lanes....\n");  }

private:
  Params p_; };

LaneFinder::LaneFinder(const Params& p):details(std::make_unique<LaneFinder::Details>(p)) {}

void LaneFinder::Run(){details->Run();}

LaneFinder::~LaneFinder() = default;
