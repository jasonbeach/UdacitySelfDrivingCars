#pragma once
#include <memory>
#include "defs.hpp"

class LaneFinder{
public:
  LaneFinder(const Params& p);
  LaneFinder(const LaneFinder&) = delete;
  LaneFinder(LaneFinder&&) = delete;
  LaneFinder& operator=(LaneFinder&&) = delete;
  LaneFinder& operator=(const LaneFinder&) = delete;
  ~LaneFinder();

  void Run();
private:
  class Details;
  std::unique_ptr<Details> details;};
