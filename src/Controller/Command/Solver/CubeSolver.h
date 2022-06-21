#ifndef _BUSYBIN_CUBE_SOLVER_H_
#define _BUSYBIN_CUBE_SOLVER_H_

#include "../../../Util/ThreadPool.h"
#include "../../../Util/Timer.h"
#include "../../../Model/RubiksCube.h"
#include "../../../Model/MoveStore/MoveStore.h"
#include "../../../Model/Goal/Goal.h"
#include "../../../Model/RubiksCubeModel.h"
#include <iostream>
using std::cout;
using std::endl;
#include <functional>
using std::bind;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
#include <atomic>
using std::atomic_bool;
#include <mutex>
using std::mutex;
using std::lock_guard;
#include <vector>
using std::vector;
#include <queue>
using std::queue;
#include <string>
using std::string;
#include <sstream>
using std::istringstream;
#include <iterator>
using std::istream_iterator;

namespace busybin
{
  /**
   * Solver controller for the cube.
   */
  class CubeSolver {
  protected:
    typedef RubiksCube::MOVE MOVE;

    // Order is import.  The cube pointer has to be initialized before the
    // MoveStores.
    ThreadPool*            pThreadPool;
    RubiksCubeModel*       pCube;

  private:
    atomic_bool movesInQueue;
    queue<MOVE> moveQueue;
    mutex       moveMutex;
    Timer       moveTimer;

    void onKeypress(int key, int scancode, int action, int mods);
    void onPulse(double elapsed);
    void replace(const string& needle, string& haystack, const string& with) const;

  protected:
    void setSolving(bool solving);
    void processGoalMoves(const Goal& goal, RubiksCube& cube,
      unsigned goalNum, vector<MOVE>& allMoves, vector<MOVE>& goalMoves);

  public:
    atomic_bool solving;
    virtual void solveCube(RubiksCube& cube) = 0;
    CubeSolver(RubiksCubeModel* pCube, ThreadPool* pThreadPool);
    virtual void initialize(std::function<void()> onInitialized);
    vector<string> simplifyMoves(const vector<string>& moves) const;
  };
}

#endif

