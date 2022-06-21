#include "CubeSolver.h"

namespace busybin
{
  /**
   * Init.
   * @param pWorld Pointer to the world (must remain in scope).
   * @param pWorldWnd The world window, used to bind key and pulse events.
   * @param pMover Pointer to the CubeMover command.
   * @param pThreadPool A ThreadPool pointer for queueing jobs.
   * @param solveKey The GLFW key that triggers the solver to start.
   */
  CubeSolver::CubeSolver(RubiksCubeModel* pCube, ThreadPool* pThreadPool) :
    pCube(pCube),
    pThreadPool(pThreadPool), 
    solving(false),
    movesInQueue(false),
    moveTimer(false)
  {
  }

  /**
   * This can be overridden in sub classes and gives solvers the chance to
   * initialize pattern databases and such (whatever's needed for the solver).
   * It's launched in a thread.
   */
  void CubeSolver::initialize(std::function<void()> onInitialized)
  {
    
  }

 
  /**
   * Put the cube in a "solving" state, which disables cube movement.  In the
   * initialization phase (when pattern databases are being indexed) the cube
   * is put in a solving state, as well as when the user triggers a solve by
   * pressing the solve key (F1, F2, etc.).
   * @param solving Whether or not the cube is being solved.  When so, cube movement
   *        is disabled.
   */
  void CubeSolver::setSolving(bool solving)
  {
    this->solving = solving;
  }

  /**
   * Helper function to process moves after a goal is achived.
   * @param goal The goal for verbosity.
   * @param cube The RC model copy.  The goalMoves will be applied.
   * @param goalNum The goal number for verbosity.
   * @param allMoves This vector holds all the moves thus far.  The
   *        goalMoves vector will be appended to it.
   * @param goalMoves This vector holds the moves required to achieve
   *        the goal.  These moves will be queued for the GL cube to
   *        display, then the vector will be cleared.
   */
  void CubeSolver::processGoalMoves(const Goal& goal, RubiksCube& cube,
    unsigned goalNum, vector<MOVE>& allMoves, vector<MOVE>& goalMoves)
  {
    cout << "Found goal " << goalNum << ": " << goal.getDescription() << '\n' << endl;

    // Add goalMoves to the end of allMoves.
    allMoves.insert(allMoves.end(), goalMoves.begin(), goalMoves.end());

    for (MOVE move : goalMoves)
    {
      // Lock the move mutex so that onPulse doesn't simultaneously mangle
      // the move queue.
      lock_guard<mutex> threadLock(this->moveMutex);

      // The RC model needs to be kept in sync as it is a copy of the actual RC
      // model.
      cube.move(move);

      // Queue this move for the GL cube to render.
      this->moveQueue.push(move);
      this->movesInQueue = true;
    }

    // Clear the vector for the next goal.
    goalMoves.clear();
  }

  /**
   * Reduce moves.  For example, L2 L2 can be removed.  L L L is the same as L'.
   * etc.
   * @param moves The set of moves required to solve the cube.
   */
  vector<string> CubeSolver::simplifyMoves(const vector<string>& moves) const
  {
    string        movesStr = "";
    istringstream stream;

    for (string move : moves)
      movesStr += move + " ";

    this->replace("U2 U2 ", movesStr, " ");
    this->replace("L2 L2 ", movesStr, " ");
    this->replace("F2 F2 ", movesStr, " ");
    this->replace("R2 R2 ", movesStr, " ");
    this->replace("B2 B2 ", movesStr, " ");
    this->replace("D2 D2 ", movesStr, " ");

    this->replace("U U' ", movesStr, " ");
    this->replace("L L' ", movesStr, " ");
    this->replace("F F' ", movesStr, " ");
    this->replace("R R' ", movesStr, " ");
    this->replace("B B' ", movesStr, " ");
    this->replace("D D' ", movesStr, " ");

    this->replace("U' U ", movesStr, " ");
    this->replace("L' L ", movesStr, " ");
    this->replace("F' F ", movesStr, " ");
    this->replace("R' R ", movesStr, " ");
    this->replace("B' B ", movesStr, " ");
    this->replace("D' D ", movesStr, " ");

    this->replace("U U U ", movesStr, "U' ");
    this->replace("L L L ", movesStr, "L' ");
    this->replace("F F F ", movesStr, "F' ");
    this->replace("R R R ", movesStr, "R' ");
    this->replace("B B B ", movesStr, "B' ");
    this->replace("B B B ", movesStr, "B' ");

    this->replace("U U ", movesStr, "U2 ");
    this->replace("L L ", movesStr, "L2 ");
    this->replace("F F ", movesStr, "F2 ");
    this->replace("R R ", movesStr, "R2 ");
    this->replace("B B ", movesStr, "B2 ");
    this->replace("D D ", movesStr, "D2 ");

    // Copy the moves back to a vector.
    stream.str(movesStr);
    return vector<string>(istream_iterator<string>(stream), istream_iterator<string>());
  }

  /**
   * Helper for replacing in a string.
   * @param needle What to replace,
   * @param haystack Where to search for needle.
   * @param with What to replace needle with.
   */
  void CubeSolver::replace(const string& needle, string& haystack, const string& with) const
  {
    string::size_type pos;

    while ((pos = haystack.find(needle)) != string::npos)
    {
      cout << "Found " << needle << " in " << haystack << " at position " << pos
           << ".  Replacing with \"" << with << "\"." << endl;
      haystack.replace(pos, needle.length(), with);
    }
  }
}

