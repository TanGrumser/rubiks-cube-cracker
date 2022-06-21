#include "KorfCubeSolver.h"

namespace busybin
{
  /**
   * Init.
   * @param pWorld Pointer to the world (must remain in scope).
   * @param pWorldWnd The world window, used to bind key and pulse events.
   * @param pMover Pointer to the CubeMover command.
   * @param pThreadPool A ThreadPool pointer for queueing jobs.
   */
  KorfCubeSolver::KorfCubeSolver(RubiksCubeModel* pCube, ThreadPool* pThreadPool) :
    CubeSolver(pCube, pThreadPool),
    cornerDB(),
    edgeG1DB(),
    edgeG2DB(),
    edgePermDB(),
    korfDB(&cornerDB, &edgeG1DB, &edgeG2DB, &edgePermDB),
    cornerDBIndexed(false),
    edgeG1DBIndexed(false),
    edgeG2DBIndexed(false),
    edgePermDBIndexed(false) 
  {
  }

  /**
   * Launch a thread to initialize the pattern databases. 
   */
  void KorfCubeSolver::initialize(std::function<void()> onInitialized)
  {
    CubeSolver::initialize(onInitialized);
    this->onInitialized = onInitialized;

    cout << "Initializing pattern databases for KorfCubeSolver." << endl;

    // Index each pattern database.
    this->pThreadPool->addJob(bind(&KorfCubeSolver::indexCornerDatabase, this));
    this->pThreadPool->addJob(bind(&KorfCubeSolver::indexEdgeG1Database, this));
    this->pThreadPool->addJob(bind(&KorfCubeSolver::indexEdgeG2Database, this));
    this->pThreadPool->addJob(bind(&KorfCubeSolver::indexEdgePermDatabase, this));
  }

  /**
   * Index the corner database.
   */
  void KorfCubeSolver::indexCornerDatabase()
  {
    // The corner pattern database will be created using a breadth-first
    // search.
    BreadthFirstCubeSearcher bfsSearcher;

    // An index model is used for building pattern databases.
    RubiksCubeIndexModel iCube;

    this->setSolving(true);

    // The seacher uses about 5GB of memory; the internal queue is quite large
    // while indexing the corner database.
    if (!this->cornerDB.fromFile("../Data/corner.pdb"))
    {
      // First create the corner database.
      CornerDatabaseGoal cornerGoal(&this->cornerDB);
      TwistStore         twistStore(iCube);

      cout << "Goal 1: " << cornerGoal.getDescription() << endl;

      bfsSearcher.findGoal(cornerGoal, iCube, twistStore);
      this->cornerDB.toFile("../Data/corner.pdb");
    }

    this->cornerDBIndexed = true;
    this->onIndexComplete();
  }

  /**
   * Index the first edge database.
   */
  void KorfCubeSolver::indexEdgeG1Database()
  {
    // The corner databases are indexed using a specialized IDDFS search.
    PatternDatabaseIndexer indexer;
    RubiksCubeIndexModel   iCube;

    this->setSolving(true);

    if (!this->edgeG1DB.fromFile("../Data/edgeG1.pdb"))
    {
      // Create the first edge database.
      EdgeDatabaseGoal      edgeG1Goal(&this->edgeG1DB);
      TwistStore            twistStore(iCube);

      cout << "Goal 2: " << edgeG1Goal.getDescription() << endl;

      indexer.findGoal(edgeG1Goal, iCube, twistStore);
      this->edgeG1DB.toFile("../Data/edgeG1.pdb");
    }

    this->edgeG1DBIndexed = true;
    this->onIndexComplete();
  }

  /**
   * Index the second edge database.
   */
  void KorfCubeSolver::indexEdgeG2Database()
  {
    PatternDatabaseIndexer indexer;
    RubiksCubeIndexModel   iCube;

    this->setSolving(true);

    if (!this->edgeG2DB.fromFile("../Data/edgeG2.pdb"))
    {
      // Create the second edge database.
      EdgeDatabaseGoal      edgeG2Goal(&this->edgeG2DB);
      TwistStore            twistStore(iCube);

      cout << "Goal 3: " << edgeG2Goal.getDescription() << endl;

      indexer.findGoal(edgeG2Goal, iCube, twistStore);
      this->edgeG2DB.toFile("../Data/edgeG2.pdb");
    }

    this->edgeG2DBIndexed = true;
    this->onIndexComplete();
  }

  /**
   * Index the edge permutation database.
   */
  void KorfCubeSolver::indexEdgePermDatabase()
  {
    PatternDatabaseIndexer indexer;
    RubiksCubeIndexModel   iCube;

    this->setSolving(true);

    if (!this->edgePermDB.fromFile("../Data/edge_perm.pdb"))
    {
      // Create the edge permutation database.
      EdgePermutationDatabaseGoal    edgePermGoal(&this->edgePermDB);
      TwistStore                     twistStore(iCube);

      cout << "Goal 4: " << edgePermGoal.getDescription() << endl;

      indexer.findGoal(edgePermGoal, iCube, twistStore);
      this->edgePermDB.toFile("../Data/edge_perm.pdb");
    }

    this->edgePermDBIndexed = true;
    this->onIndexComplete();
  }

  /**
   * Each index thread calls this when it's complete.  When all are done,
   * the Korf database is inflated and the solving flag is toggled off.
   */
  void KorfCubeSolver::onIndexComplete()
  {
    if (this->cornerDBIndexed && this->edgeG1DBIndexed && this->edgeG2DBIndexed && this->edgePermDBIndexed)
    {
      // Inflate the DB for faster access (doubles the size, but no bit-wise
      // operations are required when indexing).
      this->korfDB.inflate();

      this->setSolving(false);

      cout << "Korf initialization complete." << endl;
      onInitialized();
    }
  }

  /**
   * Solve the cube.  This is run in a separate thread.
   */
  void KorfCubeSolver::solveCube(RubiksCube& cube)
  {
    vector<MOVE>   allMoves;
    vector<MOVE>   goalMoves;

    cout << "Solving with Korf method." << endl;

    cout << "Initial cube state." << endl;

    /*
    // First goal: orient the cube with red up and white front.
    BreadthFirstCubeSearcher bfsSearcher;
    OrientGoal               orientGoal;
    RotationStore            rotStore(cube);

    goalMoves = bfsSearcher.findGoal(orientGoal, cube, rotStore);

    this->processGoalMoves(orientGoal, cube, 1, allMoves, goalMoves);
    */

    // Second goal: solve the cube.
    IDACubeSearcher      idaSearcher(&this->korfDB);
    SolveGoal            solveGoal;
    TwistStore           twistStore(cube);

    goalMoves = idaSearcher.findGoal(solveGoal, cube, twistStore);
    this->processGoalMoves(solveGoal, cube, 2, allMoves, goalMoves);

    // Print the moves.
    cout << "\n\nSolved the cube in " << allMoves.size() << " moves.\n";

    for (MOVE move : allMoves)
      cout << this->pCube->getMove(move) << ' ';
    cout << endl;

    // Display the cube model.
    cout << "Resulting cube.\n";

    // Done solving - re-enable movement.  (Note that solving is set to true in
    // the parent class on keypress.)
    this->setSolving(false);
  }
}

