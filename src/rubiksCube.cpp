#include "Controller/Command/Solver/KorfCubeSolver.h"
#include "Util/ThreadPool.h"
#include <memory>
#include "Util/StringUtils.h"

using namespace busybin;
#define MOVE busybin::RubiksCube::MOVE

MOVE parseMove(string moveString);
void handleCommandLineAruments(int argc, char *argv[], RubiksCube* cube);

CubeSolver* solver;

void solve(RubiksCube& cube) {
  
  solver->solveCube(cube);
}
/**
 * Bootstrap the application.
 */
int main(int argc, char* argv[]) {
  ThreadPool* threadPool = new ThreadPool(4);
  solver = new KorfCubeSolver(nullptr, threadPool);
  RubiksCube* cube = new RubiksCubeIndexModel();
  
  handleCommandLineAruments(argc, argv, cube);

  solver->initialize([cube]() {
    solve(*cube);
  });

  while(solver->solving) {}

  return 0;
}

void handleCommandLineAruments(int argc, char *argv[], RubiksCube* cube) {

    //Parse all command line arguments.
    for (int i = 1; i < argc; i++) {
        if (((string) argv[i]).compare("-t") == 0) {
            string turnString = (string) argv[i + 1];
            vector<string> turns = StringUtils::Split(turnString, " ");
            
            for (int i = 0; i < turns.size(); i++) {
                MOVE turn = parseMove(turns[i]);
                cube->move(turn);
            }
        }
    }
}

MOVE parseMove(string moveString) {
  #define MOVE_COMPARE(first, second, value) if (moveString.compare(first) == 0 || moveString.compare(second) == 0) return value
  
  MOVE_COMPARE("U",  "u",  MOVE::U);
  MOVE_COMPARE("U'", "u'", MOVE::UPRIME);
  MOVE_COMPARE("U2", "u2", MOVE::U2);

  MOVE_COMPARE("F",  "f",  MOVE::L);
  MOVE_COMPARE("F'", "f'", MOVE::LPRIME);
  MOVE_COMPARE("F2", "f2", MOVE::L2);
  
  MOVE_COMPARE("R",  "r",  MOVE::R);
  MOVE_COMPARE("R'", "r'", MOVE::RPRIME);
  MOVE_COMPARE("R2", "r2", MOVE::R2);
  
  MOVE_COMPARE("B",  "b",  MOVE::B);
  MOVE_COMPARE("B'", "b'", MOVE::BPRIME);
  MOVE_COMPARE("B2", "b2", MOVE::B2);
  
  MOVE_COMPARE("L",  "l",  MOVE::L);
  MOVE_COMPARE("L'", "l'", MOVE::LPRIME);
  MOVE_COMPARE("L2", "l2", MOVE::L2);
  
  MOVE_COMPARE("D",  "d",  MOVE::D);
  MOVE_COMPARE("D'", "d'", MOVE::DPRIME);
  MOVE_COMPARE("D2", "d2", MOVE::D2);

  return (MOVE)0xFF;
}