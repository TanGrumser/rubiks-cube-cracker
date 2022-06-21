#include "Controller/Command/Solver/KorfCubeSolver.h"
#include "Util/ThreadPool.h"
#include <memory>

using namespace busybin;

CubeSolver* solver;

void solve() {
  RubiksCube* cube = new RubiksCubeIndexModel();
  cube->u();
  cube->l();
  cube->d();
  cube->r();
  cube->u();
  cube->l();
  cube->d();
  cube->r();
  cube->u();
  cube->l();
  cube->d();
  cube->r();
  cube->u();
  cube->l();

  solver->solveCube(*cube);
}
/**
 * Bootstrap the application.
 */
int main(int argc, char* argv[]) {
  ThreadPool* threadPool = new ThreadPool(1);
  solver = new KorfCubeSolver(nullptr, threadPool);

  solver->initialize(solve);

  while(1) {}

  return 0;
}


