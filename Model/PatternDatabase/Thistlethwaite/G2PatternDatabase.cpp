#include "G2PatternDatabase.h"

namespace busybin
{
  /**
   * Initialize the database storage.
   *
   * For the four edges, there are 12P4 permutations (4 edges and 12
   * positions).
   *
   * There are 8 corners, and the orientations of 7 dictate the orientation of
   * the 8th (even parity).  Corners have three possible orientations, so there
   * are 3^7 corner orientation permutations.
   *
   * 12P4 * 3^7 / 1024^2 / 2 = 25981560 / 1024^2 / 2 = ~12.39MB on disk.
   */
  G2PatternDatabase::G2PatternDatabase() : PatternDatabase(25981560)
  {
  }

  /**
   * Given a cube, get an index into the pattern database.
   */
  uint32_t G2PatternDatabase::getDatabaseIndex(const RubiksCube& cube) const
  {
    typedef RubiksCubeIndexModel::EDGE   EDGE;
    typedef RubiksCubeIndexModel::CORNER CORNER;

    const RubiksCubeIndexModel& iCube = static_cast<const RubiksCubeIndexModel&>(cube);

    const uint8_t numEdges = 12;
    const array<EDGE, 4> edges = {EDGE::UB, EDGE::UF, EDGE::DF, EDGE::DB};

    // Create a permutation array consisting of 4 of the 12 edges by looping
    // over all edge piece until the 4 in the M slice are found.  The
    // permutation is made up of the edges' positions, 0-11.
    array<uint8_t, 4> edgePerm;
    unsigned          numIndexed = 0;

    for (uint8_t i = 0; i < numEdges && numIndexed != 4; ++i)
    {
      uint8_t edgeInd = iCube.getEdgeIndex((EDGE)i);

      for (uint8_t j = 0; j < 4; ++j)
      {
        if (edgeInd == (uint8_t)edges[j])
        {
          // E.g. edge UB (edgePerm[0]) is in the DR (11) position.
          // E.g. edge UF (edgePerm[1]) is in the UR (1) position.
          edgePerm[j] = i;
          ++numIndexed;
          break;
        }
      }
    }

    uint32_t rank = this->permIndexer.rank(edgePerm);

    // Now get the orientation of the corners.  7 corner orientations dictate
    // the orientation of the 8th, so only 7 need to be stored.
    array<uint8_t, 7> cornerOrientations =
    {
      iCube.getCornerOrientation(CORNER::ULB),
      iCube.getCornerOrientation(CORNER::URB),
      iCube.getCornerOrientation(CORNER::URF),
      iCube.getCornerOrientation(CORNER::ULF),
      iCube.getCornerOrientation(CORNER::DLF),
      iCube.getCornerOrientation(CORNER::DLB),
      iCube.getCornerOrientation(CORNER::DRB)
    };

    // Treat the orientations as a base-3 number, and convert it
    // to base-10.
    uint32_t orientationNum =
      cornerOrientations[0] * 729 +
      cornerOrientations[1] * 243 +
      cornerOrientations[2] * 81 +
      cornerOrientations[3] * 27 +
      cornerOrientations[4] * 9 +
      cornerOrientations[5] * 3 +
      cornerOrientations[6];

    // Combine the two (3^7 == 2187).
    return rank * 2187 + orientationNum;
  }
}
