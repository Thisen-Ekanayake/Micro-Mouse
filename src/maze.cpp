#include "maze.h"
#include "config.h"
#include "tof_sensor.h"
#include "imu.h"
#include "encoder.h"
#include "motion.h"
#include "Arduino.h"
#include <queue>
#include <stack>

namespace maze {

// ----- internal map -----
struct Cell {
  uint8_t walls; // bit0=N, bit1=E, bit2=S, bit3=W
  bool known;
  bool visited;
  int dist;
  Cell(): walls(0), known(false), visited(false), dist(-1) {}
};

static Cell mapGrid[MAZE_SIZE][MAZE_SIZE];
static int origin = MAZE_SIZE/2; // center of map -> starting location
static int cx = 0, cy = 0; // logical coords relative to origin
static Dir cdir = NORTH;
static uint16_t FRONT_THRESH = 180; // mm, tune
static uint16_t SIDE_THRESH  = 40; // mm, tune

// helpers
inline void setWallAbs(int gx, int gy, Dir d) {
  if (gx < 0 || gy < 0 || gx >= MAZE_SIZE || gy >= MAZE_SIZE) return;
  mapGrid[gx][gy].walls |= (1 << d);
  mapGrid[gx][gy].known = true;
}
inline bool hasWallAbs(int gx, int gy, Dir d) {
  if (gx < 0 || gy < 0 || gx >= MAZE_SIZE || gy >= MAZE_SIZE) return true; // treat out-of-bounds as wall
  return (mapGrid[gx][gy].walls & (1<<d)) != 0;
}
inline Dir opposite(Dir d) { return Dir((d + 2) % 4); }
inline void neighborDelta(Dir d, int &dx, int &dy) {
  if (d==NORTH) { dx=0; dy=1; }
  else if (d==EAST) { dx=1; dy=0; }
  else if (d==SOUTH) { dx=0; dy=-1; }
  else { dx=-1; dy=0; }
}

// convert local (cx,cy) to grid indices
inline int gx_of(int x) { return x + origin; }
inline int gy_of(int y) { return y + origin; }

// sense and update walls using ToF sensors (left, right, front mm)
static void senseAndUpdateWalls() {
  uint16_t left, right, front;
  readToFDistance(left, right, front); // returns mm, or large num on timeout

  bool wFront = (front < FRONT_THRESH);
  bool wLeft  = (left  < SIDE_THRESH);
  bool wRight = (right < SIDE_THRESH);

  int gxc = gx_of(cx), gyc = gy_of(cy);
  mapGrid[gxc][gyc].known = true;

  Dir fdir = cdir;
  Dir ldir = Dir((cdir + 3) % 4);
  Dir rdir = Dir((cdir + 1) % 4);
  Dir bdir = opposite(cdir);

  // set/clear front
  if (wFront) {
    setWallAbs(gxc, gyc, fdir);
    int dx,dy; neighborDelta(fdir, dx, dy);
    setWallAbs(gxc + dx, gyc + dy, bdir);
  } else {
    // ensure we clear the wall bit if previously marked incorrectly
    // (only clear if neighbor exists)
    int dx,dy; neighborDelta(fdir, dx, dy);
    int ngx = gxc + dx, ngy = gyc + dy;
    if (ngx>=0 && ngy>=0 && ngx<MAZE_SIZE && ngy<MAZE_SIZE) {
      mapGrid[gxc][gyc].walls &= ~(1<<fdir);
    }
  }

  if (wLeft) {
    setWallAbs(gxc, gyc, ldir);
    int dx,dy; neighborDelta(ldir, dx, dy);
    setWallAbs(gxc + dx, gyc + dy, opposite(ldir));
  } else {
    int dx,dy; neighborDelta(ldir, dx, dy);
    int ngx = gxc + dx, ngy = gyc + dy;
    if (ngx>=0 && ngy>=0 && ngx<MAZE_SIZE && ngy<MAZE_SIZE) {
      mapGrid[gxc][gyc].walls &= ~(1<<ldir);
    }
  }

  if (wRight) {
    setWallAbs(gxc, gyc, rdir);
    int dx,dy; neighborDelta(rdir, dx, dy);
    setWallAbs(gxc + dx, gyc + dy, opposite(rdir));
  } else {
    int dx,dy; neighborDelta(rdir, dx, dy);
    int ngx = gxc + dx, ngy = gyc + dy;
    if (ngx>=0 && ngy>=0 && ngx<MAZE_SIZE && ngy<MAZE_SIZE) {
      mapGrid[gxc][gyc].walls &= ~(1<<rdir);
    }
  }
}

// primitive: turn to target Dir using smallest rotations
static void turnToDir(Dir target) {
  int diff = (int(target) - int(cdir) + 4) % 4;
  if (diff == 0) return;
  else if (diff == 1) {
    rotate_90_right();
    cdir = Dir((cdir + 1) % 4);
  } else if (diff == 3) {
    rotate_90_left();
    cdir = Dir((cdir + 3) % 4);
  } else { // 180
    rotate_90_right();
    delay(80); // small pause
    rotate_90_right();
    cdir = Dir((cdir + 2) % 4);
  }
}

// primitive: drive one cell forward (updates cx,cy). Uses existing move_forward_cm()
// returns true if moved; false if blocked (unexpectedly)
static bool driveOneCellWithSafety() {
  // before moving re-check front quickly
  uint16_t l,r,f;
  readToFDistance(l,r,f);
  if (f < FRONT_THRESH) {
    // Unexpected obstacle right at front - mark wall and abort
    int gxc = gx_of(cx), gyc = gy_of(cy);
    setWallAbs(gxc, gyc, cdir);
    int dx,dy; neighborDelta(cdir, dx, dy);
    setWallAbs(gxc + dx, gyc + dy, opposite(cdir));
    return false;
  }

  // Move forward one cell length
  move_forward_cm(CELL_SIZE_CM);

  // assume success, update logical coords
  int dx,dy; neighborDelta(cdir, dx, dy);
  cx += dx; cy += dy;

  // after entering new cell: small pause and sense
  delay(40);
  senseAndUpdateWalls();

  // mark visited
  int gxc = gx_of(cx), gyc = gy_of(cy);
  mapGrid[gxc][gyc].visited = true;
  mapGrid[gxc][gyc].known = true;

  return true;
}

// Exploration: right-hand priority with backtracking stack
void startExploration() {
  // initialize pose to center
  cx = 0; cy = 0; cdir = NORTH;
  for (int i=0;i<MAZE_SIZE;i++) for (int j=0;j<MAZE_SIZE;j++) mapGrid[i][j] = Cell();

  int gxc = gx_of(cx), gyc = gy_of(cy);
  mapGrid[gxc][gyc].known = true;
  mapGrid[gxc][gyc].visited = true;

  // initial sensor snapshot
  senseAndUpdateWalls();

  std::stack<Pose> backtrack;

  // exploration loop
  while (true) {
    // choose ordered tries: Right, Forward, Left, Back
    Dir tries[4] = { Dir((cdir+1)%4), cdir, Dir((cdir+3)%4), Dir((cdir+2)%4) };
    bool moved = false;

    for (int i=0;i<4;i++) {
      Dir nd = tries[i];
      int dx,dy; neighborDelta(nd, dx, dy);
      int ngx = gx_of(cx + dx), ngy = gy_of(cy + dy);

      if (hasWallAbs(gx_of(cx), gy_of(cy), nd)) continue;

      // if unexplored neighbor OR not visited, prefer it
      if (!mapGrid[ngx][ngy].known || !mapGrid[ngx][ngy].visited) {
        // rotate to nd then drive
        turnToDir(nd);
        // push current pose to backtrack (we will come back here if needed)
        backtrack.push({cx, cy, cdir});
        bool ok = driveOneCellWithSafety();
        if (!ok) {
          // obstacle discovered during drive; map updated in driveOneCellWithSafety
          // continue exploring current cell
        } else {
          moved = true;
        }
        break;
      }
    }

    if (moved) continue;

    // no unexplored neighbor - try to move to any non-wall neighbor (for connectivity)
    for (int i=0;i<4;i++) {
      Dir nd = tries[i];
      int dx,dy; neighborDelta(nd, dx, dy);
      int ngx = gx_of(cx + dx), ngy = gy_of(cy + dy);
      if (!hasWallAbs(gx_of(cx), gy_of(cy), nd)) {
        // move if not same cell
        turnToDir(nd);
        bool ok = driveOneCellWithSafety();
        if (ok) { moved = true; break; }
      }
    }
    if (moved) continue;

    // fully explored from this cell - backtrack using stack
    if (backtrack.empty()) break; // exploration complete
    Pose p = backtrack.top(); backtrack.pop();

    // move towards p (p should be adjacent to current). Compute delta
    int ddx = p.x - cx;
    int ddy = p.y - cy;
    Dir need;
    if (ddx==1) need = EAST; else if (ddx==-1) need = WEST;
    else if (ddy==1) need = NORTH; else need = SOUTH;
    turnToDir(need);
    bool ok = driveOneCellWithSafety();
    if (!ok) {
      // if we can't go back, continue loop and rely on further stack entries
    }
  } // while

  Serial.println("Exploration finished.");
}

// BFS shortest path (cells must be known to be usable)
std::vector<Pose> computeShortestPath(int sx, int sy, int tx, int ty) {
  std::vector<Pose> out;
  // grid indices
  int gsx = gx_of(sx), gsy = gy_of(sy);
  int gtx = gx_of(tx), gty = gy_of(ty);

  // reset distances and parents
  static int parentX[MAZE_SIZE][MAZE_SIZE];
  static int parentY[MAZE_SIZE][MAZE_SIZE];
  for (int i=0;i<MAZE_SIZE;i++) for (int j=0;j<MAZE_SIZE;j++) {
    mapGrid[i][j].dist = -1;
    parentX[i][j] = parentY[i][j] = -9999;
  }

  std::queue<std::pair<int,int>> q;
  mapGrid[gsx][gsy].dist = 0;
  q.push({gsx,gsy});

  while (!q.empty()) {
    auto [x,y] = q.front(); q.pop();
    if (x==gtx && y==gty) break;
    for (int di=0; di<4; ++di) {
      Dir d = Dir(di);
      if (hasWallAbs(x,y,d)) continue;
      int dx,dy; neighborDelta(d, dx, dy);
      int nx = x + dx, ny = y + dy;
      if (nx<0||ny<0||nx>=MAZE_SIZE||ny>=MAZE_SIZE) continue;
      if (!mapGrid[nx][ny].known) continue; // don't go into unknown
      if (mapGrid[nx][ny].dist != -1) continue;
      mapGrid[nx][ny].dist = mapGrid[x][y].dist + 1;
      parentX[nx][ny] = x; parentY[nx][ny] = y;
      q.push({nx,ny});
    }
  }

  if (mapGrid[gtx][gty].dist == -1) {
    // no path
    return out;
  }

  // reconstruct from goal to source
  int cxg = gtx, cyg = gty;
  std::vector<std::pair<int,int>> rev;
  while (!(cxg==gsx && cyg==gsy)) {
    rev.push_back({cxg, cyg});
    int px = parentX[cxg][cyg];
    int py = parentY[cxg][cyg];
    cxg = px; cyg = py;
  }
  rev.push_back({gsx, gsy});

  // convert to Pose list in forward order (source->target), convert grid coords back to logical coords
  for (int i = int(rev.size()) - 1; i >= 0; --i) {
    int gx = rev[i].first, gy = rev[i].second;
    int lx = gx - origin, ly = gy - origin;
    Dir dir = NORTH;
    if (i < int(rev.size()) - 1) {
      // compute heading to next cell
      int nx = rev[i+1].first - gx;
      int ny = rev[i+1].second - gy;
      if (ny == 1) dir = NORTH;
      else if (nx == 1) dir = EAST;
      else if (ny == -1) dir = SOUTH;
      else if (nx == -1) dir = WEST;
    } else {
      // last cell (target) use current cdir as placeholder
      dir = NORTH;
    }
    out.push_back({lx, ly, dir});
  }

  return out;
}

// execute a path (poses are cell centers in sequence). path must start from current cell.
void executePath(const std::vector<Pose>& path) {
  if (path.size() <= 1) return;
  // current logical pose assumed to match first element in path
  for (size_t i=1;i<path.size(); ++i) {
    Pose next = path[i];
    int dx = next.x - cx;
    int dy = next.y - cy;
    Dir need;
    if (dx == 1) need = EAST;
    else if (dx == -1) need = WEST;
    else if (dy == 1) need = NORTH;
    else need = SOUTH;

    turnToDir(need);
    bool ok = driveOneCellWithSafety();
    if (!ok) {
      // unexpected obstacle while executing - replan required
      Serial.println("Obstacle while following path - aborting and replanning.");
      return;
    }
  }
}

void runToGoal(int sx, int sy, int tx, int ty) {
  auto path = computeShortestPath(sx,sy,tx,ty);
  if (path.empty()) {
    Serial.println("No path found.");
    return;
  }
  // ensure current pose is at path[0]
  executePath(path);
}

// exposed API
void begin() {
  // initialize sensors / state needed by maze module
  initToFSensor();
  imu_init();
  // assume encoders and motors were already init in main
  cx = 0; cy = 0; cdir = NORTH;
  // clear map
  for (int i=0;i<MAZE_SIZE;i++) for (int j=0;j<MAZE_SIZE;j++) mapGrid[i][j] = Cell();
}

void setFrontThresholdMm(uint16_t mm) { FRONT_THRESH = mm; }
void setSideThresholdMm(uint16_t mm) { SIDE_THRESH = mm; }

} // namespace maze
