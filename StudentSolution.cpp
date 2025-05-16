#include "acequia_manager.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>

// -----------------------------------------------------------------------------
// Basic water distribution solver for an Acequia irrigation system.
//
// This function runs a simple greedy algorithm each simulation hour:
//  1. Identify regions that need water (deficit) and regions that have extra (surplus).
//  2. Prioritize the largest deficits first to minimize penalties quickly.
//  3. Open canals and transfer as much water as possible from donors to needy regions.
//  4. Stop early if no transfers are possible, or all regions are satisfied.
//
// Key parameters:
//  - kEpsilon: tolerance for negligible amounts (avoids pointless micro-transfers).
//  - kSafetyMargin: fraction of capacity kept in donor regions to prevent creating new deficits.
//  - kMaxLoops: safety cap on inner loops to guard against thrashing.
//
// 
// -----------------------------------------------------------------------------

namespace {

constexpr double kEpsilon      = 1e-3;  // ignore amounts smaller than this
constexpr double kSafetyMargin = 0.10;  // donors keep 10% capacity as buffer
constexpr int    kMaxLoops     = 1000;  // max iterations per hour loop

// Compute how much extra water a region can safely give up.
double safeSurplus(const Region* r) {
    double extra = r->waterLevel - r->waterNeed;
    double buffer = kSafetyMargin * r->waterCapacity;
    return std::max(0.0, extra - buffer);
}

// Compute how much water a region still needs.
double getDeficit(const Region* r) {
    return std::max(0.0, r->waterNeed - r->waterLevel);
}

// Simple struct for the priority queue of needs.
struct Need {
    Region* region;  // region with deficit
    double  amount;  // how much it still needs
    // larger deficits have higher priority
    bool operator<(const Need& o) const { return amount < o.amount; }
};

} 
void solveProblems(AcequiaManager& manager) {
    const int maxHours = manager.SimulationMax;
    const auto& canals = manager.getCanals();

    // Build a lookup for canals between any two regions.
    std::unordered_map<Region*, std::unordered_map<Region*, std::vector<Canal*>>> canalMap;
    for (Canal* c : canals) {
        canalMap[c->sourceRegion][c->destinationRegion].push_back(c);
    }

    // Run until all regions are satisfied or we hit max hours.
    while (!manager.solved() && manager.hour < maxHours) {
        const auto& regions = manager.getRegions();

        // Reset all canals (closed, zero flow) at the start of each hour.
        for (Canal* c : canals) {
            if (c->isOpen) c->toggleOpen(false);
            c->setFlowRate(0.0);
        }

        // Gather regions into two groups: needy (deficit) and donors (surplus).
        std::priority_queue<Need> needs;
        std::vector<Region*> donors;
        for (Region* r : regions) {
            double d = getDeficit(r);
            if (d > kEpsilon) {
                needs.push({r, d});
            } else {
                double s = safeSurplus(r);
                if (s > kEpsilon) donors.push_back(r);
            }
        }

        bool didTransfer = false;
        int loops = 0;

        // Greedy inner loop: always serve the region with the highest deficit.
        while (!needs.empty() && !donors.empty() && loops++ < kMaxLoops) {
            auto curr = needs.top();
            needs.pop();
            Region* target = curr.region;
            double deficit = curr.amount;

            // Attempt to draw water from each donor region.
            for (Region* src : donors) {
                double avail = safeSurplus(src);
                if (avail <= kEpsilon) continue;

                // Check if there's a canal from src to target.
                auto it = canalMap[src].find(target);
                if (it == canalMap[src].end()) continue;

                for (Canal* canal : it->second) {
                    WaterSource* ws = canal->waterSource;
                    if (!ws || ws->waterLevel <= kEpsilon) continue;

                    // Determine how much we can move this time.
                    double space = target->waterCapacity - target->waterLevel;
                    double xfer = std::min({deficit, avail, ws->waterLevel, space});
                    if (xfer <= kEpsilon) continue;

                    // Execute the transfer.
                    canal->toggleOpen(true);
                    canal->setFlowRate(xfer / 3600.0);  // convert to m³/s
                    ws->updateWaterLevel(-xfer);
                    src->updateWaterLevel(-xfer);
                    target->updateWaterLevel(xfer);

                    deficit -= xfer;
                    didTransfer = true;
                    if (deficit <= kEpsilon) break;
                }
                if (deficit <= kEpsilon) break;
            }

            // If the region still needs water, requeue for another pass.
            if (deficit > kEpsilon) {
                needs.push({target, deficit});
            }
        }

        // If nothing moved, bail out early to avoid wasted cycles.
        if (!didTransfer) break;

        manager.nexthour();
    }
}
