// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgOptimization.h"
#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/Core/Profiler.h"
#include "Urho3D/Math/MathDefs.h"
#include "Urho3D/Math/Plane.h"
#include "Urho3D/Math/Vector2.h"
#include "Urho3D/Math/Vector3.h"
#include "Urho3D/Math/BoundingBox.h"

#include <EASTL/algorithm.h>
#include <EASTL/fixed_vector.h>
#include <EASTL/hash_map.h>
#include <EASTL/hash_set.h>
#include <EASTL/sort.h>

#include <cmath>
#include <cstdint>

namespace Urho3D
{

namespace
{

#ifndef URHO3D_CSG_SIMPLIFY_DIAGNOSTICS
#define URHO3D_CSG_SIMPLIFY_DIAGNOSTICS 0
#endif

#if URHO3D_CSG_SIMPLIFY_DIAGNOSTICS
    #include "Urho3D/IO/Log.h"
    #include "Urho3D/SystemUI/ImGui.h"

static bool IsCsgSimplifyDiagnosticsActive()
{
    // Diagnostics must only show when Shift is pressed.
    // Also guard against calling ImGui when there is no current context.
    if (!ui::GetCurrentContext())
        return false;
    return ui::GetIO().KeyShift;
}

static void CsgSimplifyDiagLog(const char* text)
{
    if (IsCsgSimplifyDiagnosticsActive())
        URHO3D_LOGINFO(text);
}
#else
static bool IsCsgSimplifyDiagnosticsActive() { return false; }
static void CsgSimplifyDiagLog(const char*) {}
#endif

struct SimplePolygonFailureDetails
{
    enum class Type
    {
        None,
        DuplicateQuantizedVertex,
        SelfIntersection,
    } type_{Type::None};

    // DuplicateQuantizedVertex
    IntVector3 duplicateKey_{};
    unsigned duplicateFirstIndex_{};
    unsigned duplicateSecondIndex_{};

    // SelfIntersection
    unsigned edgeA0_{};
    unsigned edgeA1_{};
    unsigned edgeB0_{};
    unsigned edgeB1_{};
};

struct PlaneGroup
{
    Plane plane_;
    unsigned shared_{};
    ea::vector<CsgPolygon> polys_;
};

struct EdgeRef
{
    unsigned polyIndex_{};
    unsigned edgeStart_{};
};

struct CanceledEdge
{
    unsigned from_{};
    unsigned to_{};
};

struct VertexGrid
{
    float cellSize_{};
    float invCellSize_{};
    ea::hash_map<IntVector3, ea::vector<Vector3>> grid_;

    void Init(float cellSize)
    {
        cellSize_ = cellSize;
        invCellSize_ = 1.0f / cellSize;
    }

    IntVector3 Quantize(const Vector3& v) const
    {
        return IntVector3{
            static_cast<int>(std::floor(v.x_ * invCellSize_)),
            static_cast<int>(std::floor(v.y_ * invCellSize_)),
            static_cast<int>(std::floor(v.z_ * invCellSize_))};
    }

    void Add(const Vector3& v) { grid_[Quantize(v)].push_back(v); }

    template <class TVector>
    void AppendCellCandidates(const IntVector3& key, TVector& result) const
    {
        const auto it = grid_.find(key);
        if (it != grid_.end())
            result.insert(result.end(), it->second.begin(), it->second.end());
    }

    template <class TVector>
    void Query(const Vector3& p0, const Vector3& p1, TVector& result) const
    {
        result.resize(0);
        const Vector3 min(ea::min(p0.x_, p1.x_), ea::min(p0.y_, p1.y_), ea::min(p0.z_, p1.z_));
        const Vector3 max(ea::max(p0.x_, p1.x_), ea::max(p0.y_, p1.y_), ea::max(p0.z_, p1.z_));

        const int minX = static_cast<int>(std::floor(min.x_ * invCellSize_));
        const int minY = static_cast<int>(std::floor(min.y_ * invCellSize_));
        const int minZ = static_cast<int>(std::floor(min.z_ * invCellSize_));
        const int maxX = static_cast<int>(std::floor(max.x_ * invCellSize_));
        const int maxY = static_cast<int>(std::floor(max.y_ * invCellSize_));
        const int maxZ = static_cast<int>(std::floor(max.z_ * invCellSize_));

        if ((maxX - minX + 1) * (maxY - minY + 1) * (maxZ - minZ + 1) <= 8)
        {
            for (int x = minX; x <= maxX; ++x)
                for (int y = minY; y <= maxY; ++y)
                    for (int z = minZ; z <= maxZ; ++z)
                        AppendCellCandidates(IntVector3{x, y, z}, result);
            return;
        }

        // Robust grid traversal: visit every grid cell intersected by the segment.
        const Vector3 dir = p1 - p0;
        if (dir.LengthSquared() < M_EPSILON * M_EPSILON)
            return;

        // Traverse in grid space where cell boundaries are integer coordinates.
        const Vector3 g0 = p0 * invCellSize_;
        const Vector3 g1 = p1 * invCellSize_;
        const Vector3 gd = g1 - g0;

        IntVector3 cell{
            static_cast<int>(std::floor(g0.x_)),
            static_cast<int>(std::floor(g0.y_)),
            static_cast<int>(std::floor(g0.z_))};
        const IntVector3 endCell{
            static_cast<int>(std::floor(g1.x_)),
            static_cast<int>(std::floor(g1.y_)),
            static_cast<int>(std::floor(g1.z_))};

        AppendCellCandidates(cell, result);

        const float inf = 1e30f;

        const int stepX = gd.x_ > 0.0f ? 1 : (gd.x_ < 0.0f ? -1 : 0);
        const int stepY = gd.y_ > 0.0f ? 1 : (gd.y_ < 0.0f ? -1 : 0);
        const int stepZ = gd.z_ > 0.0f ? 1 : (gd.z_ < 0.0f ? -1 : 0);

        const float tDeltaX = stepX != 0 ? Abs(1.0f / gd.x_) : inf;
        const float tDeltaY = stepY != 0 ? Abs(1.0f / gd.y_) : inf;
        const float tDeltaZ = stepZ != 0 ? Abs(1.0f / gd.z_) : inf;

        const float nextBoundaryX = stepX > 0 ? (static_cast<float>(cell.x_ + 1) - g0.x_) : (g0.x_ - static_cast<float>(cell.x_));
        const float nextBoundaryY = stepY > 0 ? (static_cast<float>(cell.y_ + 1) - g0.y_) : (g0.y_ - static_cast<float>(cell.y_));
        const float nextBoundaryZ = stepZ > 0 ? (static_cast<float>(cell.z_ + 1) - g0.z_) : (g0.z_ - static_cast<float>(cell.z_));

        float tMaxX = stepX != 0 ? nextBoundaryX * tDeltaX : inf;
        float tMaxY = stepY != 0 ? nextBoundaryY * tDeltaY : inf;
        float tMaxZ = stepZ != 0 ? nextBoundaryZ * tDeltaZ : inf;

        // Safety cap: segment traversal should be bounded by number of crossed cells.
        const int maxSteps = 4 + Abs(endCell.x_ - cell.x_) + Abs(endCell.y_ - cell.y_) + Abs(endCell.z_ - cell.z_);
        for (int steps = 0; steps < maxSteps; ++steps)
        {
            if (cell == endCell)
                break;

            if (tMaxX <= tMaxY && tMaxX <= tMaxZ)
            {
                cell.x_ += stepX;
                tMaxX += tDeltaX;
            }
            else if (tMaxY <= tMaxZ)
            {
                cell.y_ += stepY;
                tMaxY += tDeltaY;
            }
            else
            {
                cell.z_ += stepZ;
                tMaxZ += tDeltaZ;
            }

            AppendCellCandidates(cell, result);
        }
    }
};

static bool IsDegeneratePolygon(const CsgPolygon& polygon)
{
    return polygon.vertices_.size() < 3;
}

static float Orient2(const Vector2& a, const Vector2& b, const Vector2& c)
{
    return (b - a).CrossProduct(c - a);
}

static unsigned GetVertexId(ea::hash_map<IntVector3, unsigned>& vertexId, const IntVector3& qp)
{
    const auto it = vertexId.find(qp);
    if (it != vertexId.end())
        return it->second;
    const unsigned id = vertexId.size();
    vertexId[qp] = id;
    return id;
}

static float ColinearAreaEpsilon(const Vector2& a, const Vector2& b, float eps)
{
    // Orient2 is an area-like value (~|ab| * distance(p, ab)).
    // If we want to treat points within `eps` distance from the segment as colinear,
    // compare |orient| against eps * |ab|.
    return eps * ea::max((b - a).Length(), 1e-6f);
}

static bool OnSegment2(const Vector2& a, const Vector2& b, const Vector2& p, float eps)
{
    if (p.x_ < ea::min(a.x_, b.x_) - eps || p.x_ > ea::max(a.x_, b.x_) + eps)
        return false;
    if (p.y_ < ea::min(a.y_, b.y_) - eps || p.y_ > ea::max(a.y_, b.y_) + eps)
        return false;
    return Abs(Orient2(a, b, p)) <= ColinearAreaEpsilon(a, b, eps);
}

static bool SegmentsIntersect2(const Vector2& a0, const Vector2& a1, const Vector2& b0, const Vector2& b1, float eps)
{
    // Bounding box quick reject.
    const float aMinX = ea::min(a0.x_, a1.x_);
    const float aMaxX = ea::max(a0.x_, a1.x_);
    const float aMinY = ea::min(a0.y_, a1.y_);
    const float aMaxY = ea::max(a0.y_, a1.y_);
    const float bMinX = ea::min(b0.x_, b1.x_);
    const float bMaxX = ea::max(b0.x_, b1.x_);
    const float bMinY = ea::min(b0.y_, b1.y_);
    const float bMaxY = ea::max(b0.y_, b1.y_);

    if (aMaxX < bMinX - eps || bMaxX < aMinX - eps || aMaxY < bMinY - eps || bMaxY < aMinY - eps)
        return false;

    const float o1 = Orient2(a0, a1, b0);
    const float o2 = Orient2(a0, a1, b1);
    const float o3 = Orient2(b0, b1, a0);
    const float o4 = Orient2(b0, b1, a1);

    const float colA = ColinearAreaEpsilon(a0, a1, eps);
    const float colB = ColinearAreaEpsilon(b0, b1, eps);

    const bool o1z = Abs(o1) <= colA;
    const bool o2z = Abs(o2) <= colA;
    const bool o3z = Abs(o3) <= colB;
    const bool o4z = Abs(o4) <= colB;

    // General case
    if (!o1z && !o2z && !o3z && !o4z)
    {
        const bool ab = (o1 > 0.0f) != (o2 > 0.0f);
        const bool ba = (o3 > 0.0f) != (o4 > 0.0f);
        return ab && ba;
    }

    // Colinear / touching cases
    if (o1z && OnSegment2(a0, a1, b0, eps)) return true;
    if (o2z && OnSegment2(a0, a1, b1, eps)) return true;
    if (o3z && OnSegment2(b0, b1, a0, eps)) return true;
    if (o4z && OnSegment2(b0, b1, a1, eps)) return true;
    return false;
}

static uint64_t PackDirectedEdgeKey(unsigned from, unsigned to)
{
    return (static_cast<uint64_t>(from) << 32u) | static_cast<uint64_t>(to);
}

static uint64_t PackUndirectedEdgeKey(unsigned a, unsigned b)
{
    const unsigned lo = ea::min(a, b);
    const unsigned hi = ea::max(a, b);
    return (static_cast<uint64_t>(lo) << 32u) | static_cast<uint64_t>(hi);
}

static uint64_t PackCellKey(int x, int y)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32u) | static_cast<uint64_t>(static_cast<uint32_t>(y));
}

static float ChooseIntersectionGridCellSize(const ea::fixed_vector<Vector2, 16, true>& p2, float intersectionEps)
{
    if (p2.empty())
        return 1.0f;

    Vector2 minP = p2[0];
    Vector2 maxP = p2[0];
    for (const Vector2& p : p2)
    {
        minP.x_ = ea::min(minP.x_, p.x_);
        minP.y_ = ea::min(minP.y_, p.y_);
        maxP.x_ = ea::max(maxP.x_, p.x_);
        maxP.y_ = ea::max(maxP.y_, p.y_);
    }

    const Vector2 ext = maxP - minP;
    const float diag = ext.Length();
    const float n = static_cast<float>(p2.size());

    // Heuristic: aim for ~sqrt(n) cells across the bbox.
    // Keep it conservative and stable to avoid missing boundary cases.
    float cell = diag > M_EPSILON && n > 1.0f ? (diag / Sqrt(n)) : 1.0f;
    cell = ea::max(cell, intersectionEps * 64.0f);
    return ea::max(cell, 1e-6f);
}

static bool HasSelfIntersections2DGrid(const ea::fixed_vector<Vector2, 16, true>& p2, float intersectionEps, SimplePolygonFailureDetails* details)
{
    const unsigned n = p2.size();
    if (n < 3)
        return true;

    // For small polygons, the O(N^2) check is fast and well-tested.
    if (n <= 24)
    {
        for (unsigned i = 0; i < n; ++i)
        {
            const unsigned i1 = (i + 1u) % n;
            for (unsigned j = i + 1u; j < n; ++j)
            {
                const unsigned j1 = (j + 1u) % n;
                if (i == j || i1 == j || j1 == i)
                    continue;
                if (i == 0u && j1 == 0u)
                    continue;
                if (SegmentsIntersect2(p2[i], p2[i1], p2[j], p2[j1], intersectionEps))
                {
                    if (details)
                    {
                        details->type_ = SimplePolygonFailureDetails::Type::SelfIntersection;
                        details->edgeA0_ = i;
                        details->edgeA1_ = i1;
                        details->edgeB0_ = j;
                        details->edgeB1_ = j1;
                    }
                    return true;
                }
            }
        }
        return false;
    }

    const float cellSize = ChooseIntersectionGridCellSize(p2, intersectionEps);
    const float invCell = 1.0f / cellSize;

    ea::hash_map<uint64_t, ea::fixed_vector<unsigned, 8, true>> cellToEdges;
    cellToEdges.reserve(n * 2u);

    ea::hash_set<uint64_t> comparedPairs;
    comparedPairs.reserve(n * 8u);

    for (unsigned i = 0; i < n; ++i)
    {
        const unsigned i1 = (i + 1u) % n;
        const Vector2& a = p2[i];
        const Vector2& b = p2[i1];

        // Expand AABB to keep quantization conservative around boundaries.
        const float minXf = ea::min(a.x_, b.x_) - intersectionEps;
        const float minYf = ea::min(a.y_, b.y_) - intersectionEps;
        const float maxXf = ea::max(a.x_, b.x_) + intersectionEps;
        const float maxYf = ea::max(a.y_, b.y_) + intersectionEps;

        const int minX = static_cast<int>(std::floor(minXf * invCell));
        const int minY = static_cast<int>(std::floor(minYf * invCell));
        const int maxX = static_cast<int>(std::floor(maxXf * invCell));
        const int maxY = static_cast<int>(std::floor(maxYf * invCell));

        for (int cx = minX; cx <= maxX; ++cx)
        {
            for (int cy = minY; cy <= maxY; ++cy)
            {
                const uint64_t cellKey = PackCellKey(cx, cy);
                auto& edges = cellToEdges[cellKey];

                for (unsigned idx = 0; idx < edges.size(); ++idx)
                {
                    const unsigned j = edges[idx];
                    if (i == j)
                        continue;

                    const uint64_t pairKey = PackUndirectedEdgeKey(i, j);
                    if (comparedPairs.contains(pairKey))
                        continue;
                    comparedPairs.insert(pairKey);

                    const unsigned j1 = (j + 1u) % n;
                    if (i1 == j || j1 == i)
                        continue;
                    if ((i == 0u && j1 == 0u) || (j == 0u && i1 == 0u))
                        continue;

                    if (SegmentsIntersect2(p2[i], p2[i1], p2[j], p2[j1], intersectionEps))
                    {
                        if (details)
                        {
                            details->type_ = SimplePolygonFailureDetails::Type::SelfIntersection;
                            details->edgeA0_ = i;
                            details->edgeA1_ = i1;
                            details->edgeB0_ = j;
                            details->edgeB1_ = j1;
                        }
                        return true;
                    }
                }

                edges.push_back(i);
            }
        }
    }

    return false;
}

static bool IsSimplePolygonInPlane(const CsgPolygon& poly, float epsilon, const Vector3& axisU, const Vector3& axisV)
{
    const unsigned n = poly.vertices_.size();
    if (n < 3)
        return false;

    const float eps = ea::max(epsilon, 1e-6f);
    const float intersectionEps = ea::max(1e-9f, eps * 1e-5f);
    const float invEps = 1.0f / eps;

    ea::hash_map<IntVector3, unsigned> unique;
    unique.reserve(n * 2u);
    for (unsigned i = 0; i < n; ++i)
    {
        const IntVector3 qp = QuantizePosition(poly.vertices_[i].GetPosition(), invEps);
        const auto it = unique.find(qp);
        if (it != unique.end())
            return false;
        unique.insert({qp, i});
    }

    ea::fixed_vector<Vector2, 16, true> p2;
    p2.resize(n);
    for (unsigned i = 0; i < n; ++i)
    {
        const Vector3 p = poly.vertices_[i].GetPosition();
        p2[i] = {p.DotProduct(axisU), p.DotProduct(axisV)};
    }

    if (HasSelfIntersections2DGrid(p2, intersectionEps, nullptr))
        return false;

    return true;
}

static bool IsSimplePolygonInPlane(const CsgPolygon& poly, float epsilon)
{
    Vector3 axisU, axisV;
    MakeAxesFromNormal(poly.plane_.normal_, axisU, axisV);
    return IsSimplePolygonInPlane(poly, epsilon, axisU, axisV);
}

static bool IsSimplePolygonInPlane(
    const CsgPolygon& poly, float epsilon, const Vector3& axisU, const Vector3& axisV, SimplePolygonFailureDetails& details)
{
    details = {};

    const unsigned n = poly.vertices_.size();
    if (n < 3)
        return false;

    const float eps = ea::max(epsilon, 1e-6f);
    const float intersectionEps = ea::max(1e-9f, eps * 1e-5f);
    const float invEps = 1.0f / eps;

    ea::hash_map<IntVector3, unsigned> unique;
    unique.reserve(n * 2u);
    for (unsigned i = 0; i < n; ++i)
    {
        const IntVector3 qp = QuantizePosition(poly.vertices_[i].GetPosition(), invEps);
        const auto it = unique.find(qp);
        if (it != unique.end())
        {
            details.type_ = SimplePolygonFailureDetails::Type::DuplicateQuantizedVertex;
            details.duplicateKey_ = qp;
            details.duplicateFirstIndex_ = it->second;
            details.duplicateSecondIndex_ = i;
            return false;
        }
        unique.insert({qp, i});
    }

    ea::fixed_vector<Vector2, 16, true> p2;
    p2.resize(n);
    for (unsigned i = 0; i < n; ++i)
    {
        const Vector3 p = poly.vertices_[i].GetPosition();
        p2[i] = {p.DotProduct(axisU), p.DotProduct(axisV)};
    }

    if (HasSelfIntersections2DGrid(p2, intersectionEps, &details))
        return false;

    return true;
}

static bool IsSimplePolygonInPlane(const CsgPolygon& poly, float epsilon, SimplePolygonFailureDetails& details)
{
    Vector3 axisU, axisV;
    MakeAxesFromNormal(poly.plane_.normal_, axisU, axisV);
    return IsSimplePolygonInPlane(poly, epsilon, axisU, axisV, details);
}

static float PolygonAbsAreaInPlane(const CsgPolygon& poly, const Vector3& axisU, const Vector3& axisV)
{
    const unsigned n = poly.vertices_.size();
    if (n < 3)
        return 0.0f;

    double area2 = 0.0;
    for (unsigned i = 0; i < n; ++i)
    {
        const Vector3 p0 = poly.vertices_[i].GetPosition();
        const Vector3 p1 = poly.vertices_[(i + 1u) % n].GetPosition();
        const Vector2 a{p0.DotProduct(axisU), p0.DotProduct(axisV)};
        const Vector2 b{p1.DotProduct(axisU), p1.DotProduct(axisV)};
        area2 += static_cast<double>(a.x_) * static_cast<double>(b.y_) - static_cast<double>(b.x_) * static_cast<double>(a.y_);
    }
    return Abs(static_cast<float>(0.5 * area2));
}

static float PolygonAbsAreaInPlane(const CsgPolygon& poly)
{
    Vector3 axisU, axisV;
    MakeAxesFromNormal(poly.plane_.normal_, axisU, axisV);
    return PolygonAbsAreaInPlane(poly, axisU, axisV);
}

static bool AreCoplanarOriented(const Plane& a, const Plane& b, float normalEps, float epsilon)
{
    const float lenSqA = a.normal_.LengthSquared();
    const float lenSqB = b.normal_.LengthSquared();
    if (lenSqA <= 1e-20f || lenSqB <= 1e-20f)
        return false;

    const float dot = a.normal_.DotProduct(b.normal_);
    if (dot <= 0.0f)
        return false;

    const float cosThreshold = 1.0f - normalEps;
    const float lhs = dot * dot;
    const float rhs = cosThreshold * cosThreshold * lenSqA * lenSqB;
    if (lhs < rhs)
        return false;
    return Abs(a.d_ - b.d_) <= epsilon;
}

static unsigned GetVertexId(ea::hash_map<IntVector3, unsigned>& vertexId, const Vector3& p, float invEpsilon)
{
    const IntVector3 qp = QuantizePosition(p, invEpsilon);
    return GetVertexId(vertexId, qp);
}

static bool RemoveOneRedundantVertex(CsgPolygon& poly, float epsilon)
{
    const float eps = ea::max(epsilon, 1e-6f);
    const float epsSq = eps * eps;

    const unsigned n = poly.vertices_.size();
    if (n < 3)
        return false;

    for (unsigned i = 0; i < n; ++i)
    {
        const unsigned prev = (i + n - 1) % n;
        const unsigned next = (i + 1) % n;

        const Vector3 p0 = poly.vertices_[prev].GetPosition();
        const Vector3 p1 = poly.vertices_[i].GetPosition();
        const Vector3 p2 = poly.vertices_[next].GetPosition();

        if ((p1 - p0).LengthSquared() <= epsSq)
        {
            poly.vertices_.erase(poly.vertices_.begin() + i);
            return true;
        }

        const Vector3 line = p2 - p0;
        const float lineLenSq = line.LengthSquared();
        if (lineLenSq <= epsSq)
        {
            poly.vertices_.erase(poly.vertices_.begin() + i);
            return true;
        }

        const float lineLen = Sqrt(lineLenSq);
        const float dist = (p1 - p0).CrossProduct(line).Length() / lineLen;
        if (dist > eps)
            continue;

        const Vector3 d1 = (p1 - p0).NormalizedOrDefault(Vector3::ZERO);
        const Vector3 d2 = (p2 - p1).NormalizedOrDefault(Vector3::ZERO);
        if (d1.DotProduct(d2) <= 0.0f)
            continue;

        poly.vertices_.erase(poly.vertices_.begin() + i);
        return true;
    }

    return false;
}

static void CleanupPolygonVertices(CsgPolygon& poly, float epsilon)
{
    // Conservative local cleanup: remove near-duplicates and non-reflex colinear points.
    while (poly.vertices_.size() > 3 && RemoveOneRedundantVertex(poly, epsilon))
    {
    }
}

static bool PopOutgoingEdge(ea::hash_map<unsigned, ea::vector<unsigned>>& outgoing, unsigned from, unsigned to)
{
    auto it = outgoing.find(from);
    if (it == outgoing.end())
        return false;
    auto& v = it->second;
    for (unsigned i = 0; i < v.size(); ++i)
    {
        if (v[i] == to)
        {
            v[i] = v.back();
            v.pop_back();
            if (v.empty())
                outgoing.erase(from);
            return true;
        }
    }
    return false;
}

static unsigned ChooseNextVertex(const ea::vector<unsigned>& candidates, unsigned prev, unsigned curr, const ea::vector<Vector2>& pos2)
{
    if (candidates.empty())
        return M_MAX_UNSIGNED;
    if (candidates.size() == 1)
        return candidates[0];

    unsigned best = M_MAX_UNSIGNED;
    float bestScore = -1e30f;
    const Vector2 inDir = pos2[curr] - pos2[prev];
    const float inLen = ea::max(inDir.Length(), 1e-6f);

    for (unsigned cand : candidates)
    {
        if (cand == prev)
            continue;
        const Vector2 outDir = pos2[cand] - pos2[curr];
        const float outLen = ea::max(outDir.Length(), 1e-6f);
        const float cross = inDir.CrossProduct(outDir) / (inLen * outLen);
        const float dot = inDir.DotProduct(outDir) / (inLen * outLen);
        const float score = cross * 2.0f + dot;
        if (score > bestScore)
        {
            bestScore = score;
            best = cand;
        }
    }

    if (best == M_MAX_UNSIGNED)
        best = candidates[0];
    return best;
}

static bool TryExtractSingleLoopFromEdges(
    const ea::vector<Vector2>& pos2, ea::hash_map<unsigned, ea::vector<unsigned>>& outgoing, ea::vector<unsigned>& outLoop)
{
    outLoop.clear();
    if (outgoing.empty())
        return false;

    unsigned start = outgoing.begin()->first;
    if (outgoing.begin()->second.empty())
        return false;
    unsigned next = outgoing.begin()->second.back();
    PopOutgoingEdge(outgoing, start, next);

    outLoop.push_back(start);
    outLoop.push_back(next);

    unsigned prev = start;
    unsigned curr = next;

    const unsigned maxSteps = pos2.size() * 4u + 16u;
    for (unsigned step = 0; step < maxSteps; ++step)
    {
        if (curr == start)
            break;

        const auto it = outgoing.find(curr);
        if (it == outgoing.end())
            return false;

        const unsigned chosen = ChooseNextVertex(it->second, prev, curr, pos2);
        if (chosen == M_MAX_UNSIGNED)
            return false;

        if (!PopOutgoingEdge(outgoing, curr, chosen))
            return false;

        outLoop.push_back(chosen);
        prev = curr;
        curr = chosen;
    }

    if (outLoop.size() < 4)
        return false;
    if (outLoop.back() != start)
        return false;
    outLoop.pop_back();
    return outLoop.size() >= 3;
}

static void ProjectTo2D(const CsgPolygon& poly, const Vector3& axisU, const Vector3& axisV, ea::vector<Vector2>& out)
{
    const unsigned n = poly.vertices_.size();
    out.clear();
    out.reserve(n);
    for (unsigned i = 0; i < n; ++i)
    {
        const Vector3 p = poly.vertices_[i].GetPosition();
        out.push_back({p.DotProduct(axisU), p.DotProduct(axisV)});
    }
}

static float SignedArea2D(const ea::vector<Vector2>& poly)
{
    const unsigned n = poly.size();
    if (n < 3)
        return 0.0f;
    double area2 = 0.0;
    for (unsigned i = 0; i < n; ++i)
    {
        const Vector2& a = poly[i];
        const Vector2& b = poly[(i + 1u) % n];
        area2 += static_cast<double>(a.x_) * static_cast<double>(b.y_) - static_cast<double>(b.x_) * static_cast<double>(a.y_);
    }
    return static_cast<float>(0.5 * area2);
}

static void RemoveConsecutiveDuplicateVertices(CsgPolygon& poly, float epsilon)
{
    const float eps = ea::max(epsilon, 1e-6f);
    const float epsSq = eps * eps;
    while (poly.vertices_.size() >= 2)
    {
        bool changed = false;
        for (unsigned i = 0; i < poly.vertices_.size(); ++i)
        {
            const unsigned i1 = (i + 1u) % poly.vertices_.size();
            const Vector3 p0 = poly.vertices_[i].GetPosition();
            const Vector3 p1 = poly.vertices_[i1].GetPosition();
            if ((p0 - p1).LengthSquared() <= epsSq)
            {
                poly.vertices_.erase(poly.vertices_.begin() + i1);
                changed = true;
                break;
            }
        }
        if (!changed)
            break;
        if (poly.vertices_.size() < 3)
            break;
    }
}

static void SplitPolygonEdgesAtVertices(CsgPolygon& poly, const ea::vector<ModelVertex>& otherVertices, float epsilon)
{
    if (poly.vertices_.size() < 3 || otherVertices.empty())
        return;

    const float eps = ea::max(epsilon, 1e-6f);
    const float epsSq = eps * eps;
    const float invEps = 1.0f / eps;

    Vector3 axisU, axisV;
    MakeAxesFromNormal(poly.plane_.normal_, axisU, axisV);

    ea::vector<Vector2> poly2;
    ProjectTo2D(poly, axisU, axisV, poly2);

    ea::vector<Vector2> other2;
    other2.resize(otherVertices.size());
    for (unsigned i = 0; i < otherVertices.size(); ++i)
    {
        const Vector3 p = otherVertices[i].GetPosition();
        other2[i] = {p.DotProduct(axisU), p.DotProduct(axisV)};
    }

    struct SplitPoint
    {
        float t_{};
        Vector3 pos_;
    };

    ea::vector<ModelVertex> out;
    out.reserve(poly.vertices_.size() + otherVertices.size());

    const unsigned n = poly.vertices_.size();
    for (unsigned i = 0; i < n; ++i)
    {
        const unsigned i1 = (i + 1u) % n;
        const Vector2 a0 = poly2[i];
        const Vector2 a1 = poly2[i1];
        const Vector2 ab = a1 - a0;
        const float abLenSq = ab.LengthSquared();

        out.push_back(poly.vertices_[i]);

        if (abLenSq <= 1e-20f)
            continue;

        const Vector3 p0 = poly.vertices_[i].GetPosition();
        const Vector3 p1 = poly.vertices_[i1].GetPosition();

        ea::vector<SplitPoint> splits;
        splits.reserve(8);
        ea::hash_set<IntVector3> used;
        used.reserve(8);

        for (unsigned j = 0; j < otherVertices.size(); ++j)
        {
            const Vector3 p = otherVertices[j].GetPosition();
            if ((p - p0).LengthSquared() <= epsSq || (p - p1).LengthSquared() <= epsSq)
                continue;

            const Vector2 q = other2[j];
            if (!OnSegment2(a0, a1, q, eps))
                continue;

            const float t = (q - a0).DotProduct(ab) / abLenSq;
            if (!(t > 0.0f && t < 1.0f))
                continue;

            const IntVector3 key = QuantizePosition(p, invEps);
            if (used.contains(key))
                continue;
            used.insert(key);

            splits.push_back({t, p});
        }

        if (splits.empty())
            continue;

        ea::sort(splits.begin(), splits.end(), [](const SplitPoint& lhs, const SplitPoint& rhs) { return lhs.t_ < rhs.t_; });

        for (const SplitPoint& sp : splits)
        {
            ModelVertex newV = LerpModelVertex(poly.vertices_[i], poly.vertices_[i1], sp.t_);
            newV.SetPosition(sp.pos_);
            out.push_back(newV);
        }
    }

    poly.vertices_ = ea::move(out);
    RemoveConsecutiveDuplicateVertices(poly, epsilon);
}

static void SplitPolygonsMutuallyAtOverlaps(CsgPolygon& a, CsgPolygon& b, float epsilon)
{
    SplitPolygonEdgesAtVertices(a, b.vertices_, epsilon);
    SplitPolygonEdgesAtVertices(b, a.vertices_, epsilon);
}

struct BoundaryCancellationBuilder
{
    float invEps_{};
    ea::hash_map<IntVector3, unsigned> vertexId_;
    ea::vector<ModelVertex> repVertex_;
    ea::vector<Vector3> repPos_;

    void Init(float invEps, unsigned reserveVertices)
    {
        invEps_ = invEps;
        vertexId_.clear();
        vertexId_.reserve(reserveVertices * 2u);
        repVertex_.clear();
        repPos_.clear();
        repVertex_.reserve(reserveVertices);
        repPos_.reserve(reserveVertices);
    }

    unsigned GetId(const ModelVertex& v)
    {
        const Vector3 p = v.GetPosition();
        const IntVector3 qp = QuantizePosition(p, invEps_);
        const auto it = vertexId_.find(qp);
        if (it != vertexId_.end())
            return it->second;
        const unsigned id = vertexId_.size();
        vertexId_.insert({qp, id});
        repVertex_.push_back(v);
        repPos_.push_back(p);
        return id;
    }

    void AddDirectedEdges(const CsgPolygon& poly, ea::hash_map<uint64_t, unsigned>& directedCount)
    {
        const unsigned n = poly.vertices_.size();
        for (unsigned i = 0; i < n; ++i)
        {
            const unsigned i1 = (i + 1u) % n;
            const unsigned id0 = GetId(poly.vertices_[i]);
            const unsigned id1 = GetId(poly.vertices_[i1]);
            if (id0 == id1)
                continue;
            directedCount[PackDirectedEdgeKey(id0, id1)] += 1;
        }
    }
};

static bool TryMergeByBoundaryCancellation(
    const CsgPolygon& a, const CsgPolygon& b, const Vector3& axisU, const Vector3& axisV, float normalEps, float epsilon, float epsSq, CsgPolygon& out)
{
    if (a.shared_ != b.shared_)
        return false;

    const Vector3 na = a.plane_.normal_.NormalizedOrDefault(Vector3::ZERO);
    const Vector3 nb = b.plane_.normal_.NormalizedOrDefault(Vector3::ZERO);
    const float planeDot = na == Vector3::ZERO || nb == Vector3::ZERO ? 0.0f : na.DotProduct(nb);
    const float planeDDiff = Abs(a.plane_.d_ - b.plane_.d_);
    if (planeDot < 1.0f - normalEps || planeDDiff > epsilon)
        return false;

    if (!IsSimplePolygonInPlane(a, epsilon, axisU, axisV) || !IsSimplePolygonInPlane(b, epsilon, axisU, axisV))
        return false;

    // Fallback path: make partial colinear overlaps explicit shared edges.
    CsgPolygon aSplit = a;
    CsgPolygon bSplit = b;
    SplitPolygonsMutuallyAtOverlaps(aSplit, bSplit, epsilon);

    const float eps = ea::max(epsilon, 1e-6f);
    const float invEps = 1.0f / eps;

    BoundaryCancellationBuilder builder;
    builder.Init(invEps, a.vertices_.size() + b.vertices_.size());

    ea::hash_map<uint64_t, unsigned> directedCount;
    directedCount.reserve((a.vertices_.size() + b.vertices_.size()) * 2u);
    builder.AddDirectedEdges(aSplit, directedCount);
    builder.AddDirectedEdges(bSplit, directedCount);

    // Cancel opposite edges.
    ea::hash_set<uint64_t> visitedUndirected;
    visitedUndirected.reserve(directedCount.size() * 2u);
    ea::vector<CanceledEdge> boundaryEdges;
    boundaryEdges.reserve(directedCount.size());

    for (const auto& it : directedCount)
    {
        const uint64_t key = it.first;
        const unsigned from = static_cast<unsigned>(key >> 32u);
        const unsigned to = static_cast<unsigned>(key & 0xffffffffu);
        const uint64_t und = PackUndirectedEdgeKey(from, to);
        if (visitedUndirected.contains(und))
            continue;
        visitedUndirected.insert(und);

        const uint64_t rev = PackDirectedEdgeKey(to, from);
        const unsigned cF = it.second;
        const auto itRev = directedCount.find(rev);
        const unsigned cR = itRev != directedCount.end() ? itRev->second : 0u;
        if (cF == cR)
            continue;

        const bool keepForward = cF > cR;
        const unsigned keepCount = keepForward ? (cF - cR) : (cR - cF);
        const unsigned kFrom = keepForward ? from : to;
        const unsigned kTo = keepForward ? to : from;
        for (unsigned k = 0; k < keepCount; ++k)
            boundaryEdges.push_back({kFrom, kTo});
    }

    if (boundaryEdges.empty())
        return false;

    // Build 2D positions for angle-based traversal.
    ea::vector<Vector2> pos2;
    pos2.resize(builder.repPos_.size());
    for (unsigned i = 0; i < builder.repPos_.size(); ++i)
    {
        const Vector3 p = builder.repPos_[i];
        pos2[i] = {p.DotProduct(axisU), p.DotProduct(axisV)};
    }

    ea::hash_map<unsigned, ea::vector<unsigned>> outgoing;
    outgoing.reserve(boundaryEdges.size() * 2u);
    for (const auto& e : boundaryEdges)
        outgoing[e.from_].push_back(e.to_);

    ea::vector<unsigned> loopIds;
    if (!TryExtractSingleLoopFromEdges(pos2, outgoing, loopIds))
        return false;
    if (!outgoing.empty())
        return false;

    out.shared_ = a.shared_;
    out.plane_ = a.plane_;
    out.vertices_.resize(loopIds.size());
    for (unsigned i = 0; i < loopIds.size(); ++i)
        out.vertices_[i] = builder.repVertex_[loopIds[i]];

    CleanupPolygonVertices(out, epsilon);
    if (IsDegeneratePolygon(out))
        return false;

    const float areaA = PolygonAbsAreaInPlane(a, axisU, axisV);
    const float areaB = PolygonAbsAreaInPlane(b, axisU, axisV);
    const float expected = areaA + areaB;
    const float tol = expected * 1e-3f + epsilon * epsilon * 10.0f;

    SimplePolygonFailureDetails failure;
    if (!IsSimplePolygonInPlane(out, epsilon, axisU, axisV, failure))
        return false;

    const float areaOut = PolygonAbsAreaInPlane(out, axisU, axisV);
    if (!(Abs(areaOut - expected) <= tol))
        return false;

    // Normalize winding to keep signed area consistent with plane axes.
    ea::vector<Vector2> out2;
    ProjectTo2D(out, axisU, axisV, out2);
    if (SignedArea2D(out2) < 0.0f)
        ea::reverse(out.vertices_.begin(), out.vertices_.end());

    return true;
}

static bool ExtractLoopByIndices(const ea::vector<ModelVertex>& vertices, unsigned begin, unsigned endExclusive, CsgPolygon& out)
{
    out.vertices_.resize(0);
    if (vertices.empty())
        return false;

    const unsigned n = vertices.size();
    begin %= n;
    endExclusive %= n;

    unsigned i = begin;
    while (i != endExclusive)
    {
        out.vertices_.push_back(vertices[i]);
        i = (i + 1u) % n;
        if (out.vertices_.size() > n + 1u)
            return false;
    }
    return out.vertices_.size() >= 3;
}

static bool ValidateRepairedLoop(
    CsgPolygon& candidate, float epsilon, float expectedArea, float expectedTol, const Vector3& axisU, const Vector3& axisV)
{
    CleanupPolygonVertices(candidate, epsilon);
    if (IsDegeneratePolygon(candidate))
        return false;
    if (!IsSimplePolygonInPlane(candidate, epsilon, axisU, axisV))
        return false;
    const float area = PolygonAbsAreaInPlane(candidate, axisU, axisV);
    return Abs(area - expectedArea) <= expectedTol;
}

static bool TryRepairDuplicateVertexLoop(
    CsgPolygon& poly, const SimplePolygonFailureDetails& details, const Vector3& axisU, const Vector3& axisV, float epsilon, float expectedArea, float expectedTol)
{
    if (details.type_ != SimplePolygonFailureDetails::Type::DuplicateQuantizedVertex)
        return false;

    const unsigned n = poly.vertices_.size();
    if (n < 4)
        return false;

    unsigned i0 = ea::min(details.duplicateFirstIndex_, details.duplicateSecondIndex_);
    unsigned i1 = ea::max(details.duplicateFirstIndex_, details.duplicateSecondIndex_);
    if (i0 >= n || i1 >= n || i0 == i1)
        return false;

    // Only attempt repair when the duplicate positions are extremely close.
    const float repairEps = ea::max(1e-7f, epsilon * 1e-3f);
    const float repairEpsSq = repairEps * repairEps;
    const Vector3 p0 = poly.vertices_[i0].GetPosition();
    const Vector3 p1 = poly.vertices_[i1].GetPosition();
    if ((p0 - p1).LengthSquared() > repairEpsSq)
        return false;

    CsgPolygon loopA;
    loopA.shared_ = poly.shared_;
    loopA.plane_ = poly.plane_;
    if (!ExtractLoopByIndices(poly.vertices_, i0, (i1 + 1u) % n, loopA))
        loopA.vertices_.resize(0);

    CsgPolygon loopB;
    loopB.shared_ = poly.shared_;
    loopB.plane_ = poly.plane_;
    if (!ExtractLoopByIndices(poly.vertices_, i1, (i0 + 1u) % n, loopB))
        loopB.vertices_.resize(0);

    const bool aOk = ValidateRepairedLoop(loopA, epsilon, expectedArea, expectedTol, axisU, axisV);
    const bool bOk = ValidateRepairedLoop(loopB, epsilon, expectedArea, expectedTol, axisU, axisV);
    if (!aOk && !bOk)
        return false;

    if (aOk && bOk)
    {
        const float areaA = PolygonAbsAreaInPlane(loopA);
        const float areaB = PolygonAbsAreaInPlane(loopB);
        poly = areaA >= areaB ? ea::move(loopA) : ea::move(loopB);
    }
    else
        poly = aOk ? ea::move(loopA) : ea::move(loopB);

    return true;
}

static bool TryMergeAtSharedEdge(const CsgPolygon& a, const CsgPolygon& b, unsigned aEdgeStart, unsigned bEdgeStart, CsgPolygon& out,
    float normalEps, float epsilon, float epsSq)
{
    if (a.shared_ != b.shared_)
    {
        CsgSimplifyDiagLog("CSG simplify: merge reject (shared mismatch)");
        return false;
    }

    const Vector3 na = a.plane_.normal_.NormalizedOrDefault(Vector3::ZERO);
    const Vector3 nb = b.plane_.normal_.NormalizedOrDefault(Vector3::ZERO);
    const float planeDot = na == Vector3::ZERO || nb == Vector3::ZERO ? 0.0f : na.DotProduct(nb);
    const float planeDDiff = Abs(a.plane_.d_ - b.plane_.d_);
    if (planeDot < 1.0f - normalEps || planeDDiff > epsilon)
    {
        CsgSimplifyDiagLog("CSG simplify: merge reject (plane mismatch)");
        return false;
    }

    Vector3 axisU, axisV;
    MakeAxesFromNormal(a.plane_.normal_, axisU, axisV);

    const unsigned aCount = a.vertices_.size();
    const unsigned bCount = b.vertices_.size();
    if (aCount < 3 || bCount < 3)
    {
        CsgSimplifyDiagLog("CSG simplify: merge reject (degenerate input)");
        return false;
    }

    const unsigned a0 = aEdgeStart;
    const unsigned a1 = (aEdgeStart + 1u) % aCount;
    const unsigned b0 = bEdgeStart;
    const unsigned b1 = (bEdgeStart + 1u) % bCount;

    const Vector3 av0 = a.vertices_[a0].GetPosition();
    const Vector3 av1 = a.vertices_[a1].GetPosition();
    const Vector3 bv0 = b.vertices_[b0].GetPosition();
    const Vector3 bv1 = b.vertices_[b1].GetPosition();

    const float d01Sq = (av0 - bv1).LengthSquared();
    const float d10Sq = (av1 - bv0).LengthSquared();
    if (d01Sq > epsSq || d10Sq > epsSq)
    {
        CsgSimplifyDiagLog("CSG simplify: merge reject (edge mismatch)");
        return false;
    }

    out.shared_ = a.shared_;
    out.plane_ = a.plane_;
    out.vertices_.resize(0);
    out.vertices_.reserve(aCount + bCount - 2u);

    for (unsigned i = 0; i <= aEdgeStart; ++i)
        out.vertices_.push_back(a.vertices_[i]);

    for (unsigned k = 0; k < bCount - 2u; ++k)
    {
        const unsigned idx = (bEdgeStart + 2u + k) % bCount;
        out.vertices_.push_back(b.vertices_[idx]);
    }

    for (unsigned i = aEdgeStart + 1u; i < aCount; ++i)
        out.vertices_.push_back(a.vertices_[i]);

    CleanupPolygonVertices(out, epsilon);
    if (IsDegeneratePolygon(out))
    {
        CsgSimplifyDiagLog("CSG simplify: merge reject (degenerate output)");
        return false;
    }

    const float areaA = PolygonAbsAreaInPlane(a, axisU, axisV);
    const float areaB = PolygonAbsAreaInPlane(b, axisU, axisV);
    const float expected = areaA + areaB;
    const float tol = expected * 1e-3f + epsilon * epsilon * 10.0f;

    SimplePolygonFailureDetails failure;
    if (!IsSimplePolygonInPlane(out, epsilon, axisU, axisV, failure))
    {
        if (!TryRepairDuplicateVertexLoop(out, failure, axisU, axisV, epsilon, expected, tol))
        {
            CsgPolygon canceled;
            if (!TryMergeByBoundaryCancellation(a, b, axisU, axisV, normalEps, epsilon, epsSq, canceled))
            {
                CsgSimplifyDiagLog("CSG simplify: merge reject (non-simple polygon)");
                return false;
            }
            out = ea::move(canceled);
        }
    }

    const float areaOut = PolygonAbsAreaInPlane(out, axisU, axisV);
    if (!(Abs(areaOut - expected) <= tol))
    {
        CsgSimplifyDiagLog("CSG simplify: merge reject (area mismatch)");
        return false;
    }

    return true;
}

static bool TryMergeAtSharedEdge(const CsgPolygon& a, const CsgPolygon& b, unsigned aEdgeStart, unsigned bEdgeStart, CsgPolygon& out,
    const Vector3& axisU, const Vector3& axisV, float normalEps, float epsilon, float epsSq)
{
    if (a.shared_ != b.shared_)
        return false;

    const Vector3 na = a.plane_.normal_.NormalizedOrDefault(Vector3::ZERO);
    const Vector3 nb = b.plane_.normal_.NormalizedOrDefault(Vector3::ZERO);
    const float planeDot = na == Vector3::ZERO || nb == Vector3::ZERO ? 0.0f : na.DotProduct(nb);
    const float planeDDiff = Abs(a.plane_.d_ - b.plane_.d_);
    if (planeDot < 1.0f - normalEps || planeDDiff > epsilon)
        return false;

    const unsigned aCount = a.vertices_.size();
    const unsigned bCount = b.vertices_.size();
    if (aCount < 3 || bCount < 3)
        return false;

    const unsigned a0 = aEdgeStart;
    const unsigned a1 = (aEdgeStart + 1u) % aCount;
    const unsigned b0 = bEdgeStart;
    const unsigned b1 = (bEdgeStart + 1u) % bCount;

    const Vector3 av0 = a.vertices_[a0].GetPosition();
    const Vector3 av1 = a.vertices_[a1].GetPosition();
    const Vector3 bv0 = b.vertices_[b0].GetPosition();
    const Vector3 bv1 = b.vertices_[b1].GetPosition();

    const float d01Sq = (av0 - bv1).LengthSquared();
    const float d10Sq = (av1 - bv0).LengthSquared();
    if (d01Sq > epsSq || d10Sq > epsSq)
        return false;

    out.shared_ = a.shared_;
    out.plane_ = a.plane_;
    out.vertices_.resize(0);
    out.vertices_.reserve(aCount + bCount - 2u);

    for (unsigned i = 0; i <= aEdgeStart; ++i)
        out.vertices_.push_back(a.vertices_[i]);

    for (unsigned k = 0; k < bCount - 2u; ++k)
    {
        const unsigned idx = (bEdgeStart + 2u + k) % bCount;
        out.vertices_.push_back(b.vertices_[idx]);
    }

    for (unsigned i = aEdgeStart + 1u; i < aCount; ++i)
        out.vertices_.push_back(a.vertices_[i]);

    CleanupPolygonVertices(out, epsilon);
    if (IsDegeneratePolygon(out))
        return false;

    const float areaA = PolygonAbsAreaInPlane(a, axisU, axisV);
    const float areaB = PolygonAbsAreaInPlane(b, axisU, axisV);
    const float expected = areaA + areaB;
    const float tol = expected * 1e-3f + epsilon * epsilon * 10.0f;

    SimplePolygonFailureDetails failure;
    if (!IsSimplePolygonInPlane(out, epsilon, axisU, axisV, failure))
    {
        if (!TryRepairDuplicateVertexLoop(out, failure, axisU, axisV, epsilon, expected, tol))
        {
            CsgPolygon canceled;
            if (!TryMergeByBoundaryCancellation(a, b, axisU, axisV, normalEps, epsilon, epsSq, canceled))
                return false;
            out = ea::move(canceled);
        }
    }

    const float areaOut = PolygonAbsAreaInPlane(out, axisU, axisV);
    if (!(Abs(areaOut - expected) <= tol))
        return false;

    return true;
}

static bool MergeByFullEdgesToFixpoint(ea::vector<CsgPolygon>& group, float epsilon, float normalEps, float epsSq)
{
    bool mergedEver = false;

    ea::erase_if(group, IsDegeneratePolygon);
    if (group.size() < 2)
        return false;

    // Cache plane basis once per coplanar group.
    Vector3 axisU, axisV;
    MakeAxesFromNormal(group.front().plane_.normal_, axisU, axisV);

    const float invEpsilon = 1.0f / epsilon;

    struct DirectedEdgeInfo
    {
        unsigned edgeStart_{};
        unsigned fromId_{};
        unsigned toId_{};
        uint64_t key_{};
    };

    auto removeEdgeRef = [](ea::hash_map<uint64_t, ea::fixed_vector<EdgeRef, 2, true>>& edgesByDirected, uint64_t key,
        unsigned polyIndex, unsigned edgeStart)
    {
        const auto it = edgesByDirected.find(key);
        if (it == edgesByDirected.end())
            return;
        auto& refs = it->second;
        for (unsigned i = 0; i < refs.size(); ++i)
        {
            if (refs[i].polyIndex_ == polyIndex && refs[i].edgeStart_ == edgeStart)
            {
                refs.erase(refs.begin() + i);
                break;
            }
        }
        if (refs.empty())
            edgesByDirected.erase(it);
    };

    ea::hash_map<IntVector3, unsigned> vertexId;
    vertexId.reserve(group.size() * 16u);

    ea::vector<ea::vector<DirectedEdgeInfo>> edgesByPoly;
    edgesByPoly.resize(group.size());

    ea::hash_map<uint64_t, ea::fixed_vector<EdgeRef, 2, true>> edgesByDirected;
    edgesByDirected.reserve(group.size() * 8u);

    auto rebuildPolyEdges = [&](unsigned pi)
    {
        edgesByPoly[pi].clear();
        const auto& poly = group[pi];
        const unsigned n = poly.vertices_.size();
        if (n < 3)
            return;

        edgesByPoly[pi].reserve(n);
        for (unsigned ei = 0; ei < n; ++ei)
        {
            const Vector3 p0 = poly.vertices_[ei].GetPosition();
            const Vector3 p1 = poly.vertices_[(ei + 1u) % n].GetPosition();
            const unsigned id0 = GetVertexId(vertexId, p0, invEpsilon);
            const unsigned id1 = GetVertexId(vertexId, p1, invEpsilon);
            if (id0 == id1)
                continue;
            const uint64_t key = PackDirectedEdgeKey(id0, id1);
            edgesByPoly[pi].push_back(DirectedEdgeInfo{ei, id0, id1, key});
        }
    };

    auto addPolyToMap = [&](unsigned pi)
    {
        for (const auto& e : edgesByPoly[pi])
        {
            auto& refs = edgesByDirected[e.key_];
            refs.push_back(EdgeRef{pi, e.edgeStart_});
            ea::sort(refs.begin(), refs.end(), [](const EdgeRef& lhs, const EdgeRef& rhs)
            {
                if (lhs.polyIndex_ != rhs.polyIndex_)
                    return lhs.polyIndex_ < rhs.polyIndex_;
                return lhs.edgeStart_ < rhs.edgeStart_;
            });
        }
    };

    auto removePolyFromMap = [&](unsigned pi)
    {
        for (const auto& e : edgesByPoly[pi])
            removeEdgeRef(edgesByDirected, e.key_, pi, e.edgeStart_);
    };

    for (unsigned pi = 0; pi < group.size(); ++pi)
    {
        rebuildPolyEdges(pi);
        addPolyToMap(pi);
    }

    ea::vector<unsigned char> dirty(group.size(), 1);

    for (;;)
    {
        bool mergedThisRound = false;

        for (unsigned pi = 0; pi < group.size(); ++pi)
        {
            if (!dirty[pi])
                continue;
            dirty[pi] = 0;

            if (IsDegeneratePolygon(group[pi]))
                continue;

            // Mirror the original deterministic behavior: scan polys/edges in order and perform the first valid merge.
            for (const auto& e : edgesByPoly[pi])
            {
                const uint64_t revKey = PackDirectedEdgeKey(e.toId_, e.fromId_);
                const auto it = edgesByDirected.find(revKey);
                if (it == edgesByDirected.end())
                    continue;

                // Copy refs because the merge path mutates edgesByDirected and may invalidate iterators/references.
                const auto otherRefs = it->second;
                for (EdgeRef other : otherRefs)
                {
                    const unsigned otherIndex = other.polyIndex_;
                    if (otherIndex == pi)
                        continue;
                    if (otherIndex >= group.size())
                        continue;
                    if (IsDegeneratePolygon(group[otherIndex]))
                        continue;

                    CsgPolygon merged;
                    if (!TryMergeAtSharedEdge(group[pi], group[otherIndex], e.edgeStart_, other.edgeStart_, merged,
                            axisU, axisV, normalEps, epsilon, epsSq))
                    {
                        continue;
                    }

                    // Update maps incrementally.
                    removePolyFromMap(pi);
                    removePolyFromMap(otherIndex);

                    group[pi] = ea::move(merged);
                    group[otherIndex].vertices_.resize(0);

                    rebuildPolyEdges(pi);
                    edgesByPoly[otherIndex].clear();

                    addPolyToMap(pi);

                    mergedEver = true;
                    mergedThisRound = true;

                    // Conservative correctness: after merge, any polygon may become eligible (mirrors rebuild+rescan).
                    ea::fill(dirty.begin(), dirty.end(), 1);
                    break;
                }

                if (mergedThisRound)
                    break;
            }

            if (mergedThisRound)
                break;
        }

        if (!mergedThisRound)
            break;
    }

    ea::erase_if(group, IsDegeneratePolygon);
    return mergedEver;
}

static unsigned ResolveTJunctions(ea::vector<CsgPolygon>& polygons, float epsilon, unsigned* outPasses)
{
    URHO3D_PROFILE_FUNCTION();

    unsigned insertedTotal = 0;
    if (outPasses)
        *outPasses = 0;

    BoundingBox bbox;
    for (const auto& poly : polygons)
        for (const auto& v : poly.vertices_)
            bbox.Merge(v.GetPosition());

    if (!bbox.Defined())
        return insertedTotal;

    const Vector3 size = bbox.Size();
    const float maxDim = ea::max(size.x_, ea::max(size.y_, size.z_));
    float cellSize = maxDim > M_EPSILON ? maxDim / 20.0f : 1.0f;
    cellSize = ea::max(cellSize, epsilon * 10.0f);

    VertexGrid grid;
    grid.Init(cellSize);
    for (const auto& poly : polygons)
        for (const auto& v : poly.vertices_)
            grid.Add(v.GetPosition());

    bool changed = true;
    const float epsilonSq = epsilon * epsilon;
    unsigned pass = 0;

    ea::fixed_vector<Vector3, 32, true> candidates;

    const float invEps = 1.0f / ea::max(epsilon, 1e-6f);

    auto runPass = [&](bool onlyOpenEdges) -> bool
    {
        ea::hash_map<IntVector3, unsigned> vertexId;
        vertexId.reserve(polygons.size() * 16u);
        ea::hash_map<uint64_t, unsigned> undirectedCount;
        if (onlyOpenEdges)
        {
            undirectedCount.reserve(polygons.size() * 32u);
            for (const auto& poly : polygons)
            {
                const unsigned n = poly.vertices_.size();
                if (n < 3)
                    continue;
                for (unsigned i = 0; i < n; ++i)
                {
                    const unsigned i1 = (i + 1u) % n;
                    const Vector3 p0 = poly.vertices_[i].GetPosition();
                    const Vector3 p1 = poly.vertices_[i1].GetPosition();
                    const unsigned id0 = GetVertexId(vertexId, p0, invEps);
                    const unsigned id1 = GetVertexId(vertexId, p1, invEps);
                    if (id0 == id1)
                        continue;
                    undirectedCount[PackUndirectedEdgeKey(id0, id1)] += 1u;
                }
            }
        }

        bool changedLocal = false;
        ++pass;

        for (auto& poly : polygons)
        {
            if (poly.vertices_.empty())
                continue;

            for (unsigned i = 0; i < poly.vertices_.size(); ++i)
            {
                const unsigned i1 = (i + 1u) % poly.vertices_.size();
                const Vector3 p0 = poly.vertices_[i].GetPosition();
                const Vector3 p1 = poly.vertices_[i1].GetPosition();

                if (onlyOpenEdges)
                {
                    const unsigned id0 = GetVertexId(vertexId, p0, invEps);
                    const unsigned id1 = GetVertexId(vertexId, p1, invEps);
                    if (id0 == id1)
                        continue;
                    const uint64_t und = PackUndirectedEdgeKey(id0, id1);
                    const auto it = undirectedCount.find(und);
                    if (it != undirectedCount.end() && it->second >= 2u)
                        continue;
                }

                Vector3 edgeDir = p1 - p0;
                const float edgeLen = edgeDir.Length();
                if (edgeLen < epsilon)
                    continue;

                edgeDir /= edgeLen;
                grid.Query(p0, p1, candidates);

                float bestT = edgeLen;
                Vector3 bestPos;
                bool foundSplit = false;

                for (const auto& pos : candidates)
                {
                    const Vector3 vec = pos - p0;
                    const float t = vec.DotProduct(edgeDir);

                    if (t > epsilon && t < edgeLen - epsilon)
                    {
                        const Vector3 proj = p0 + edgeDir * t;
                        if ((pos - proj).LengthSquared() < epsilonSq)
                        {
                            if (t < bestT)
                            {
                                bestT = t;
                                bestPos = pos;
                                foundSplit = true;
                            }
                        }
                    }
                }

                if (foundSplit)
                {
                    ModelVertex newV = LerpModelVertex(poly.vertices_[i], poly.vertices_[i1], bestT / edgeLen);
                    newV.SetPosition(bestPos);

                    poly.vertices_.insert(poly.vertices_.begin() + i + 1, newV);
                    changedLocal = true;
                    ++insertedTotal;
                    --i; // Re-check the shortened edge.
                }
            }
        }

        return changedLocal;
    };

    while (changed)
    {
        changed = runPass(true);
        if (!changed)
            changed = runPass(false);
    }

    if (outPasses)
        *outPasses = pass;
    return insertedTotal;
}

} // namespace

bool CsgSimplifyPolygons(ea::vector<CsgPolygon>& polygons, float epsilon)
{
    // Assumptions:
    // - Input polygons may be in any order; we group by (shared_, oriented plane).
    // - Input is not pre-cleaned; only minimal-correct steps described in documentation are applied.
    URHO3D_PROFILE_FUNCTION();

    const unsigned polygonsBefore = polygons.size();

    if (polygons.empty())
        return false;

    if (epsilon <= 0.0f)
        epsilon = CSG_DEFAULT_EPSILON;

    // Group-and-merge stage is only meaningful for 2+ polygons.
    if (polygons.size() < 2)
        return false;

    const float normalEps = ea::max(1e-4f, epsilon * 0.1f);
    const float epsSq = epsilon * epsilon;

    // Step 1: Group by (shared_, oriented plane). No plane canonicalization.
    ea::fixed_vector<PlaneGroup, 32, true> planeGroups;
    planeGroups.reserve(polygons.size());

    for (auto& p : polygons)
    {
        bool placed = false;
        for (auto& g : planeGroups)
        {
            if (g.shared_ == p.shared_ && AreCoplanarOriented(g.plane_, p.plane_, normalEps, epsilon))
            {
                g.polys_.push_back(ea::move(p));
                placed = true;
                break;
            }
        }

        if (!placed)
        {
            PlaneGroup g;
            g.plane_ = p.plane_;
            g.shared_ = p.shared_;
            g.polys_.push_back(ea::move(p));
            planeGroups.push_back(ea::move(g));
        }
    }

    polygons.clear();

    // Step 2: Per-group conservative merging.
    for (auto& g : planeGroups)
    {
        auto& group = g.polys_;
        if (group.empty())
            continue;

        // Pass A: Merge polygons that share a full edge (reverse edge match) to a fixpoint.
        MergeByFullEdgesToFixpoint(group, epsilon, normalEps, epsSq);

        // Pass B: Resolve coplanar T-junctions, then merge again; repeat until no net change.
        if (group.size() > 1)
        {
            for (;;)
            {
                const unsigned polysBefore = group.size();
                unsigned vertsBefore = 0;
                for (const auto& p : group)
                    vertsBefore += p.vertices_.size();

                ResolveTJunctions(group, epsilon, nullptr);
                MergeByFullEdgesToFixpoint(group, epsilon, normalEps, epsSq);

                const unsigned polysAfter = group.size();
                unsigned vertsAfter = 0;
                for (const auto& p : group)
                    vertsAfter += p.vertices_.size();

                if (polysAfter == polysBefore && vertsAfter == vertsBefore)
                    break;
            }
        }

        // Step 3: Emit non-degenerate polygons back to output.
        for (auto& p : group)
        {
            if (p.vertices_.size() >= 3)
                polygons.push_back(ea::move(p));
        }
    }

    // Return true iff polygon count changed (vertex-only changes are not reported).
    return polygons.size() != polygonsBefore;
}

} // namespace Urho3D