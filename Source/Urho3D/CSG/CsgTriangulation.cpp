// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgTriangulation.h"

#include "Urho3D/CSG/CsgOptimization.h"
#include "Urho3D/Container/Ptr.h"
#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/Math/MathDefs.h"
#include "Urho3D/Math/Vector2.h"
#include "Urho3D/Math/Vector3.h"

#include <EASTL/hash_map.h>
#include <EASTL/sort.h>

namespace Urho3D
{

bool CsgTriangulatePolygon(const CsgPolygon& poly, ea::vector<unsigned>& outIndices)
{
    const unsigned numVertices = poly.vertices_.size();
    if (numVertices < 3)
        return false;

    outIndices.clear();
    if (numVertices == 3)
    {
        outIndices = {0, 1, 2};
        return true;
    }

    // Project to 2D for robustness
    Vector3 axisU, axisV;
    {
        MakeAxesFromNormal(poly.plane_.normal_, axisU, axisV);
    }

    ea::vector<Vector2> verts2D(numVertices);
    for (unsigned i = 0; i < numVertices; ++i)
    {
        Vector3 p = poly.vertices_[i].GetPosition();
        verts2D[i] = {p.DotProduct(axisU), p.DotProduct(axisV)};
    }

    ea::vector<unsigned> indices(numVertices);
    for (unsigned i = 0; i < numVertices; ++i)
        indices[i] = i;

    unsigned count = numVertices;
    int maxIterations = static_cast<int>(count * 2);

    while (count > 2)
    {
        if (maxIterations-- <= 0)
            break;

        bool earFound = false;

        // Pass 1: Look for a valid ear
        for (unsigned i = 0; i < count; ++i)
        {
            unsigned prev = (i + count - 1) % count;
            unsigned next = (i + 1) % count;

            unsigned u = indices[prev];
            unsigned v = indices[i];
            unsigned w = indices[next];

            // Check convexity using 3D normal
            const Vector3& p0_3d = poly.vertices_[u].GetPosition();
            const Vector3& p1_3d = poly.vertices_[v].GetPosition();
            const Vector3& p2_3d = poly.vertices_[w].GetPosition();
            Vector3 e1_3d = p1_3d - p0_3d;
            Vector3 e2_3d = p2_3d - p1_3d;

            if (e1_3d.CrossProduct(e2_3d).DotProduct(poly.plane_.normal_) < -M_EPSILON)
                continue; // Reflex

            const Vector2& p0 = verts2D[u];
            const Vector2& p1 = verts2D[v];
            const Vector2& p2 = verts2D[w];

            // Check if any other vertex is inside the triangle
            bool containsVertex = false;
            for (unsigned j = 0; j < count; ++j)
            {
                if (j == prev || j == i || j == next)
                    continue;

                const Vector2& p = verts2D[indices[j]];

                Vector2 v0 = p2 - p0;
                Vector2 v1 = p1 - p0;
                Vector2 v2 = p - p0;

                float dot00 = v0.DotProduct(v0);
                float dot01 = v0.DotProduct(v1);
                float dot02 = v0.DotProduct(v2);
                float dot11 = v1.DotProduct(v1);
                float dot12 = v1.DotProduct(v2);

                float denom = dot00 * dot11 - dot01 * dot01;
                if (Abs(denom) < M_EPSILON)
                    continue;

                float invDenom = 1.0f / denom;
                float u_bary = (dot11 * dot02 - dot01 * dot12) * invDenom;
                float v_bary = (dot00 * dot12 - dot01 * dot02) * invDenom;

                if (u_bary >= -1e-5f && v_bary >= -1e-5f && (u_bary + v_bary) <= 1.0f + 1e-5f)
                {
                    containsVertex = true;
                    break;
                }
            }

            if (!containsVertex)
            {
                outIndices.push_back(u);
                outIndices.push_back(v);
                outIndices.push_back(w);

                indices.erase(indices.begin() + i);
                --count;
                earFound = true;
                break;
            }
        }

        // Pass 2: Force clip first convex ear
        if (!earFound)
        {
            for (unsigned i = 0; i < count; ++i)
            {
                unsigned prev = (i + count - 1) % count;
                unsigned next = (i + 1) % count;
                unsigned u = indices[prev];
                unsigned v = indices[i];
                unsigned w = indices[next];

                const Vector3& p0_3d = poly.vertices_[u].GetPosition();
                const Vector3& p1_3d = poly.vertices_[v].GetPosition();
                const Vector3& p2_3d = poly.vertices_[w].GetPosition();
                Vector3 e1_3d = p1_3d - p0_3d;
                Vector3 e2_3d = p2_3d - p1_3d;

                if (e1_3d.CrossProduct(e2_3d).DotProduct(poly.plane_.normal_) >= -M_EPSILON)
                {
                    outIndices.push_back(u);
                    outIndices.push_back(v);
                    outIndices.push_back(w);
                    indices.erase(indices.begin() + i);
                    --count;
                    earFound = true;
                    break;
                }
            }
        }

        // Pass 3: Force clip any ear
        if (!earFound)
        {
            unsigned i = 0;
            unsigned prev = (i + count - 1) % count;
            unsigned next = (i + 1) % count;
            outIndices.push_back(indices[prev]);
            outIndices.push_back(indices[i]);
            outIndices.push_back(indices[next]);
            indices.erase(indices.begin() + i);
            --count;
        }
    }

    return true;
}

bool CsgTriangulateGeometry(
    const ea::vector<CsgPolygon>& polys, float epsilon,
    ea::vector<ModelVertex>& outVertices, ea::vector<unsigned>& outIndexData)
{
    outVertices.clear();
    outIndexData.clear();

    if (polys.empty())
        return false;

    const float eps = ea::max(epsilon, 1e-6f);
    const float invEps = 1.0f / eps;

    unsigned estimatedVertexCount = 0;
    unsigned estimatedIndexCount = 0;
    for (const auto& poly : polys)
    {
        estimatedVertexCount += poly.vertices_.size();
        if (poly.vertices_.size() >= 3)
            estimatedIndexCount += (poly.vertices_.size() - 2u) * 3u;
    }

    outVertices.reserve(estimatedVertexCount);
    outIndexData.reserve(estimatedIndexCount);

    ea::hash_map<IntVector3, ea::vector<unsigned>> buckets;
    buckets.reserve(estimatedVertexCount * 2u);

    const auto getOrCreateVertexIndex = [&](const ModelVertex& vertex) -> unsigned
    {
        const IntVector3 key = QuantizePosition(vertex.GetPosition(), invEps);
        auto& bucket = buckets[key];

        for (const unsigned existingIndex : bucket)
        {
            const ModelVertex& existing = outVertices[existingIndex];

            // Fast reject by position with CSG epsilon.
            if (!existing.position_.Equals(vertex.position_, eps))
                continue;

            // Other attributes are expected to match very closely if the vertex is truly shared.
            if (existing == vertex)
                return existingIndex;
        }

        const unsigned newIndex = outVertices.size();
        outVertices.push_back(vertex);
        bucket.push_back(newIndex);
        return newIndex;
    };

    ea::vector<unsigned> triIndices;
    for (const auto& poly : polys)
    {
        if (poly.vertices_.size() < 3)
            continue;

        if (!CsgTriangulatePolygon(poly, triIndices))
            continue;

        for (const unsigned polyVertexIndex : triIndices)
            outIndexData.push_back(getOrCreateVertexIndex(poly.vertices_[polyVertexIndex]));
    }

    return !outIndexData.empty();
}

SharedPtr<CsgTriangulatedModel> CsgTriangulatePolygons(ea::vector<CsgPolygon> polygons, float epsilon)
{
    auto result = MakeShared<CsgTriangulatedModel>();

    if (polygons.empty())
        return result;

    ea::hash_map<unsigned, ea::vector<CsgPolygon>> groups;
    for (auto& p : polygons)
        groups[p.shared_].push_back(ea::move(p));

    ea::vector<unsigned> groupKeys;
    groupKeys.reserve(groups.size());
    for (const auto& pair : groups)
        groupKeys.push_back(pair.first);
    ea::sort(groupKeys.begin(), groupKeys.end());

    result->geometries_.reserve(groupKeys.size());

    for (const unsigned sharedKey : groupKeys)
    {
        auto& polys = groups[sharedKey];

        CsgSimplifyPolygons(polys, epsilon);

        CsgTriangulatedModel::Geometry geometry;
        geometry.shared_ = sharedKey;

        if (!CsgTriangulateGeometry(polys, epsilon, geometry.vertices_, geometry.indexData_))
            continue;

        if (geometry.vertices_.empty() || geometry.indexData_.empty())
            continue;

        for (const auto& v : geometry.vertices_)
        {
            const Vector3 pos = v.GetPosition();
            if (!result->hasAnyVertex_)
            {
                result->boundingBox_ = BoundingBox(pos, pos);
                result->hasAnyVertex_ = true;
            }
            else
                result->boundingBox_.Merge(pos);
        }

        result->geometries_.push_back(ea::move(geometry));
    }

    return result;
}

} // namespace Urho3D
