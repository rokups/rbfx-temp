// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/CSG/CsgBsp.h"
#include "Urho3D/Math/BoundingBox.h"
#include "Urho3D/Math/MathDefs.h"

#include <EASTL/algorithm.h>
#include <EASTL/fixed_vector.h>
#include <EASTL/sort.h>

namespace Urho3D
{

namespace
{

void FlipModelVertex(ModelVertex& v)
{
    v.normal_ = -v.normal_;
    v.tangent_ = Vector4(-v.tangent_.x_, -v.tangent_.y_, -v.tangent_.z_, v.tangent_.w_);
    if (v.HasBinormal())
        v.binormal_ = -v.binormal_;
}

void SplitPolygon(const CsgPolygon& poly, const Plane& plane, CsgPolygon& frontPoly, CsgPolygon& backPoly, float epsilon)
{
    const auto& vertices = poly.vertices_;
    const unsigned count = vertices.size();

    if (count < 3)
        return;

    ea::fixed_vector<float, 16, true> dists;
    dists.reserve(count);
    int frontCount = 0;
    int backCount = 0;

    for (const auto& v : vertices)
    {
        const float d = plane.Distance(v.GetPosition());
        dists.push_back(d);
        if (d > epsilon)
            frontCount++;
        else if (d < -epsilon)
            backCount++;
    }

    if (backCount == 0)
    {
        frontPoly = poly;
        return;
    }
    if (frontCount == 0)
    {
        backPoly = poly;
        return;
    }

    frontPoly.shared_ = poly.shared_;
    frontPoly.plane_ = poly.plane_;
    backPoly.shared_ = poly.shared_;
    backPoly.plane_ = poly.plane_;

    // Worst case may add up to one extra vertex per edge, but we can at least avoid the first few reallocations.
    frontPoly.vertices_.reserve(count + 2);
    backPoly.vertices_.reserve(count + 2);

    for (unsigned i = 0; i < count; ++i)
    {
        const unsigned j = (i + 1) % count;
        const auto& v1 = vertices[i];
        const auto& v2 = vertices[j];
        const float d1 = dists[i];
        const float d2 = dists[j];

        const bool v1Front = d1 >= -epsilon;
        const bool v1Back = d1 <= epsilon;

        if (v1Front)
            frontPoly.vertices_.push_back(v1);
        if (v1Back)
            backPoly.vertices_.push_back(v1);

        const bool crossing = (d1 > epsilon && d2 < -epsilon) || (d1 < -epsilon && d2 > epsilon);
        if (crossing)
        {
            float t = d1 / (d1 - d2);
            t = Clamp(t, 0.0f, 1.0f);

            const ModelVertex mid = LerpModelVertex(v1, v2, t);
            frontPoly.vertices_.push_back(mid);
            backPoly.vertices_.push_back(mid);
        }
    }
}

template <bool Move, class PolygonList>
ea::vector<CsgPolygon> ClipImpl(const BspNode& node, PolygonList& polygons, CsgClipMode mode, float epsilon)
{
    if (polygons.empty())
        return {};

    ea::vector<CsgPolygon> frontList;
    ea::vector<CsgPolygon> backList;
    for (auto& poly : polygons)
    {
        int frontCount = 0;
        int backCount = 0;

        for (const auto& v : poly.vertices_)
        {
            const float d = node.plane_.normal_.DotProduct(v.GetPosition()) + node.plane_.d_;
            if (d > epsilon)
                frontCount++;
            else if (d < -epsilon)
                backCount++;
        }

        if (frontCount == 0 && backCount == 0)
        {
            auto& list = node.plane_.normal_.DotProduct(poly.plane_.normal_) > 0.0f ? frontList : backList;
            if constexpr (Move)
                list.push_back(ea::move(poly));
            else
                list.push_back(poly);
        }
        else if (frontCount > 0 && backCount == 0)
        {
            if constexpr (Move)
                frontList.push_back(ea::move(poly));
            else
                frontList.push_back(poly);
        }
        else if (backCount > 0 && frontCount == 0)
        {
            if constexpr (Move)
                backList.push_back(ea::move(poly));
            else
                backList.push_back(poly);
        }
        else
        {
            CsgPolygon f, b;
            SplitPolygon(poly, node.plane_, f, b, epsilon);
            if (!f.vertices_.empty())
                frontList.push_back(ea::move(f));
            if (!b.vertices_.empty())
                backList.push_back(ea::move(b));
        }
    }

    ea::vector<CsgPolygon> result;

    if (node.front_)
    {
        auto f = node.front_->Clip(ea::move(frontList), mode, epsilon);
        result.insert(result.end(), std::make_move_iterator(f.begin()), std::make_move_iterator(f.end()));
    }
    else if (mode == CsgClipMode::ClipToOutside)
        result.insert(result.end(), std::make_move_iterator(frontList.begin()), std::make_move_iterator(frontList.end()));

    if (node.back_)
    {
        auto b = node.back_->Clip(ea::move(backList), mode, epsilon);
        result.insert(result.end(), std::make_move_iterator(b.begin()), std::make_move_iterator(b.end()));
    }
    else if (mode == CsgClipMode::ClipToInside)
        result.insert(result.end(), std::make_move_iterator(backList.begin()), std::make_move_iterator(backList.end()));

    return result;
}

float CalculatePlaneScore(const Plane& plane, const ea::vector<CsgPolygon>& polygons, float epsilon)
{
    const float K_split = 1.0f;
    const float K_balance = 0.5f;
    const float K_coplanar = 5.0f;

    int frontCount = 0;
    int backCount = 0;
    int splitCount = 0;
    int nearCoplanarSplitCount = 0;

    const unsigned maxPolysToCheck = 100;
    const unsigned stride = (polygons.size() > maxPolysToCheck) ? (polygons.size() / maxPolysToCheck) : 1;

    for (unsigned i = 0; i < polygons.size(); i += stride)
    {
        const auto& poly = polygons[i];
        int vFront = 0;
        int vBack = 0;

        for (const auto& v : poly.vertices_)
        {
            const float d = plane.normal_.DotProduct(v.GetPosition()) + plane.d_;
            if (d > epsilon)
                vFront++;
            else if (d < -epsilon)
                vBack++;
        }

        if (vFront > 0 && vBack > 0)
        {
            splitCount++;
            if (Abs(plane.normal_.DotProduct(poly.plane_.normal_)) > 0.99f)
                nearCoplanarSplitCount++;
        }
        else if (vFront > 0)
            frontCount++;
        else if (vBack > 0)
            backCount++;
    }

    return K_split * splitCount + K_balance * Abs((float)frontCount - (float)backCount) + K_coplanar * nearCoplanarSplitCount;
}

Plane PickSplittingPlane(const ea::vector<CsgPolygon>& polygons, float epsilon)
{
    if (polygons.empty())
        return Plane::UP;

    Plane bestPlane = polygons[polygons.size() / 2].plane_;
    float bestScore = 1e30f;

    const unsigned numCandidates = Clamp((unsigned)(polygons.size() / 3), 1u, 5u);
    unsigned step = polygons.size() / numCandidates;
    if (step == 0)
        step = 1;

    for (unsigned i = 0; i < polygons.size(); i += step)
    {
        const Plane& candidate = polygons[i].plane_;
        const float score = CalculatePlaneScore(candidate, polygons, epsilon);
        if (score < bestScore)
        {
            bestScore = score;
            bestPlane = candidate;
        }
    }
    return bestPlane;
}

Vector3 GetPolygonCentroid(const CsgPolygon& poly)
{
    Vector3 center = Vector3::ZERO;
    for (const auto& v : poly.vertices_)
        center += v.GetPosition();
    if (!poly.vertices_.empty())
        center /= (float)poly.vertices_.size();
    return center;
}

uint32_t ExpandBits(uint32_t v)
{
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

uint32_t Morton3D(const Vector3& p)
{
    const float x = Clamp(p.x_, 0.0f, 1.0f);
    const float y = Clamp(p.y_, 0.0f, 1.0f);
    const float z = Clamp(p.z_, 0.0f, 1.0f);

    const uint32_t xx = ExpandBits((uint32_t)(x * 1023.0f));
    const uint32_t yy = ExpandBits((uint32_t)(y * 1023.0f));
    const uint32_t zz = ExpandBits((uint32_t)(z * 1023.0f));

    return xx * 4 + yy * 2 + zz;
}

} // namespace

CsgPolygon::CsgPolygon() = default;

CsgPolygon::CsgPolygon(ea::vector<ModelVertex>&& verts, unsigned sharedKey)
    : vertices_(ea::move(verts))
    , shared_(sharedKey)
{
    if (vertices_.size() >= 3)
        plane_ = Plane(vertices_[0].GetPosition(), vertices_[1].GetPosition(), vertices_[2].GetPosition());
}

CsgPolygon::CsgPolygon(CsgPolygon&&) = default;
CsgPolygon& CsgPolygon::operator=(CsgPolygon&&) = default;

CsgPolygon::CsgPolygon(const CsgPolygon&) = default;
CsgPolygon& CsgPolygon::operator=(const CsgPolygon&) = default;

void CsgPolygon::Flip()
{
    ea::reverse(vertices_.begin(), vertices_.end());
    for (auto& v : vertices_)
        FlipModelVertex(v);
    plane_.normal_ = -plane_.normal_;
    plane_.d_ = -plane_.d_;
}

bool CsgPolygon::IsConvex() const
{
    if (vertices_.size() < 3)
        return false;

    const unsigned count = vertices_.size();
    for (unsigned i = 0; i < count; ++i)
    {
        const Vector3& p0 = vertices_[i].GetPosition();
        const Vector3& p1 = vertices_[(i + 1) % count].GetPosition();
        const Vector3& p2 = vertices_[(i + 2) % count].GetPosition();

        const Vector3 edge1 = p1 - p0;
        const Vector3 edge2 = p2 - p1;
        const Vector3 cross = edge1.CrossProduct(edge2);

        if (cross.DotProduct(plane_.normal_) < -M_EPSILON)
            return false;
    }
    return true;
}

void BspNode::Build(ea::vector<CsgPolygon>&& polygons, float epsilon)
{
    if (polygons.empty())
        return;

    // Termination heuristic: if there is only one polygon, further splitting cannot improve
    // the BSP and may lead to pathological recursion on degenerate inputs.
    if (polygons.size() == 1)
    {
        // `ClipImpl` always classifies polygons against `plane_`, even for leaf nodes,
        // so ensure it's initialized to a meaningful plane.
        auto& poly = polygons.front();
        if (poly.vertices_.size() >= 3 && poly.plane_.normal_.LengthSquared() <= M_EPSILON)
            poly.plane_ = Plane(poly.vertices_[0].GetPosition(), poly.vertices_[1].GetPosition(), poly.vertices_[2].GetPosition());

        plane_ = poly.plane_;
        polygons_.push_back(ea::move(poly));
        return;
    }

    plane_ = PickSplittingPlane(polygons, epsilon);

    ea::vector<CsgPolygon> frontList;
    ea::vector<CsgPolygon> backList;
    for (auto& poly : polygons)
    {
        int frontCount = 0;
        int backCount = 0;
        int coplanarCount = 0;

        for (const auto& v : poly.vertices_)
        {
            const float d = plane_.normal_.DotProduct(v.GetPosition()) + plane_.d_;
            if (d > epsilon)
                frontCount++;
            else if (d < -epsilon)
                backCount++;
            else
                coplanarCount++;
        }

        if (coplanarCount == (int)poly.vertices_.size())
        {
            polygons_.push_back(ea::move(poly));
        }
        else if (frontCount > 0 && backCount == 0)
        {
            frontList.push_back(ea::move(poly));
        }
        else if (backCount > 0 && frontCount == 0)
        {
            backList.push_back(ea::move(poly));
        }
        else
        {
            CsgPolygon f, b;
            SplitPolygon(poly, plane_, f, b, epsilon);
            if (!f.vertices_.empty())
                frontList.push_back(ea::move(f));
            if (!b.vertices_.empty())
                backList.push_back(ea::move(b));
        }
    }

    if (!frontList.empty())
    {
        front_ = ea::make_unique<BspNode>();
        front_->Build(ea::move(frontList), epsilon);
    }

    if (!backList.empty())
    {
        back_ = ea::make_unique<BspNode>();
        back_->Build(ea::move(backList), epsilon);
    }
}

ea::vector<CsgPolygon> BspNode::Clip(const ea::vector<CsgPolygon>& polygons, CsgClipMode mode, float epsilon) const
{
    return ClipImpl<false>(*this, polygons, mode, epsilon);
}

ea::vector<CsgPolygon> BspNode::Clip(ea::vector<CsgPolygon>&& polygons, CsgClipMode mode, float epsilon) const
{
    return ClipImpl<true>(*this, polygons, mode, epsilon);
}

void BspNode::Invert()
{
    plane_.normal_ = -plane_.normal_;
    plane_.d_ = -plane_.d_;

    for (auto& poly : polygons_)
        poly.Flip();

    if (front_)
        front_->Invert();
    if (back_)
        back_->Invert();

    ea::swap(front_, back_);
}

CsgBsp::CsgBsp() = default;

void CsgBsp::Build(ea::vector<CsgPolygon>&& polygons, float epsilon)
{
    if (polygons.empty())
        return;

    BoundingBox bbox;
    for (const auto& poly : polygons)
    {
        for (const auto& v : poly.vertices_)
            bbox.Merge(v.GetPosition());
    }

    const Vector3 size = bbox.Size();
    const Vector3 min = bbox.min_;
    const Vector3 scale(
        size.x_ > 0.0f ? 1.0f / size.x_ : 0.0f,
        size.y_ > 0.0f ? 1.0f / size.y_ : 0.0f,
        size.z_ > 0.0f ? 1.0f / size.z_ : 0.0f);

    struct KeyedPolygon
    {
        uint32_t key_{};
        CsgPolygon polygon_;
    };

    ea::vector<KeyedPolygon> keyed;
    keyed.reserve(polygons.size());

    for (auto& poly : polygons)
    {
        const Vector3 c = GetPolygonCentroid(poly);
        const uint32_t key = Morton3D((c - min) * scale);
        keyed.push_back({key, ea::move(poly)});
    }

    ea::sort(keyed.begin(), keyed.end(), [](const KeyedPolygon& a, const KeyedPolygon& b) {
        return a.key_ < b.key_;
    });

    polygons.clear();
    polygons.reserve(keyed.size());
    for (auto& item : keyed)
        polygons.push_back(ea::move(item.polygon_));

    root_ = ea::make_unique<BspNode>();
    root_->Build(ea::move(polygons), epsilon);
}

ea::vector<CsgPolygon> CsgBsp::Clip(const ea::vector<CsgPolygon>& polygons, CsgClipMode mode, float epsilon) const
{
    if (!root_)
    {
        if (mode == CsgClipMode::ClipToInside)
            return {};
        return polygons;
    }

    return root_->Clip(polygons, mode, epsilon);
}

ea::vector<CsgPolygon> CsgBsp::Clip(ea::vector<CsgPolygon>&& polygons, CsgClipMode mode, float epsilon) const
{
    if (!root_)
    {
        if (mode == CsgClipMode::ClipToInside)
            return {};
        return ea::move(polygons);
    }

    return root_->Clip(ea::move(polygons), mode, epsilon);
}

void CsgBsp::Invert()
{
    if (root_)
        root_->Invert();
}

ea::vector<CsgPolygon> CsgBsp::AllPolygons() const
{
    ea::vector<CsgPolygon> result;
    CollectPolygons(root_.get(), result);
    return result;
}

ea::vector<CsgPolygon> CsgBsp::AllPolygonsInverted() const
{
    auto result = AllPolygons();
    for (auto& polygon : result)
        polygon.Flip();
    return result;
}

void CsgBsp::CollectPolygons(const BspNode* node, ea::vector<CsgPolygon>& result) const
{
    if (!node)
        return;
    result.insert(result.end(), node->polygons_.begin(), node->polygons_.end());
    CollectPolygons(node->front_.get(), result);
    CollectPolygons(node->back_.get(), result);
}

} // namespace Urho3D
