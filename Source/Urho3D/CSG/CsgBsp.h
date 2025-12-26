// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/Graphics/ModelView.h"
#include "Urho3D/Math/Plane.h"

#include <EASTL/unique_ptr.h>
#include <EASTL/vector.h>

namespace Urho3D
{

enum class CsgClipMode
{
    ClipToInside,
    ClipToOutside
};

struct URHO3D_API CsgPolygon
{
    ea::vector<ModelVertex> vertices_;
    unsigned shared_{};  // (sourceMeshId << 16) | sourceGeometryIndex
    Plane plane_;

    CsgPolygon();
    CsgPolygon(ea::vector<ModelVertex>&& verts, unsigned sharedKey);

    CsgPolygon(CsgPolygon&&);
    CsgPolygon& operator=(CsgPolygon&&);

    CsgPolygon(const CsgPolygon&);
    CsgPolygon& operator=(const CsgPolygon&);

    void Flip();
    bool IsConvex() const;
};

struct URHO3D_API BspNode
{
    Plane plane_;
    ea::vector<CsgPolygon> polygons_;
    ea::unique_ptr<BspNode> front_;
    ea::unique_ptr<BspNode> back_;

    void Build(ea::vector<CsgPolygon>&& polygons, float epsilon);
    ea::vector<CsgPolygon> Clip(const ea::vector<CsgPolygon>& polygons, CsgClipMode mode, float epsilon) const;
    ea::vector<CsgPolygon> Clip(ea::vector<CsgPolygon>&& polygons, CsgClipMode mode, float epsilon) const;
    void Invert();
};

class URHO3D_API CsgBsp
{
public:
    CsgBsp();

    /// Return true if BSP tree is built (non-empty).
    explicit operator bool() const noexcept { return root_ != nullptr; }

    void Build(ea::vector<CsgPolygon>&& polygons, float epsilon = CSG_DEFAULT_EPSILON);
    ea::vector<CsgPolygon> Clip(const ea::vector<CsgPolygon>& polygons, CsgClipMode mode, float epsilon = CSG_DEFAULT_EPSILON) const;
    ea::vector<CsgPolygon> Clip(ea::vector<CsgPolygon>&& polygons, CsgClipMode mode, float epsilon = CSG_DEFAULT_EPSILON) const;

    void Invert();
    ea::vector<CsgPolygon> AllPolygons() const;
    ea::vector<CsgPolygon> AllPolygonsInverted() const;

private:
    void CollectPolygons(const BspNode* node, ea::vector<CsgPolygon>& result) const;

    ea::unique_ptr<BspNode> root_;
};

} // namespace Urho3D
