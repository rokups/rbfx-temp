// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/Graphics/ModelView.h"
#include "Urho3D/Math/Vector3.h"

#include <EASTL/vector.h>

namespace Urho3D
{

struct CsgPolygon;
class ModelVertex;

static constexpr float CSG_DEFAULT_EPSILON = 0.0001f;

/// Pick a stable perpendicular axis for the provided normal.
URHO3D_API Vector3 MakePerpendicular(const Vector3& normal);
/// Make stable orthonormal basis vectors for the provided normal.
URHO3D_API void MakeAxesFromNormal(const Vector3& normal, Vector3& axisU, Vector3& axisV);
/// Quantize position into integer key using 1/epsilon scale.
URHO3D_API IntVector3 QuantizePosition(const Vector3& p, float invEps);
/// Lerp vertex attributes for CSG operations.
URHO3D_API ModelVertex LerpModelVertex(const ModelVertex& a, const ModelVertex& b, float t);

}   // namespace Urho3D
