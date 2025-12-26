// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgBsp.h"

#include <EASTL/vector.h>

namespace Urho3D
{

bool CsgSimplifyPolygons(ea::vector<CsgPolygon>& polygons, float epsilon);

}   // namespace Urho3D
