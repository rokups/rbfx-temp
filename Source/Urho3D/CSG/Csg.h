// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/Container/Ptr.h"
#include "Urho3D/Math/Matrix3x4.h"
#include "Urho3D/Urho3D.h"

#include <EASTL/utility.h>
#include <EASTL/span.h>
#include <EASTL/vector.h>

namespace Urho3D
{

class Context;
class CsgBsp;
class CsgTriangulatedModel;
class Model;
struct CsgPolygon;
struct ResourceRefList;

enum class CsgOperation
{
    Union,
    DifferenceAB,
    Intersection,
};

static const char* CsgOperationNames[] = {
    "Union",
    "DifferenceAB",
    "Intersection",
    nullptr
};

/// Compute CSG boolean operation between two BSP operands and return resulting polygons.
///
/// `epsilon` is used consistently for BSP clipping in this operation.
URHO3D_API ea::vector<CsgPolygon> CsgBooleanOperation(
    const CsgBsp& a, const CsgBsp& b, CsgOperation op, float epsilon = CSG_DEFAULT_EPSILON);

/// Extract CSG polygons from a `Model` operand.
///
/// `worldTransform` is applied to vertex positions (and normals/tangents) during extraction.
/// `meshId` is encoded into `CsgPolygon::shared_` as (meshId << 16) | geometryIndex.
URHO3D_API ea::vector<CsgPolygon> CsgBuildPolygonsFromModel(
    const Model* model, const Matrix3x4& worldTransform, unsigned meshId = 0);

/// Build BSP tree from a `Model` operand.
///
/// `worldTransform` is applied to vertex positions (and normals/tangents) during extraction.
/// `meshId` is encoded into `CsgPolygon::shared_` as (meshId << 16) | geometryIndex.
/// Returns empty BSP if extraction fails.
URHO3D_API CsgBsp CsgBuildBspFromModel(
    const Model* model,
    const Matrix3x4& worldTransform,
    unsigned meshId = 0,
    float epsilon = CSG_DEFAULT_EPSILON);

/// Triangulate polygons into CPU representation of CSG output.
URHO3D_API SharedPtr<CsgTriangulatedModel> CsgTriangulatePolygons(
    ea::vector<CsgPolygon> polygons, float epsilon = CSG_DEFAULT_EPSILON);

/// Build output `Model` from triangulated CSG result.
///
/// `sourceModels` index corresponds to `sourceMeshId` decoded from `CsgPolygon::shared_`.
/// `sourceMaterials` index corresponds to `sourceMeshId`, and resource list index corresponds
/// to `sourceGeometryIndex` decoded from `CsgPolygon::shared_`.
URHO3D_API SharedPtr<Model> CsgBuildModel(
    Context* context,
    const CsgTriangulatedModel& triangulated,
    ea::span<const Model* const> sourceModels,
    ea::span<const ResourceRefList* const> sourceMaterials,
    ResourceRefList* outMaterials = nullptr,
    ModelViewExportFlags exportFlags = ModelViewExportFlag::None);

/// Register CSG library objects.
void URHO3D_API RegisterCsgLibrary(Context* context);

} // namespace Urho3D
