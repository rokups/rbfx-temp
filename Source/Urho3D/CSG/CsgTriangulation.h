// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/Urho3D.h"
#include "Urho3D/CSG/CsgBsp.h"
#include "Urho3D/Container/RefCounted.h"
#include "Urho3D/Math/BoundingBox.h"

#include <EASTL/vector.h>

namespace Urho3D
{

/// Triangulated CPU representation of CSG output.
class URHO3D_API CsgTriangulatedModel : public RefCounted
{
public:
	struct Geometry
	{
		/// `(sourceMeshId << 16) | sourceGeometryIndex` copied from `CsgPolygon::shared_`.
		/// Used on the main thread to resolve source vertex layout and output material.
		unsigned shared_{};

		/// Triangulated vertex stream in unpacked `ModelVertex` format.
		ea::vector<ModelVertex> vertices_;
		ea::vector<unsigned> indexData_;
	};

	ea::vector<Geometry> geometries_;
	BoundingBox boundingBox_;
	bool hasAnyVertex_{};

	CsgTriangulatedModel() = default;
	~CsgTriangulatedModel() override = default;

	CsgTriangulatedModel(const CsgTriangulatedModel&) = delete;
	CsgTriangulatedModel& operator=(const CsgTriangulatedModel&) = delete;
};

/// Triangulate polygon into triangle list indices (triplets).
URHO3D_API bool CsgTriangulatePolygon(const CsgPolygon& poly, ea::vector<unsigned>& outIndices);

/// Triangulate a collection of polygons into one indexed triangle list.
/// Output vertices are returned as full ModelVertex (unpacked).
URHO3D_API bool CsgTriangulateGeometry(
	const ea::vector<CsgPolygon>& polys, float epsilon,
	ea::vector<ModelVertex>& outVertices, ea::vector<unsigned>& outIndexData);

} // namespace Urho3D
