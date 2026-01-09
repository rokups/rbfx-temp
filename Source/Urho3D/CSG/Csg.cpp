// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/CsgBsp.h"
#include "Urho3D/CSG/CsgCommon.h"
#include "Urho3D/CSG/CsgComponent.h"
#include "Urho3D/CSG/CsgRoot.h"
#include "Urho3D/CSG/CsgTriangulation.h"
#include "Urho3D/Container/ByteVector.h"
#include "Urho3D/Core/Context.h"
#include "Urho3D/Core/Variant.h"
#include "Urho3D/Graphics/Geometry.h"
#include "Urho3D/Graphics/IndexBuffer.h"
#include "Urho3D/Graphics/Material.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/Graphics/ModelView.h"
#include "Urho3D/Graphics/VertexBuffer.h"
#include "Urho3D/Math/Matrix3.h"
#include "Urho3D/Math/Vector3.h"
#include "Urho3D/Math/Vector4.h"
#include "Urho3D/RenderAPI/RenderAPIDefs.h"

#include <EASTL/algorithm.h>
#include <EASTL/fixed_vector.h>
#include <EASTL/hash_map.h>
#include <EASTL/hash_set.h>
#include <EASTL/sort.h>

#include <cassert>
#include <type_traits>

namespace Urho3D
{

namespace
{

inline unsigned PackSharedKey(unsigned meshId, unsigned geometryIndex)
{
    return (meshId << 16u) | (geometryIndex & 0xFFFFu);
}

inline ea::pair<unsigned, unsigned> UnpackSharedKey(unsigned sharedKey)
{
    const unsigned meshId = (sharedKey >> 16u) & 0xFFFFu;
    const unsigned geometryIndex = sharedKey & 0xFFFFu;
    return {meshId, geometryIndex};
}

bool IsSupportedCsgVertexElement(const VertexElement& element)
{
    if (element.stepRate_ != 0)
        return false; // Per-instance data is not supported.

    switch (element.semantic_)
    {
    case SEM_POSITION:
    case SEM_NORMAL:
    case SEM_BINORMAL:
    case SEM_TANGENT:
    case SEM_TEXCOORD:
    case SEM_COLOR: return true;

    case SEM_BLENDWEIGHTS:
    case SEM_BLENDINDICES:
    case SEM_OBJECTINDEX:
    default: return false;
    }
}

ea::vector<VertexElement> GetSupportedVertexElements(const Geometry* sourceGeometry)
{
    assert(sourceGeometry);
    ea::vector<VertexElement> elements;

    if (sourceGeometry)
    {
        unsigned vertexSize = 0;
        unsigned indexSize = 0;
        const unsigned char* vertexData = nullptr;
        const unsigned char* indexData = nullptr;
        const ea::vector<VertexElement>* srcElements = nullptr;
        sourceGeometry->GetRawData(vertexData, vertexSize, indexData, indexSize, srcElements);
        assert(vertexData);
        assert(srcElements);

        elements.reserve(srcElements->size());
        ea::copy_if(srcElements->begin(), srcElements->end(), ea::back_inserter(elements), IsSupportedCsgVertexElement);
    }
    else
    {
        elements.emplace_back(TYPE_VECTOR3, SEM_POSITION, 0);
    }

    VertexBuffer::UpdateOffsets(elements);
    return elements;
}

ByteVector PackVerticesToLayout(const ea::vector<ModelVertex>& vertices, const ea::vector<VertexElement>& elements)
{
    // `ModelVertex` is documented to be equivalent to an array of Vector4. This function relies on that.
    // Keep these checks close to the usage to fail loudly if ModelVertex layout changes.
    constexpr unsigned kModelVertexVector4Count = 6u + ModelVertex::MaxColors + ModelVertex::MaxUVs;
    static_assert(std::is_standard_layout_v<ModelVertex>, "ModelVertex must be standard-layout");
    static_assert(alignof(ModelVertex) == alignof(Vector4), "ModelVertex and Vector4 alignment must match");
    static_assert(sizeof(ModelVertex) == sizeof(Vector4) * kModelVertexVector4Count, "ModelVertex must be exactly N*Vector4 bytes");

    const unsigned vertexCount = vertices.size();
    const unsigned vertexSize = VertexBuffer::GetVertexSize(elements);

    ByteVector packed;
    packed.resize(vertexCount * vertexSize);
    if (vertexCount == 0)
        return packed;

    // ModelVertex is equivalent to an array of Vector4 and vertices are contiguous in memory.
    const Vector4* srcUnpacked = reinterpret_cast<const Vector4*>(vertices.data());
    ea::vector<Vector4> dstUnpacked(vertexCount * elements.size());

    VertexBuffer::ShuffleUnpackedVertexData(
        vertexCount, srcUnpacked, ModelVertex::VertexElements, dstUnpacked.data(), elements, true);

    const unsigned srcStride = sizeof(Vector4) * elements.size();
    for (unsigned elementIndex = 0; elementIndex < elements.size(); ++elementIndex)
    {
        VertexBuffer::PackVertexData(dstUnpacked.data() + elementIndex, srcStride, packed.data(), vertexSize,
            elements[elementIndex], 0, vertexCount);
    }

    return packed;
}

} // namespace

ea::vector<CsgPolygon> CsgBuildPolygonsFromModel(const Model* model, const Matrix3x4& worldTransform, unsigned meshId)
{
    ea::vector<CsgPolygon> polygons;
    if (!model)
        return polygons;

    const Matrix3 basis = worldTransform.ToMatrix3();
    const bool basisOk = !basis.IsNaN() && !basis.IsInf() && Abs(basis.Determinant()) > M_EPSILON;
    const Matrix3 normalMatrix = basisOk ? basis.Inverse().Transpose() : Matrix3::IDENTITY;

    const unsigned numGeometries = model->GetNumGeometries();
    for (unsigned i = 0; i < numGeometries; ++i)
    {
        Geometry* geom = model->GetGeometry(i, 0);
        if (!geom)
            continue;

        unsigned vertexSize = 0;
        unsigned indexSize = 0;
        const unsigned char* vertexData = nullptr;
        const unsigned char* indexData = nullptr;
        const ea::vector<VertexElement>* elements = nullptr;

        geom->GetRawData(vertexData, vertexSize, indexData, indexSize, elements);
        if (!vertexData || !elements)
            continue;

        // Element lookup table
        ea::fixed_vector<const VertexElement*, MAX_VERTEX_ELEMENT_SEMANTICS * 4> elementPointers;
        elementPointers.resize(elementPointers.max_size(), nullptr);
        for (const VertexElement& element : *elements)
            elementPointers[(element.semantic_ << 2) + element.index_] = &element;

        const unsigned indexCount = geom->GetIndexCount();
        const unsigned vertexCount = geom->GetVertexCount();

        for (unsigned j = 0; j < indexCount; j += 3)
        {
            unsigned i0, i1, i2;
            if (!indexData)
            {
                i0 = j;
                i1 = j + 1;
                i2 = j + 2;
            }
            else if (indexSize == 2)
            {
                i0 = reinterpret_cast<const unsigned short*>(indexData)[j];
                i1 = reinterpret_cast<const unsigned short*>(indexData)[j + 1];
                i2 = reinterpret_cast<const unsigned short*>(indexData)[j + 2];
            }
            else
            {
                i0 = reinterpret_cast<const unsigned*>(indexData)[j];
                i1 = reinterpret_cast<const unsigned*>(indexData)[j + 1];
                i2 = reinterpret_cast<const unsigned*>(indexData)[j + 2];
            }

            ea::vector<ModelVertex> verts;
            for (unsigned index : {i0, i1, i2})
            {
                ModelVertex v{};

                // Position is required.
                if (const VertexElement* element = elementPointers[SEM_POSITION << 2])
                {
                    Vector4 position4;
                    VertexBuffer::UnpackVertexData(
                        vertexData, vertexSize, *element, index, 1, &position4, sizeof(Vector4));
                    v.SetPosition(worldTransform * position4.ToVector3());
                }

                // Normal.
                if (const VertexElement* element = elementPointers[SEM_NORMAL << 2])
                {
                    Vector4 normal4;
                    VertexBuffer::UnpackVertexData(
                        vertexData, vertexSize, *element, index, 1, &normal4, sizeof(Vector4));
                    const Vector3 transformedNormal = normalMatrix * normal4.ToVector3();
                    v.SetNormal(transformedNormal.NormalizedOrDefault(Vector3::UP));
                }

                // Tangent.
                if (const VertexElement* element = elementPointers[SEM_TANGENT << 2])
                {
                    Vector4 tangent4;
                    VertexBuffer::UnpackVertexData(
                        vertexData, vertexSize, *element, index, 1, &tangent4, sizeof(Vector4));
                    const Vector3 transformedTangent =
                        (basis * tangent4.ToVector3()).NormalizedOrDefault(Vector3::RIGHT);
                    v.tangent_ = Vector4(transformedTangent, tangent4.w_);
                }

                // Binormal.
                if (const VertexElement* element = elementPointers[SEM_BINORMAL << 2])
                {
                    Vector4 binormal4;
                    VertexBuffer::UnpackVertexData(
                        vertexData, vertexSize, *element, index, 1, &binormal4, sizeof(Vector4));
                    const Vector3 transformedBinormal =
                        (basis * binormal4.ToVector3()).NormalizedOrDefault(Vector3::UP);
                    v.binormal_ = Vector4(transformedBinormal, 0.0f);
                }

                // Colors and UVs.
                for (unsigned char i = 0; i < ModelVertex::MaxColors; ++i)
                {
                    if (const VertexElement* element = elementPointers[(SEM_COLOR << 2) + i])
                    {
                        VertexBuffer::UnpackVertexData(
                            vertexData, vertexSize, *element, index, 1, &v.color_[i], sizeof(Vector4));
                    }
                }

                for (unsigned char i = 0; i < ModelVertex::MaxUVs; ++i)
                {
                    if (const VertexElement* element = elementPointers[(SEM_TEXCOORD << 2) + i])
                    {
                        VertexBuffer::UnpackVertexData(
                            vertexData, vertexSize, *element, index, 1, &v.uv_[i], sizeof(Vector4));
                    }
                }

                // Ignore animation/instancing data (blend weights/indices, per-instance elements).

                verts.push_back(v);
            }

            polygons.emplace_back(ea::move(verts), PackSharedKey(meshId, i));
        }
    }

    return polygons;
}

ea::vector<CsgPolygon> CsgBooleanOperation(const CsgBsp& bspA, const CsgBsp& bspB, CsgOperation op, float epsilon)
{
    ea::vector<CsgPolygon> resultPolys;

    switch (op)
    {
    case CsgOperation::Union:
    {
        auto aOutB = bspB.Clip(bspA.AllPolygons(), CsgClipMode::ClipToOutside, epsilon);
        auto bOutA = bspA.Clip(bspB.AllPolygons(), CsgClipMode::ClipToOutside, epsilon);

        resultPolys.insert(
            resultPolys.end(), std::make_move_iterator(aOutB.begin()), std::make_move_iterator(aOutB.end()));
        resultPolys.insert(
            resultPolys.end(), std::make_move_iterator(bOutA.begin()), std::make_move_iterator(bOutA.end()));
        break;
    }
    case CsgOperation::DifferenceAB:
    {
        auto aOutB = bspB.Clip(bspA.AllPolygons(), CsgClipMode::ClipToOutside, epsilon);
        auto bInA = bspA.Clip(bspB.AllPolygonsInverted(), CsgClipMode::ClipToInside, epsilon);

        resultPolys.insert(
            resultPolys.end(), std::make_move_iterator(aOutB.begin()), std::make_move_iterator(aOutB.end()));
        resultPolys.insert(
            resultPolys.end(), std::make_move_iterator(bInA.begin()), std::make_move_iterator(bInA.end()));
        break;
    }
    case CsgOperation::Intersection:
    {
        auto aInB = bspB.Clip(bspA.AllPolygons(), CsgClipMode::ClipToInside, epsilon);
        auto bInA = bspA.Clip(bspB.AllPolygons(), CsgClipMode::ClipToInside, epsilon);

        resultPolys.insert(
            resultPolys.end(), std::make_move_iterator(aInB.begin()), std::make_move_iterator(aInB.end()));
        resultPolys.insert(
            resultPolys.end(), std::make_move_iterator(bInA.begin()), std::make_move_iterator(bInA.end()));
        break;
    }
    }

    return resultPolys;
}

CsgBsp CsgBuildBspFromModel(const Model* model, const Matrix3x4& worldTransform, unsigned meshId, float epsilon)
{
    if (!model)
        return {};

    auto polygons = CsgBuildPolygonsFromModel(model, worldTransform, meshId);
    if (polygons.empty())
        return {};

    CsgBsp result;
    result.Build(ea::move(polygons), epsilon);
    return result;
}

SharedPtr<Model> CsgBuildModel(Context* context, const CsgTriangulatedModel& triangulated,
    ea::span<const Model* const> sourceModels, ea::span<const ResourceRefList* const> sourceMaterials,
    ResourceRefList* outMaterials)
{
    if (outMaterials)
    {
        outMaterials->type_ = Material::GetTypeStatic();
        outMaterials->names_.clear();
    }

    if (triangulated.geometries_.empty() || !triangulated.hasAnyVertex_)
        return MakeShared<Model>(context);

    ea::vector<SharedPtr<VertexBuffer>> modelVertexBuffers;
    ea::vector<SharedPtr<IndexBuffer>> modelIndexBuffers;
    ea::vector<SharedPtr<Geometry>> modelGeometries;
    modelVertexBuffers.reserve(triangulated.geometries_.size());
    modelIndexBuffers.reserve(triangulated.geometries_.size());
    modelGeometries.reserve(triangulated.geometries_.size());

    if (outMaterials)
        outMaterials->names_.reserve(triangulated.geometries_.size());

    for (const auto& src : triangulated.geometries_)
    {
        const unsigned vertexCount = src.vertices_.size();
        if (vertexCount == 0 || src.indexData_.empty())
            continue;

        const auto [meshId, geometryIndex] = UnpackSharedKey(src.shared_);

        const Model* srcModel = meshId < sourceModels.size() ? sourceModels[meshId] : nullptr;
        Geometry* srcGeometry = srcModel ? srcModel->GetGeometry(geometryIndex, 0) : nullptr;
        ea::vector<VertexElement> vbElements = GetSupportedVertexElements(srcGeometry);

        auto vb = MakeShared<VertexBuffer>(context);
        vb->SetShadowed(true);
        if (!vb->SetSize(vertexCount, vbElements))
            continue;

        const ByteVector packedVertices = PackVerticesToLayout(src.vertices_, vbElements);
        vb->Update(packedVertices.data());

        const bool largeIndices = vertexCount > 0xFFFFu;
        auto ib = MakeShared<IndexBuffer>(context);
        ib->SetShadowed(true);
        ib->SetSize(src.indexData_.size(), largeIndices);
        ib->SetUnpackedData(src.indexData_.data(), 0, src.indexData_.size());

        auto geom = MakeShared<Geometry>(context);
        geom->SetNumVertexBuffers(1);
        geom->SetVertexBuffer(0, vb);
        geom->SetIndexBuffer(ib);
        geom->SetDrawRange(TRIANGLE_LIST, 0, ib->GetIndexCount(), 0, vb->GetVertexCount());

        modelVertexBuffers.push_back(vb);
        modelIndexBuffers.push_back(ib);
        modelGeometries.push_back(geom);

        if (outMaterials)
        {
            const ResourceRefList* srcMatList = meshId < sourceMaterials.size() ? sourceMaterials[meshId] : nullptr;
            if (srcMatList && geometryIndex < srcMatList->names_.size())
                outMaterials->names_.push_back(srcMatList->names_[geometryIndex]);
            else
                outMaterials->names_.push_back(ea::string{});
        }
    }

    if (modelGeometries.empty())
        return MakeShared<Model>(context);

    auto outModel = MakeShared<Model>(context);
    outModel->SetVertexBuffers(modelVertexBuffers, {}, {});
    outModel->SetIndexBuffers(modelIndexBuffers);
    outModel->SetNumGeometries(modelGeometries.size());
    for (unsigned i = 0; i < modelGeometries.size(); ++i)
        outModel->SetGeometry(i, 0, modelGeometries[i]);
    outModel->SetBoundingBox(triangulated.boundingBox_);

    return outModel;
}

void RegisterCsgLibrary(Context* context)
{
    CsgComponent::RegisterObject(context);
    CsgBrush::RegisterObject(context);
    CsgRoot::RegisterObject(context);
}

} // namespace Urho3D
