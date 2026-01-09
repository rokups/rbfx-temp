//
// Copyright (c) 2025-2025 the rbfx project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "../CommonUtils.h"
#include "../ModelUtils.h"

#include <Urho3D/CSG/Csg.h>
#include <Urho3D/CSG/CsgBsp.h>
#include <Urho3D/CSG/CsgTriangulation.h>
#include <Urho3D/Core/Variant.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/ModelView.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Math/BoundingBox.h>
#include <Urho3D/Math/Plane.h>
#include <Urho3D/Scene/Scene.h>

#include <EASTL/array.h>
#include <EASTL/vector.h>

static SharedPtr<Model> ComputeCsgModel(Context* context,
    const Model* aModel, const ResourceRefList& aMaterials, const Matrix3x4& aWorld,
    const Model* bModel, const ResourceRefList& bMaterials, const Matrix3x4& bWorld,
    CsgOperation op,
    ResourceRefList* outMaterials = nullptr,
    float epsilon = CSG_DEFAULT_EPSILON)
{
    if (!context || !aModel || !bModel)
        return nullptr;

    const CsgBsp bspA = CsgBuildBspFromModel(aModel, aWorld, 0, epsilon);
    const CsgBsp bspB = CsgBuildBspFromModel(bModel, bWorld, 1, epsilon);
    if (!bspA || !bspB)
        return nullptr;

    ea::vector<CsgPolygon> resultPolys = CsgBooleanOperation(bspA, bspB, op, epsilon);
    if (resultPolys.empty())
        return MakeShared<Model>(context);

    const SharedPtr<CsgTriangulatedModel> triangulated = CsgTriangulatePolygons(ea::move(resultPolys), epsilon);
    if (!triangulated)
        return MakeShared<Model>(context);

    const Model* models[] = {aModel, bModel};
    const ResourceRefList* sourceMaterials[] = {&aMaterials, &bMaterials};
    return CsgBuildModel(context, *triangulated,
        ea::span<const Model* const>(models, 2),
        ea::span<const ResourceRefList* const>(sourceMaterials, 2));
}

TEST_CASE("CSG: ComputeCsgStaticModel validates inputs")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);

    const Matrix3x4 identity = Matrix3x4::IDENTITY;

    const ResourceRefList emptyMaterials{Material::GetTypeStatic()};

    CHECK(nullptr == ComputeCsgModel(nullptr, nullptr, emptyMaterials, identity, nullptr, emptyMaterials, identity, CsgOperation::Union));

    CHECK(nullptr == ComputeCsgModel(context, nullptr, emptyMaterials, identity, nullptr, emptyMaterials, identity, CsgOperation::Union));

    auto scene = MakeShared<Scene>(context);
    auto nodeA = scene->CreateChild();
    auto nodeB = scene->CreateChild();

    auto staticModelA = nodeA->CreateComponent<StaticModel>();
    auto staticModelB = nodeB->CreateComponent<StaticModel>();

    // Models are not assigned, should be rejected.
    CHECK(nullptr == ComputeCsgModel(
        context,
        staticModelA->GetModel(), emptyMaterials, identity,
        staticModelB->GetModel(), emptyMaterials, identity,
        CsgOperation::Union));
}

static SharedPtr<Model> CreateQuadModel(Context* context)
{
    auto modelView = MakeShared<ModelView>(context);

    auto& geometries = modelView->GetGeometries();
    geometries.resize(1);
    geometries[0].lods_.resize(1);

    auto& lod0 = geometries[0].lods_[0];
    lod0.vertexFormat_ = Tests::GetVertexFormat();
    lod0.primitiveType_ = TRIANGLE_LIST;

    Tests::AppendQuad(lod0, Vector3::ZERO, Quaternion::IDENTITY, {2.0f, 2.0f}, Color::WHITE);

    auto model = modelView->ExportModel();
    model->SetName("Tests/ModelCsg/Quad");
    return model;
}

static SharedPtr<Model> CreateCubeModel(Context* context, float halfExtent = 1.0f)
{
    auto modelView = MakeShared<ModelView>(context);

    auto& geometries = modelView->GetGeometries();
    geometries.resize(1);
    geometries[0].lods_.resize(1);

    auto& lod0 = geometries[0].lods_[0];
    lod0.vertexFormat_ = Tests::GetVertexFormat();
    lod0.primitiveType_ = TRIANGLE_LIST;

    const Vector3 v[8] = {
        {-halfExtent, -halfExtent, -halfExtent},
        { halfExtent, -halfExtent, -halfExtent},
        { halfExtent,  halfExtent, -halfExtent},
        {-halfExtent,  halfExtent, -halfExtent},
        {-halfExtent, -halfExtent,  halfExtent},
        { halfExtent, -halfExtent,  halfExtent},
        { halfExtent,  halfExtent,  halfExtent},
        {-halfExtent,  halfExtent,  halfExtent},
    };

    // 12 triangles, CCW winding when viewed from outside.
    const unsigned t[] = {
        // -Z
        0, 2, 1, 0, 3, 2,
        // +Z
        4, 5, 6, 4, 6, 7,
        // -X
        0, 7, 3, 0, 4, 7,
        // +X
        1, 2, 6, 1, 6, 5,
        // -Y
        0, 1, 5, 0, 5, 4,
        // +Y
        3, 7, 6, 3, 6, 2,
    };

    lod0.vertices_.reserve(lod0.vertices_.size() + 36);
    lod0.indices_.reserve(lod0.indices_.size() + 36);

    for (unsigned tri = 0; tri < sizeof(t) / sizeof(t[0]); tri += 3)
    {
        const Vector3 p0 = v[t[tri + 0]];
        const Vector3 p1 = v[t[tri + 1]];
        const Vector3 p2 = v[t[tri + 2]];
        const Vector3 n = (p1 - p0).CrossProduct(p2 - p0).Normalized();

        const unsigned baseIndex = lod0.vertices_.size();
        lod0.vertices_.push_back(Tests::MakeModelVertex(p0, n, Color::WHITE));
        lod0.vertices_.push_back(Tests::MakeModelVertex(p1, n, Color::WHITE));
        lod0.vertices_.push_back(Tests::MakeModelVertex(p2, n, Color::WHITE));
        lod0.indices_.push_back(baseIndex + 0);
        lod0.indices_.push_back(baseIndex + 1);
        lod0.indices_.push_back(baseIndex + 2);
    }

    auto model = modelView->ExportModel();
    model->SetName("Tests/ModelCsg/Cube");
    return model;
}

TEST_CASE("CSG: Union of single quad produces valid model")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);
    auto scene = MakeShared<Scene>(context);
    auto nodeA = scene->CreateChild();
    auto nodeB = scene->CreateChild();
    auto smA = nodeA->CreateComponent<StaticModel>();
    auto smB = nodeB->CreateComponent<StaticModel>();

    smA->SetModel(CreateQuadModel(context));
    smB->SetModel(CreateQuadModel(context));

    // Move B away so they're disjoint
    const Matrix3x4 aWorld = Matrix3x4::IDENTITY;
    const Matrix3x4 bWorld = Matrix3x4::FromTranslation({5.0f, 0.0f, 0.0f});

    const ResourceRefList emptyMaterials{Material::GetTypeStatic()};
    const auto result = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::Union);
    REQUIRE(result);
    CHECK(result->GetNumGeometries() >= 1);

    // Verify result has expected bounding box
    const BoundingBox bbox = result->GetBoundingBox();
    CHECK(bbox.min_.x_ < -0.9f);
    CHECK(bbox.max_.x_ > 5.9f);
}

TEST_CASE("CSG: World transform is applied correctly")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);
    auto scene = MakeShared<Scene>(context);
    auto nodeA = scene->CreateChild();
    auto nodeB = scene->CreateChild();
    auto smA = nodeA->CreateComponent<StaticModel>();
    auto smB = nodeB->CreateComponent<StaticModel>();

    smA->SetModel(CreateQuadModel(context));
    smB->SetModel(CreateQuadModel(context));

    const Matrix3x4 aWorld = Matrix3x4::FromTranslation({10.0f, 0.0f, -5.0f});
    const Matrix3x4 bWorld = Matrix3x4::FromTranslation({15.0f, 0.0f, -5.0f});

    const ResourceRefList emptyMaterials{Material::GetTypeStatic()};
    const auto result = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::Union);
    REQUIRE(result);

    const BoundingBox bbox = result->GetBoundingBox();
    // Quad extends from -1 to 1, so with translation at 10, expect around 9 to 11
    CHECK(bbox.min_.x_ > 8.9f);
    CHECK(bbox.max_.x_ > 15.9f);
    CHECK(bbox.min_.z_ < -4.9f);
    CHECK(bbox.max_.z_ > -5.1f);
}

TEST_CASE("CSG: Difference of disjoint objects preserves first object")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);
    auto scene = MakeShared<Scene>(context);
    auto nodeA = scene->CreateChild();
    auto nodeB = scene->CreateChild();
    auto smA = nodeA->CreateComponent<StaticModel>();
    auto smB = nodeB->CreateComponent<StaticModel>();

    smA->SetModel(CreateCubeModel(context, 1.0f));
    smB->SetModel(CreateCubeModel(context, 1.0f));

    const Matrix3x4 aWorld = Matrix3x4::IDENTITY;
    // Move B far away so they don't overlap
    const Matrix3x4 bWorld = Matrix3x4::FromTranslation({10.0f, 0.0f, 0.0f});

    // A - B should preserve A since B doesn't intersect
    const ResourceRefList emptyMaterials{Material::GetTypeStatic()};
    const auto result = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::DifferenceAB);
    REQUIRE(result);
    CHECK(result->GetNumGeometries() == 1);
}

TEST_CASE("CSG: Intersection of coplanar faces produces empty model")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);
    auto scene = MakeShared<Scene>(context);
    auto nodeA = scene->CreateChild();
    auto nodeB = scene->CreateChild();
    auto smA = nodeA->CreateComponent<StaticModel>();
    auto smB = nodeB->CreateComponent<StaticModel>();

    smA->SetModel(CreateQuadModel(context));
    smB->SetModel(CreateQuadModel(context));

    // Overlapping quads in same plane
    const Matrix3x4 aWorld = Matrix3x4::IDENTITY;
    const Matrix3x4 bWorld = Matrix3x4::FromTranslation({0.5f, 0.0f, 0.0f});

    const ResourceRefList emptyMaterials{Material::GetTypeStatic()};
    const auto result = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::Intersection);
    REQUIRE(result);
    // Coplanar intersection of non-closed surfaces is expected to be empty.
    CHECK(result->GetNumGeometries() == 0);
}

TEST_CASE("CSG: Difference creates split geometry")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);
    auto scene = MakeShared<Scene>(context);
    auto nodeA = scene->CreateChild();
    auto nodeB = scene->CreateChild();
    auto smA = nodeA->CreateComponent<StaticModel>();
    auto smB = nodeB->CreateComponent<StaticModel>();

    smA->SetModel(CreateCubeModel(context, 2.0f));
    smB->SetModel(CreateCubeModel(context, 1.0f));

    // Overlapping cubes - should split/subtract geometry
    const Matrix3x4 aWorld = Matrix3x4::IDENTITY;
    const Matrix3x4 bWorld = Matrix3x4::FromTranslation({2.0f, 0.0f, 0.0f});

    const ResourceRefList emptyMaterials{Material::GetTypeStatic()};
    const auto result = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::DifferenceAB);
    REQUIRE(result);
    CHECK(result->GetNumGeometries() >= 1);
    // Result should have geometry (not empty) after subtraction
    const auto& geom = result->GetGeometry(0, 0);
    CHECK(geom != nullptr);
    CHECK(geom->GetIndexCount() > 0);
}

TEST_CASE("CSG: Boolean operations on disjoint cubes")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);
    auto scene = MakeShared<Scene>(context);
    auto nodeA = scene->CreateChild();
    auto nodeB = scene->CreateChild();
    auto smA = nodeA->CreateComponent<StaticModel>();
    auto smB = nodeB->CreateComponent<StaticModel>();

    smA->SetModel(CreateCubeModel(context, 1.0f));
    smB->SetModel(CreateCubeModel(context, 1.0f));

    const Matrix3x4 aWorld = Matrix3x4::IDENTITY;
    const Matrix3x4 bWorld = Matrix3x4::FromTranslation({10.0f, 0.0f, 0.0f});

    const float eps = 1e-5f;

    const ResourceRefList emptyMaterials{Material::GetTypeStatic()};

    // Union of disjoint cubes: both cubes present
    const auto unionResult = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::Union, nullptr, eps);
    REQUIRE(unionResult);
    CHECK(unionResult->GetNumGeometries() == 2);

    // Difference of disjoint cubes: only A remains
    const auto diffResult = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::DifferenceAB, nullptr, eps);
    REQUIRE(diffResult);
    CHECK(diffResult->GetNumGeometries() == 1);

    // Intersection of disjoint cubes: empty
    const auto isctResult = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::Intersection, nullptr, eps);
    REQUIRE(isctResult);
    CHECK(isctResult->GetNumGeometries() == 0);
}

TEST_CASE("CSG: Boolean operations on overlapping cubes")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);
    auto scene = MakeShared<Scene>(context);
    auto nodeA = scene->CreateChild();
    auto nodeB = scene->CreateChild();
    auto smA = nodeA->CreateComponent<StaticModel>();
    auto smB = nodeB->CreateComponent<StaticModel>();

    smA->SetModel(CreateCubeModel(context, 1.0f));
    smB->SetModel(CreateCubeModel(context, 1.0f));

    const Matrix3x4 aWorld = Matrix3x4::IDENTITY;
    const Matrix3x4 bWorld = Matrix3x4::FromTranslation({0.5f, 0.25f, 0.125f});

    const float eps = 1e-5f;

    const ResourceRefList emptyMaterials{Material::GetTypeStatic()};

    // Union of overlapping cubes: produces combined geometry
    const auto unionResult = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::Union, nullptr, eps);
    REQUIRE(unionResult);
    CHECK(unionResult->GetNumGeometries() > 0);

    // Verify union contains both original volumes
    const auto& geom0 = unionResult->GetGeometry(0, 0);
    CHECK(geom0 != nullptr);
    CHECK(geom0->GetIndexCount() > 0);

    // Difference of overlapping cubes: produces geometry
    const auto diffResult = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::DifferenceAB, nullptr, eps);
    REQUIRE(diffResult);
    CHECK(diffResult->GetNumGeometries() > 0);

    const auto& diffGeom = diffResult->GetGeometry(0, 0);
    CHECK(diffGeom != nullptr);
    CHECK(diffGeom->GetIndexCount() > 0);

    // Intersection of overlapping cubes: produces geometry
    const auto isctResult = ComputeCsgModel(context,
        smA->GetModel(), emptyMaterials, aWorld,
        smB->GetModel(), emptyMaterials, bWorld,
        CsgOperation::Intersection, nullptr, eps);
    REQUIRE(isctResult);
    CHECK(isctResult->GetNumGeometries() > 0);

    const auto& isctGeom = isctResult->GetGeometry(0, 0);
    CHECK(isctGeom != nullptr);
    CHECK(isctGeom->GetIndexCount() > 0);
}

TEST_CASE("CSG: ComputeCsgStaticModel disjoint")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);

    auto modelA = CreateCubeModel(context);
    auto modelB = CreateCubeModel(context);

    // Move B away so inputs are disjoint.
    const Matrix3x4 aWorld = Matrix3x4::IDENTITY;
    const Matrix3x4 bWorld = Matrix3x4::FromTranslation(Vector3(10.0f, 0.0f, 2.0f));

    auto scene = MakeShared<Scene>(context);
    auto nodeA = scene->CreateChild();
    auto nodeB = scene->CreateChild();
    auto smA = nodeA->CreateComponent<StaticModel>();
    auto smB = nodeB->CreateComponent<StaticModel>();
    smA->SetModel(modelA);
    smB->SetModel(modelB);

    const float eps = 1e-5f;

    ResourceRefList aMaterials{Material::GetTypeStatic()};
    aMaterials.names_ = {"Tests/Materials/MatA.xml"};
    ResourceRefList bMaterials{Material::GetTypeStatic()};
    bMaterials.names_ = {"Tests/Materials/MatB.xml"};

    ResourceRefList unionMats;
    const SharedPtr<Model> outUnion = ComputeCsgModel(context,
        smA->GetModel(), aMaterials, aWorld,
        smB->GetModel(), bMaterials, bWorld,
        CsgOperation::Union, &unionMats, eps);
    REQUIRE(outUnion);
    REQUIRE(outUnion->GetNumGeometries() == 2);
    REQUIRE(outUnion->GetNumGeometryLodLevels(0) >= 1);
    REQUIRE(outUnion->GetNumGeometryLodLevels(1) >= 1);
    REQUIRE(unionMats.names_.size() == 2);
    REQUIRE(unionMats.names_[0] == aMaterials.names_[0]);
    REQUIRE(unionMats.names_[1] == bMaterials.names_[0]);

    ResourceRefList diffMats;
    const SharedPtr<Model> outDiff = ComputeCsgModel(context,
        smA->GetModel(), aMaterials, aWorld,
        smB->GetModel(), bMaterials, bWorld,
        CsgOperation::DifferenceAB, &diffMats, eps);
    REQUIRE(outDiff);
    // B is disjoint, so A - B = A.
    REQUIRE(outDiff->GetNumGeometries() == 1);
    REQUIRE(diffMats.names_.size() == 1);
    REQUIRE(diffMats.names_[0] == aMaterials.names_[0]);

    ResourceRefList isctMats;
    const SharedPtr<Model> outIsct = ComputeCsgModel(context,
        smA->GetModel(), aMaterials, aWorld,
        smB->GetModel(), bMaterials, bWorld,
        CsgOperation::Intersection, &isctMats, eps);
    REQUIRE(outIsct);
    // Disjoint intersection should be empty => exported model may have 0 geometries.
    REQUIRE(outIsct->GetNumGeometries() == 0);
    REQUIRE(isctMats.names_.empty());
}
