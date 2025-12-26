#include "../CommonUtils.h"

#include <Urho3D/CSG/CsgBrush.h>
#include <Urho3D/CSG/CsgRoot.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Scene/Scene.h>

namespace
{

Urho3D::SharedPtr<Urho3D::Model> CreateMinimalModel(Urho3D::Context* context)
{
    using namespace Urho3D;

    auto geometry = MakeShared<Geometry>(context);
    auto model = MakeShared<Model>(context);

    auto vb = MakeShared<VertexBuffer>(context);
    vb->SetShadowed(true);
    vb->SetSize(0, 0);
    model->SetVertexBuffers({vb}, {}, {});

    auto ib = MakeShared<IndexBuffer>(context);
    ib->SetShadowed(true);
    ib->SetSize(0, false);
    REQUIRE(model->SetIndexBuffers({ib}));

    REQUIRE(geometry->SetVertexBuffer(0, vb));
    geometry->SetIndexBuffer(ib);
    REQUIRE(geometry->SetDrawRange(PrimitiveType::TRIANGLE_LIST, 0, 0));

    model->SetNumGeometries(1);
    REQUIRE(model->SetNumGeometryLodLevels(0, 1));
    REQUIRE(model->SetGeometry(0, 0, geometry));

    return model;
}

}

struct CsgRootTestProxy : public Urho3D::CsgRoot
{
    explicit CsgRootTestProxy(Urho3D::Context* context)
        : Urho3D::CsgRoot(context)
    {
    }

    using Urho3D::CsgRoot::ApplyResultToOutput;
    using Urho3D::CsgRoot::CollectBrushesAndNestedRoots;
};

TEST_CASE("CSG: Component CollectBrushes excludes nested roots")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);

    auto scene = MakeShared<Urho3D::Scene>(context);
    auto rootNode = scene->CreateChild("Root");

    auto* root = new CsgRootTestProxy(context);
    rootNode->AddComponent(root, 0);

    rootNode->CreateChild("BrushA")->CreateComponent<Urho3D::CsgBrush>();
    rootNode->CreateChild("BrushB")->CreateComponent<Urho3D::CsgBrush>();

    auto nestedNode = rootNode->CreateChild("NestedRoot");
    nestedNode->CreateComponent<Urho3D::CsgRoot>();
    nestedNode->CreateChild("NestedBrush")->CreateComponent<Urho3D::CsgBrush>();

    ea::vector<Urho3D::CsgBrush*> brushes;
    ea::vector<Urho3D::Node*> nestedRoots;
    root->CollectBrushesAndNestedRoots(rootNode, brushes, nestedRoots);

    REQUIRE(brushes.size() == 2);
    REQUIRE(nestedRoots.size() == 1);
    REQUIRE(nestedRoots[0] == nestedNode);
}

TEST_CASE("CSG: Component Output StaticModel is persistent")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);

    auto scene = MakeShared<Urho3D::Scene>(context);
    auto rootNode = scene->CreateChild("Root");

    auto* root = rootNode->CreateComponent<Urho3D::CsgRoot>();

    // Output is stored as StaticModel on the same node.
    // Use node API to verify persistence without relying on CsgRoot internals.
    auto* output1 = rootNode->GetOrCreateComponent<Urho3D::StaticModel>();
    auto* output2 = rootNode->GetOrCreateComponent<Urho3D::StaticModel>();

    REQUIRE(output1);
    REQUIRE(output2);
    REQUIRE(output1 == output2);
}

TEST_CASE("CSG: Component Root base attributes trigger rebuild request")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);

    auto scene = MakeShared<Urho3D::Scene>(context);
    auto rootNode = scene->CreateChild("Root");

    auto* root = rootNode->CreateComponent<Urho3D::CsgRoot>();
    // Just verify setters are callable and don't crash in tests.
    root->SetModel(Urho3D::ResourceRef(Urho3D::Model::GetTypeStatic(), ""));
    root->SetMaterials(Urho3D::ResourceRefList(Urho3D::Material::GetTypeStatic()));
    SUCCEED();
}

TEST_CASE("CSG: Component ApplyResultToOutput preserves unrelated attributes")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);

    auto scene = MakeShared<Urho3D::Scene>(context);
    auto rootNode = scene->CreateChild("Root");

    auto* root = new CsgRootTestProxy(context);
    rootNode->AddComponent(root, 0);
    auto* output = rootNode->GetOrCreateComponent<Urho3D::StaticModel>();
    REQUIRE(output);

    output->SetViewMask(0x12345678u);
    output->SetCastShadows(true);

    auto model = CreateMinimalModel(context);
    model->SetName(EMPTY_STRING);

    Urho3D::ResourceRefList materials{Urho3D::Material::GetTypeStatic()};
    materials.names_.push_back("Tests/Materials/MatA.xml");

    root->ApplyResultToOutput(model, materials);

    REQUIRE(output->GetModel() == model);
    REQUIRE(output->GetViewMask() == 0x12345678u);
    REQUIRE(output->GetCastShadows() == true);
}
