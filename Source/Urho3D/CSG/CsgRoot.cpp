// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "../Precompiled.h"

#include "Urho3D/CSG/CsgRoot.h"

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBrush.h"
#include "Urho3D/CSG/CsgBsp.h"
#include "Urho3D/Core/Context.h"
#include "Urho3D/Core/Timer.h"
#include "Urho3D/Core/WorkQueue.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Graphics/Material.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/Graphics/StaticModel.h"
#include "Urho3D/IO/Log.h"
#include "Urho3D/Resource/ResourceCache.h"
#include "Urho3D/Scene/Node.h"
#include "Urho3D/Scene/Scene.h"

#include <memory>

namespace Urho3D
{

struct CsgRoot::BrushStep
{
    ea::vector<CsgPolygon> polygons_;

    SharedPtr<Model> model_;
    ResourceRefList materials_;
    unsigned meshId_{};
    CsgOperation op_{};
    WeakPtr<CsgBrush> brush_;
    ea::vector<BrushStep> children_;
};

struct CsgRoot::BuildJob
{
    WeakPtr<CsgRoot> root_;
    Context* context_{};
    unsigned buildId_{};
    float epsilon_{};

    SharedPtr<Model> baseModel_;
    ResourceRefList baseMaterials_;
    BrushStep rootStep_;

    ea::vector<SharedPtr<Model>> sourceModels_;
    ea::vector<ResourceRefList> sourceMaterials_;
};

namespace
{

Node* GetOrCreateTemporaryChild(Node* parent, const ea::string& name)
{
    if (!parent)
        return nullptr;

    if (Node* existing = parent->GetChild(StringHash(name), false))
        return existing;

    Node* child = parent->CreateTemporaryChild(name);
    child->SetEnabled(false);
    return child;
}

void SetStaticModelMaterials(StaticModel* staticModel, const ea::vector<SharedPtr<Material>>& materials)
{
    if (!staticModel)
        return;

    const unsigned numGeometries = staticModel->GetNumGeometries();
    for (unsigned i = 0; i < numGeometries; ++i)
    {
        Material* material = i < materials.size() ? materials[i].Get() : nullptr;
        staticModel->SetMaterial(i, material);
    }
}

} // namespace

CsgRoot::CsgRoot(Context* context)
    : CsgComponent(context)
{
}

void CsgRoot::RegisterObject(Context* context)
{
    context->AddFactoryReflection<CsgRoot>(Category_Scene);

    URHO3D_ATTRIBUTE("Deferred Delay Sec", float, deferredDelaySec_, 0.1f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Epsilon", float, epsilon_, CSG_DEFAULT_EPSILON, AM_DEFAULT);
    URHO3D_COPY_BASE_ATTRIBUTES(CsgComponent);
    URHO3D_ACTION_STATIC_LABEL("Rebuild", Rebuild, "Rebuild component CSG now");
}

void CsgRoot::ApplyAttributes()
{
    Component::ApplyAttributes();

    // Node::Load calls ApplyAttributes() after children/components exist.
    // Generated output model is not serialized as a resource reference, so we must rebuild after load.
    RequestRebuild();
}

void CsgRoot::OnSetEnabled()
{
    Component::OnSetEnabled();
    UpdateSubscriptions();
}

void CsgRoot::OnNodeSet(Node* previousNode, Node* currentNode)
{
    Component::OnNodeSet(previousNode, currentNode);
    UpdateSubscriptions();
}

void CsgRoot::OnSceneSet(Scene* previousScene, Scene* scene)
{
    Component::OnSceneSet(previousScene, scene);
    UpdateSubscriptions();
}

void CsgRoot::OnMarkedDirty(Node* node)
{
    // Called by Node::MarkDirty when any listened node becomes dirty.
    // This is the most reliable way to catch editor-driven transform changes.
    if (!IsEnabledEffective())
        return;

    rebuildDirty_ = true;
    ++rebuildRequestId_;
    lastChangeTimeMs_ = Time::GetSystemTime();

    ScheduleDeferredRebuild();
}

void CsgRoot::OnModelChanged()
{
    BaseClassName::OnModelChanged();
    RequestRebuild();
}

void CsgRoot::OnMaterialsChanged()
{
    BaseClassName::OnMaterialsChanged();
    RequestRebuild();
}

void CsgRoot::SetDeferredDelaySec(float value)
{
    deferredDelaySec_ = ea::max(0.0f, value);
}

void CsgRoot::SetEpsilon(float value)
{
    epsilon_ = ea::max(0.0f, value);
    RequestRebuild();
}

void CsgRoot::RequestRebuild()
{
    rebuildDirty_ = true;

    ++rebuildRequestId_;

    lastChangeTimeMs_ = Time::GetSystemTime();

    ScheduleDeferredRebuild();
}

void CsgRoot::RegisterBrush(CsgBrush* brush)
{
    RegisterBrushInternal(brush, true);
}

void CsgRoot::UnregisterBrush(CsgBrush* brush)
{
    UnregisterBrushInternal(brush);
}

StaticModel* CsgRoot::GetOrCreateOutputStaticModel()
{
    if (!node_)
        return nullptr;

    if (output_)
        return output_;

    output_ = node_->GetOrCreateComponent<StaticModel>();
    return output_;
}

void CsgRoot::CollectBrushesAndNestedRoots(
    Node* rootNode, ea::vector<CsgBrush*>& outBrushes, ea::vector<Node*>& outNestedRootNodes) const
{
    outBrushes.clear();
    outNestedRootNodes.clear();

    if (!rootNode)
        return;

    ea::vector<Node*> stack;
    stack.push_back(rootNode);

    while (!stack.empty())
    {
        Node* node = stack.back();
        stack.pop_back();

        if (node != rootNode)
        {
            if (CsgRoot* nestedRoot = node->GetComponent<CsgRoot>())
            {
                if (nestedRoot != this)
                {
                    outNestedRootNodes.push_back(node);
                    continue;
                }
            }
        }

        if (CsgBrush* brush = node->GetComponent<CsgBrush>())
        {
            if (brush->IsEnabledEffective())
                outBrushes.push_back(brush);
        }

        const unsigned numChildren = node->GetNumChildren();
        // Preserve child order deterministically for stable operand ordering.
        for (unsigned i = numChildren; i-- > 0;)
        {
            Node* child = node->GetChild(i);
            if (child && child->IsEnabled())
                stack.push_back(child);
        }
    }
}

void CsgRoot::ApplyResultToOutput(Model* model, const ResourceRefList& materials)
{
    StaticModel* output = GetOrCreateOutputStaticModel();
    if (!output)
        return;

    output->SetModel(model);

    output->SetMaterialsAttr(materials);
}

void CsgRoot::UpdateSubscriptions()
{
    Scene* scene = GetScene();

    const bool isActive = IsEnabledEffective() && scene;

    // Always detach any previously attached listeners.
    for (const WeakPtr<Node>& weakNode : listenedNodes_)
    {
        if (Node* oldNode = weakNode)
            oldNode->RemoveListener(this);
    }
    listenedNodes_.clear();

    if (!isActive)
    {
        return;
    }

    PruneBrushes();

    // Rebuild listener set from registered brushes.
    listenedNodes_.reserve(brushes_.size());

    for (const WeakPtr<CsgBrush>& weakBrush : brushes_)
    {
        CsgBrush* brush = weakBrush;
        if (!brush || !brush->IsEnabledEffective())
            continue;

        Node* brushNode = brush->GetNode();
        if (!brushNode)
            continue;

        brushNode->AddListener(this);

        bool alreadyTracked = false;
        for (const WeakPtr<Node>& existing : listenedNodes_)
        {
            if (existing == brushNode)
            {
                alreadyTracked = true;
                break;
            }
        }
        if (!alreadyTracked)
            listenedNodes_.push_back(WeakPtr<Node>(brushNode));
    }
}

void CsgRoot::RegisterBrushInternal(CsgBrush* brush, bool requestRebuild)
{
    if (!brush)
        return;

    PruneBrushes();

    for (const WeakPtr<CsgBrush>& weakBrush : brushes_)
    {
        if (weakBrush == brush)
            return;
    }

    brushes_.push_back(WeakPtr<CsgBrush>(brush));

    if (IsEnabledEffective() && GetScene())
    {
        if (Node* brushNode = brush->GetNode())
        {
            brushNode->AddListener(this);

            bool alreadyTracked = false;
            for (const WeakPtr<Node>& existing : listenedNodes_)
            {
                if (existing == brushNode)
                {
                    alreadyTracked = true;
                    break;
                }
            }
            if (!alreadyTracked)
                listenedNodes_.push_back(WeakPtr<Node>(brushNode));
        }
    }

    if (requestRebuild)
        RequestRebuild();
}

void CsgRoot::UnregisterBrushInternal(CsgBrush* brush)
{
    if (!brush)
        return;

    Node* brushNode = brush->GetNode();
    if (brushNode)
        brushNode->RemoveListener(this);

    for (unsigned i = 0; i < brushes_.size();)
    {
        CsgBrush* existing = brushes_[i];
        if (!existing || existing == brush)
            brushes_.erase(brushes_.begin() + i);
        else
            ++i;
    }

    for (unsigned i = 0; i < listenedNodes_.size();)
    {
        Node* existing = listenedNodes_[i];
        if (!existing || existing == brushNode)
            listenedNodes_.erase(listenedNodes_.begin() + i);
        else
            ++i;
    }

    RequestRebuild();
}

void CsgRoot::PruneBrushes()
{
    for (unsigned i = 0; i < brushes_.size();)
    {
        if (!brushes_[i])
            brushes_.erase(brushes_.begin() + i);
        else
            ++i;
    }

    for (unsigned i = 0; i < listenedNodes_.size();)
    {
        if (!listenedNodes_[i])
            listenedNodes_.erase(listenedNodes_.begin() + i);
        else
            ++i;
    }
}

void CsgRoot::ScheduleDeferredRebuild()
{
    if (!IsEnabledEffective() || !node_)
        return;

    // If base model is not set, avoid scheduling rebuilds that can never succeed.
    if (GetModelAttr().name_.empty())
        return;

    WorkQueue* queue = GetSubsystem<WorkQueue>();
    if (!queue)
        return;

    if (deferredDelaySec_ <= 0.0f)
    {
        Rebuild();
        return;
    }

    if (deferredRebuildScheduled_)
        return;

    deferredRebuildScheduled_ = true;
    const unsigned scheduleToken = ++deferredScheduleToken_;

    queue->PostDelayedTaskForMainThread([weak = WeakPtr<CsgRoot>(this), scheduleToken](unsigned, WorkQueue* queue)
    {
        if (CsgRoot* root = weak)
            root->DeferredRebuildTick(scheduleToken, queue);
    });
}

void CsgRoot::DeferredRebuildTick(unsigned scheduleToken, WorkQueue* queue)
{
    if (scheduleToken != deferredScheduleToken_)
        return;

    if (!IsEnabledEffective() || !node_)
    {
        deferredRebuildScheduled_ = false;
        return;
    }

    if (!rebuildDirty_ || GetModelAttr().name_.empty())
    {
        deferredRebuildScheduled_ = false;
        return;
    }

    const unsigned nowMs = Time::GetSystemTime();
    const unsigned delayMs = static_cast<unsigned>(deferredDelaySec_ * 1000.0f);
    if (nowMs - lastChangeTimeMs_ >= delayMs)
    {
        deferredRebuildScheduled_ = false;
        Rebuild();
        return;
    }

    if (!queue)
    {
        deferredRebuildScheduled_ = false;
        return;
    }

    queue->PostDelayedTaskForMainThread([weak = WeakPtr<CsgRoot>(this), scheduleToken](unsigned, WorkQueue* queue)
    {
        if (CsgRoot* root = weak)
            root->DeferredRebuildTick(scheduleToken, queue);
    });
}

void CsgRoot::Rebuild()
{
    if (!node_)
        return;

    if (rebuildInFlight_)
        return;

    StaticModel* tempB = GetOrCreateTempStaticModel("Operand", tempB_);
    StaticModel* tempBase = GetOrCreateTempStaticModel("Base", tempBase_);

    if (!tempB || !tempBase)
    {
        ApplyResultToOutput(nullptr, {});
        return;
    }

    // Base operand must be provided explicitly on CsgRoot.
    if (GetModelAttr().name_.empty())
    {
        URHO3D_LOGERRORF(
            "CsgRoot (%s:%u) has no <Model> set. CSG build is discarded.", node_->GetName().c_str(), node_->GetID());
        ApplyResultToOutput(nullptr, {});
        return;
    }

    if (!SetupOperandStaticModel(tempBase, GetModelAttr(), GetMaterialsAttr()))
    {
        ApplyResultToOutput(nullptr, {});
        return;
    }

    auto job = std::make_shared<BuildJob>();
    job->root_ = this;
    job->context_ = context_;
    job->buildId_ = rebuildRequestId_;
    job->epsilon_ = epsilon_;
    job->baseModel_ = tempBase->GetModel();
    job->baseMaterials_ = tempBase->GetMaterialsAttr();

    // Prepare source lookup tables for output vertex layout and materials.
    // MeshId 0 is reserved for the base operand.
    job->sourceModels_.clear();
    job->sourceMaterials_.clear();
    job->sourceModels_.push_back(job->baseModel_);
    job->sourceMaterials_.push_back(job->baseMaterials_);

    // Extract base polygons in CsgRoot local space on the main thread.
    job->rootStep_.model_ = job->baseModel_;
    job->rootStep_.materials_ = job->baseMaterials_;
    job->rootStep_.meshId_ = 0;
    job->rootStep_.polygons_ = CsgBuildPolygonsFromModel(job->baseModel_.Get(), Matrix3x4::IDENTITY, 0);

    // Build operation tree recursively starting from root node.
    const Matrix3x4 invRootWorld = node_->GetWorldTransform().Inverse();
    if (!BuildOperationTree(job->rootStep_, node_, invRootWorld, tempB, job->sourceModels_, job->sourceMaterials_))
    {
        // No operations: return base model directly.
        const ResourceRefList baseMaterials = tempBase->GetMaterialsAttr();
        ApplyResultToOutput(tempBase->GetModel(), baseMaterials);
        rebuildDirty_ = false;
        return;
    }

    WorkQueue* workQueue = GetSubsystem<WorkQueue>();
    rebuildInFlight_ = true;
    rebuildInFlightId_ = job->buildId_;
    rebuildDirty_ = false;

    workQueue->PostTask([job](unsigned threadIndex, WorkQueue* queue) { ExecuteBuildJob(threadIndex, queue, job); },
        TaskPriority::High);
}

void CsgRoot::ExecuteBuildJob(unsigned threadIndex, WorkQueue* queue, const std::shared_ptr<BuildJob>& job)
{
    (void)threadIndex;

    ea::vector<CsgPolygon> resultPolygons = job->rootStep_.polygons_;

    HiresTimer timer;
    EvaluateOperationTree(job->rootStep_, resultPolygons, job->epsilon_);
    SharedPtr<CsgTriangulatedModel> triangulated;
    if (!resultPolygons.empty())
        triangulated = CsgTriangulatePolygons(ea::move(resultPolygons), job->epsilon_);
    URHO3D_LOGDEBUG("CSG rebuild took {}.{}ms", timer.GetUSec() / 1000, timer.GetUSec() % 1000);

    queue->PostTaskForMainThread([job, triangulated = ea::move(triangulated)](unsigned, WorkQueue*) mutable
    { ApplyBuildJobResultOnMainThread(job, ea::move(triangulated)); }, TaskPriority::High);
}

void CsgRoot::ApplyBuildJobResultOnMainThread(
    const std::shared_ptr<BuildJob>& job, SharedPtr<CsgTriangulatedModel> triangulated)
{
    CsgRoot* root = job->root_;
    if (!root)
        return;

    if (root->rebuildInFlightId_ != job->buildId_)
        return;

    root->rebuildInFlight_ = false;

    // Drop stale results.
    if (root->rebuildRequestId_ != job->buildId_)
    {
        if (root->rebuildDirty_)
            root->Rebuild();
        return;
    }

    ResourceRefList materials;
    SharedPtr<Model> renderableModel;
    if (!triangulated)
    {
        // Triangulation failed or resulted in empty output.
        renderableModel = MakeShared<Model>(root->context_);
    }
    else
    {
        ea::vector<const Model*> sourceModels;
        sourceModels.reserve(job->sourceModels_.size());
        for (const SharedPtr<Model>& model : job->sourceModels_)
            sourceModels.push_back(model.Get());

        ea::vector<const ResourceRefList*> sourceMaterials;
        sourceMaterials.reserve(job->sourceMaterials_.size());
        for (const ResourceRefList& mats : job->sourceMaterials_)
            sourceMaterials.push_back(&mats);

        renderableModel = CsgBuildModel(root->context_, *triangulated,
            ea::span<const Model* const>(sourceModels.data(), sourceModels.size()),
            ea::span<const ResourceRefList* const>(sourceMaterials.data(), sourceMaterials.size()), &materials,
            ModelViewExportFlag::None);
    }

    root->ApplyResultToOutput(renderableModel, materials);

    // Acknowledge changes.
    root->AcknowledgeBrushes(job->rootStep_);
}

StaticModel* CsgRoot::GetOrCreateTempStaticModel(const ea::string& name, WeakPtr<StaticModel>& cache)
{
    if (!node_)
        return nullptr;

    Node* tempParent = tempNode_;
    if (!tempParent)
    {
        tempParent = node_->CreateTemporaryChild("__CsgRootTemp");
        tempParent->SetEnabled(false);
        tempNode_ = tempParent;
    }

    Node* child = GetOrCreateTemporaryChild(tempParent, name);

    if (!cache)
    {
        cache = child->GetOrCreateComponent<StaticModel>();
        cache->SetTemporary(true);
    }

    return cache;
}

bool CsgRoot::SetupOperandStaticModel(
    StaticModel* staticModel, const ResourceRef& modelAttr, const ResourceRefList& materialsAttr) const
{
    if (!staticModel)
        return false;

    const ea::string& modelName = modelAttr.name_;
    if (modelName.empty())
        return false;

    auto* cache = GetSubsystem<ResourceCache>();
    Model* model = cache->GetResource<Model>(modelName);
    if (!model)
        return false;

    staticModel->SetModel(model);

    // Material fallback policy:
    // If material list is empty, treat it as "use the model's default materials".
    // StaticModel::ApplyMaterialList() uses the model's resource name with extension .txt by default.
    if (materialsAttr.names_.empty())
        staticModel->ApplyMaterialList();
    else
        staticModel->SetMaterialsAttr(materialsAttr);

    return true;
}

bool CsgRoot::SetupOperandStaticModel(StaticModel* staticModel, const CsgBrush* brush) const
{
    if (!staticModel || !brush)
        return false;

    return SetupOperandStaticModel(staticModel, brush->GetModelAttr(), brush->GetMaterialsAttr());
}

bool CsgRoot::BuildOperationTree(BrushStep& rootStep, Node* node, const Matrix3x4& invRootWorld, StaticModel* tempModel,
    ea::vector<SharedPtr<Model>>& sourceModels, ea::vector<ResourceRefList>& sourceMaterials)
{

    if (!node)
        return false;

    const unsigned numChildren = node->GetNumChildren();
    for (unsigned i = 0; i < numChildren; ++i)
    {
        Node* child = node->GetChild(i);
        if (!child || !child->IsEnabled())
            continue;

        // Ignore nested CsgRoots (except this one).
        if (CsgRoot* nestedRoot = child->GetComponent<CsgRoot>())
        {
            if (nestedRoot != this)
                continue;
        }

        if (CsgBrush* brush = child->GetComponent<CsgBrush>())
        {
            if (!brush->IsEnabledEffective())
                continue;

            // Ensure we listen to this brush even if it wasn't explicitly registered yet.
            RegisterBrushInternal(brush, false);

            if (!SetupOperandStaticModel(tempModel, brush))
                continue;

            BrushStep step;
            step.model_ = tempModel->GetModel();

            step.materials_ = tempModel->GetMaterialsAttr();
            step.meshId_ = sourceModels.size();
            step.op_ = brush->GetOperation();
            step.brush_ = brush;

            // Store source metadata for final model build on the main thread.
            sourceModels.push_back(step.model_);
            sourceMaterials.push_back(step.materials_);

            // Extract polygons in CsgRoot local space (main thread).
            const Matrix3x4 brushToRoot = invRootWorld * child->GetWorldTransform();
            step.polygons_ = CsgBuildPolygonsFromModel(step.model_.Get(), brushToRoot, step.meshId_);

            // Recurse into the brush subtree.
            BuildOperationTree(step, child, invRootWorld, tempModel, sourceModels, sourceMaterials);
            rootStep.children_.push_back(ea::move(step));
        }
        else
        {
            // Recurse into nodes without brushes.
            BuildOperationTree(rootStep, child, invRootWorld, tempModel, sourceModels, sourceMaterials);
        }
    }

    return !rootStep.children_.empty();
}

static ea::vector<CsgPolygon> EvaluateBooleanPolygons(
    ea::vector<CsgPolygon> a, ea::vector<CsgPolygon> b, CsgOperation op, float epsilon)
{
    // Handle empty operands explicitly to avoid treating "empty BSP" as a hard failure.
    if (a.empty() && b.empty())
        return {};

    if (a.empty())
    {
        switch (op)
        {
        case CsgOperation::Union: return b;
        case CsgOperation::DifferenceAB: return {};
        case CsgOperation::Intersection: return {};
        }
    }

    if (b.empty())
    {
        switch (op)
        {
        case CsgOperation::Union: return a;
        case CsgOperation::DifferenceAB: return a;
        case CsgOperation::Intersection: return {};
        }
    }

    CsgBsp bspA;
    bspA.Build(ea::move(a), epsilon);
    CsgBsp bspB;
    bspB.Build(ea::move(b), epsilon);
    if (!bspA || !bspB)
        return {};

    return CsgBooleanOperation(bspA, bspB, op, epsilon);
}

void CsgRoot::EvaluateOperationTree(const BrushStep& step, ea::vector<CsgPolygon>& currentPolygons, float epsilon)
{
    for (const BrushStep& child : step.children_)
    {
        ea::vector<CsgPolygon> childPolygons = child.polygons_;
        if (!child.children_.empty())
            EvaluateOperationTree(child, childPolygons, epsilon);

        currentPolygons =
            EvaluateBooleanPolygons(ea::move(currentPolygons), ea::move(childPolygons), child.op_, epsilon);
    }
}

void CsgRoot::AcknowledgeBrushes(const BrushStep& step)
{
    for (const BrushStep& child : step.children_)
    {
        if (CsgBrush* brush = child.brush_)
            brush->ClearDirty();

        AcknowledgeBrushes(child);
    }
}

} // namespace Urho3D
