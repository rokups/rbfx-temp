// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/Core/Variant.h"
#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgComponent.h"

namespace Urho3D
{

class CsgRoot;

/// Operand component for component-based CSG workflow.
///
/// High-level behavior:
/// - Represents one operand in a `CsgRoot` boolean operation tree.
/// - The operand geometry is taken from the base `CsgComponent` attributes (`Model` + `Materials`).
/// - The boolean operation to apply against the accumulated result is controlled by `op_`.
///
/// Root association:
/// - A brush registers itself with the nearest containing `CsgRoot` (walking up parent nodes).
/// - The association is updated when the brush is moved between nodes/scenes or enabled/disabled.
///
/// Dirty semantics:
/// - A brush becomes dirty when its operation/model/materials/state changes.
/// - Dirtiness is acknowledged by the root after a successful rebuild.
class URHO3D_API CsgBrush : public CsgComponent
{
    URHO3D_OBJECT(CsgBrush, CsgComponent);

public:
    explicit CsgBrush(Context* context);
    ~CsgBrush() override;

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    /// Get boolean operation used when combining this brush with the current CSG result.
    CsgOperation GetOperation() const { return operation_; }
    /// Set boolean operation used when combining this brush with the current CSG result.
    void SetOperation(CsgOperation op);
    /// Whether brush state has changed since last acknowledged by a root.
    bool IsDirty() const { return dirty_; }
    /// Acknowledge brush state change.
    void ClearDirty() { dirty_ = false; }

protected:
    const ResourceRef& GetDebugModelAttr() const override { return GetModelAttr(); }

    void ApplyAttributes() override;
    void OnSetEnabled() override;
    void OnNodeSet(Node* previousNode, Node* currentNode) override;
    void OnSceneSet(Scene* previousScene, Scene* scene) override;
    void OnModelChanged() override;
    void OnMaterialsChanged() override;

private:
    void MarkDirty();
    CsgRoot* FindContainingRoot() const;
    void UpdateRootRegistration();

    CsgOperation operation_{CsgOperation::Union};
    WeakPtr<CsgRoot> root_;
    bool dirty_{};
};

} // namespace Urho3D
