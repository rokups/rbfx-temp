// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/Core/Variant.h"
#include "Urho3D/Graphics/Material.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/Scene/Component.h"

namespace Urho3D
{

class DebugRenderer;

/// Base class for CSG components with common model and material handling.
/// This is an abstract base class and should not be instantiated directly.
class URHO3D_API CsgComponent : public Component
{
    URHO3D_OBJECT(CsgComponent, Component);

public:
    explicit CsgComponent(Context* context);

    /// Register object factory.
    /// @nobind
    static void RegisterObject(Context* context);

    const ResourceRef& GetModelAttr() const { return modelAttr_; }
    virtual void SetModel(const ResourceRef& value);

    virtual const ResourceRefList& GetMaterialsAttr() const;
    virtual void SetMaterials(const ResourceRefList& value);

protected:
    void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;
    void DrawDebugGeometryInternal(DebugRenderer* debug, bool depthTest, const ResourceRef& modelAttr, const Color& wireColor) const;
    virtual const ResourceRef& GetDebugModelAttr() const = 0;
    virtual void OnModelChanged();
    virtual void OnMaterialsChanged() {}

private:
    void AddBrush();

    ResourceRef modelAttr_{Model::GetTypeStatic()};
    ResourceRefList materialsAttr_{Material::GetTypeStatic()};
};

} // namespace Urho3D
